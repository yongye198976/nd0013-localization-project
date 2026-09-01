
#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>
#include <atomic>
#include <mutex>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
}

void Accuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min( max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

int main(){

	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(2s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

	//Create lidar
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	// CANDO: Can modify lidar values to get different scan resolutions
	lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
	lidar_bp.SetAttribute("rotation_frequency", "60");
	lidar_bp.SetAttribute("points_per_second", "500000");

	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	std::atomic<bool> scan_ready(false);
	std::mutex scan_mutex;
	double scan_timestamp = 0.0;
	Pose scan_true_pose;

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));

	// Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
	if(pcl::io::loadPCDFile("map.pcd", *mapCloud) < 0 || mapCloud->empty()){
		cerr << "Failed to load map.pcd" << endl;
		return 1;
	}
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 
	PointCloudT::Ptr filteredMap(new PointCloudT);
	pcl::VoxelGrid<PointT> mapFilter;
	mapFilter.setInputCloud(mapCloud);
	mapFilter.setLeafSize(0.5f, 0.5f, 0.5f);
	mapFilter.filter(*filteredMap);

	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);
	Pose poseRef(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180));

	lidar->Listen([&scan_ready, &scan_mutex, &scan_timestamp, &scan_true_pose,
			&poseRef, &scanCloud](auto data){
		if(scan_ready.load())
			return;

		std::lock_guard<std::mutex> lock(scan_mutex);
		if(scan_ready.load())
			return;

		auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
		for (const auto& detection : *scan){
			if((detection.x*detection.x + detection.y*detection.y + detection.z*detection.z) > 8.0){
				pclCloud.points.push_back(PointT(detection.x, detection.y, detection.z));
			}
		}
		if(pclCloud.points.size() > 5000){ // CANDO: Can modify this value to get different scan resolutions
			scan_timestamp = scan->GetTimestamp();
			const auto sensor_transform = scan->GetSensorTransform();
			const double sensor_yaw = sensor_transform.rotation.yaw * pi / 180.0;
			scan_true_pose = Pose(
				Point(sensor_transform.location.x + 0.5 * cos(sensor_yaw),
					sensor_transform.location.y + 0.5 * sin(sensor_yaw),
					sensor_transform.location.z - 1.8),
				Rotate(sensor_yaw,
					sensor_transform.rotation.pitch * pi / 180.0,
					sensor_transform.rotation.roll * pi / 180.0)) - poseRef;
			*scanCloud = pclCloud;
			scan_ready.store(true);
		}
	});
	
	double maxError = 0;

	while (!viewer->wasStopped())
  	{
		while(!scan_ready.load()){
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}
		if(refresh_view){
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = Pose(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180)) - poseRef;
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));


		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}

  		viewer->spinOnce ();
		
		if(scan_ready.load()){
			PointCloudT::Ptr stable_scan(new PointCloudT);
			double current_scan_timestamp = 0.0;
			Pose synchronized_true_pose;
			{
				std::lock_guard<std::mutex> lock(scan_mutex);
				*stable_scan = *scanCloud;
				current_scan_timestamp = scan_timestamp;
				synchronized_true_pose = scan_true_pose;
				pclCloud.clear();
				scan_ready.store(false);
			}

			// Downsample the scan before registration.
			pcl::VoxelGrid<PointT> voxel_filter;
			voxel_filter.setInputCloud(stable_scan);
			voxel_filter.setLeafSize(0.5f, 0.5f, 0.5f);
			voxel_filter.filter(*cloudFiltered);
			PointCloudT::Ptr registration_scan(new PointCloudT);
			const Eigen::Matrix4d lidar_to_map = transform2D(pi / 2.0, 0, 0);
			pcl::transformPointCloud(*cloudFiltered, *registration_scan, lidar_to_map);

			// Use vehicle odometry as the motion-model seed for scan matching.
			static double previous_scan_timestamp = current_scan_timestamp;
			const double delta_time = min(
				max(current_scan_timestamp - previous_scan_timestamp, 0.0), 1.0);
			previous_scan_timestamp = current_scan_timestamp;
			const auto velocity = vehicle->GetVelocity();
			const auto angular_velocity = vehicle->GetAngularVelocity();
			const double horizontal_speed = sqrt(
				velocity.x * velocity.x + velocity.y * velocity.y);
			static double previous_velocity_x = velocity.x;
			static double previous_velocity_y = velocity.y;
			static double previous_angular_velocity_z = angular_velocity.z;
			Pose predicted_pose = pose;
			if(horizontal_speed > 0.05){
				predicted_pose.position.x += 0.5
					* (previous_velocity_x + velocity.x) * delta_time;
				predicted_pose.position.y += 0.5
					* (previous_velocity_y + velocity.y) * delta_time;
				predicted_pose.rotation.yaw += 0.5
					* (previous_angular_velocity_z + angular_velocity.z)
					* pi / 180.0 * delta_time;
			}
			previous_velocity_x = velocity.x;
			previous_velocity_y = velocity.y;
			previous_angular_velocity_z = angular_velocity.z;
			Eigen::Matrix4d predicted_transform = transform3D(
				predicted_pose.rotation.yaw, 0, 0,
				predicted_pose.position.x, predicted_pose.position.y, 0);

			// Restrict registration to the part of the map visible to the 30 m LiDAR.
			PointCloudT::Ptr local_map(new PointCloudT);
			pcl::CropBox<PointT> map_crop;
			map_crop.setInputCloud(filteredMap);
			const float predicted_x = static_cast<float>(predicted_transform(0, 3));
			const float predicted_y = static_cast<float>(predicted_transform(1, 3));
			map_crop.setMin(Eigen::Vector4f(predicted_x - 35.0f, predicted_y - 35.0f, -10.0f, 1.0f));
			map_crop.setMax(Eigen::Vector4f(predicted_x + 35.0f, predicted_y + 35.0f, 10.0f, 1.0f));
			map_crop.filter(*local_map);

			double measured_distance = 0;
			pcl::IterativeClosestPoint<PointT, PointT> map_icp;
			map_icp.setMaximumIterations(20);
			map_icp.setMaxCorrespondenceDistance(2.0);
			map_icp.setTransformationEpsilon(0.0001);
			map_icp.setEuclideanFitnessEpsilon(0.001);
			map_icp.setInputSource(registration_scan);
			map_icp.setInputTarget(local_map);
			PointCloudT::Ptr map_aligned(new PointCloudT);
			map_icp.align(*map_aligned, predicted_transform.cast<float>());

			Eigen::Matrix4d estimated_transform = horizontal_speed > 0.05
				? predicted_transform
				: transform3D(pose.rotation.yaw, 0, 0, pose.position.x, pose.position.y, 0);
			if(horizontal_speed > 0.05 && map_icp.hasConverged()){
				Eigen::Matrix4d candidate_transform =
					map_icp.getFinalTransformation().cast<double>();
				Eigen::Matrix4d map_correction = candidate_transform * predicted_transform.inverse();
				double correction_distance = sqrt(
					map_correction(0, 3) * map_correction(0, 3)
					+ map_correction(1, 3) * map_correction(1, 3));
				measured_distance = correction_distance;
				double correction_yaw = abs(atan2(
					map_correction(1, 0), map_correction(0, 0)));
				if(correction_distance < 0.5 && correction_yaw < 0.15
						&& map_icp.getFitnessScore(2.0) < 0.5)
					estimated_transform = candidate_transform;
			}

			Pose estimated_pose = getPose(estimated_transform);
			pose = Pose(Point(estimated_pose.position.x, estimated_pose.position.y, 0),
				Rotate(estimated_pose.rotation.yaw, 0, 0));
			estimated_transform = transform3D(
				pose.rotation.yaw, 0, 0,
				pose.position.x, pose.position.y, 0);
			static int validation_frame = 0;
			static bool validation_finished = false;
			if(validation_frame >= 20 && !validation_finished){
				control.throttle = 0.35f;
				control.steer = 0.0f;
				control.brake = 0.0f;
				control.reverse = false;
				vehicle->ApplyControl(control);
			}
			double validation_error = sqrt(
				(synchronized_true_pose.position.x - pose.position.x) * (synchronized_true_pose.position.x - pose.position.x)
				+ (synchronized_true_pose.position.y - pose.position.y) * (synchronized_true_pose.position.y - pose.position.y));
			static double validation_max_error = 0;
			validation_max_error = max(validation_max_error, validation_error);
			double validation_distance = sqrt(
				synchronized_true_pose.position.x * synchronized_true_pose.position.x
				+ synchronized_true_pose.position.y * synchronized_true_pose.position.y);
			if(validation_frame++ < 60 || validation_frame % 10 == 0){
				cout << "validation frame=" << validation_frame
					<< " distance=" << validation_distance
					<< " relative=" << measured_distance
					<< " pose=(" << pose.position.x << "," << pose.position.y << ")"
					<< " error=" << validation_error
					<< " max=" << validation_max_error << endl;
			}
			if(!validation_finished && validation_distance >= 170.0){
				validation_finished = true;
				cout << "LONG TEST " << (validation_max_error <= 1.2 ? "PASSED" : "FAILED")
					<< ": distance=" << validation_distance
					<< " max_error=" << validation_max_error << endl;
				control.throttle = 0.0f;
				control.brake = 1.0f;
				vehicle->ApplyControl(control);
			}

			// Transform the scan into the map frame for visualization.
			PointCloudT::Ptr transformed_scan(new PointCloudT);
			pcl::transformPointCloud(*registration_scan, *transformed_scan, estimated_transform);

			viewer->removePointCloud("scan");
			renderPointCloud(viewer, transformed_scan, "scan", Color(1,0,0) );

			viewer->removeAllShapes();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
          
		  	double poseError = sqrt(
				(synchronized_true_pose.position.x - pose.position.x) * (synchronized_true_pose.position.x - pose.position.x)
				+ (synchronized_true_pose.position.y - pose.position.y) * (synchronized_true_pose.position.y - pose.position.y));
			if(poseError > maxError)
				maxError = poseError;
			double distDriven = sqrt( (truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y) );
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

			if(maxError > 1.2 || distDriven >= 170.0 ){
				viewer->removeShape("eval");
			if(maxError > 1.2){
				viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
			}
			else{
				viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
			}
		}

		}
  	}
	return 0;
}
