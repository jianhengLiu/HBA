#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "ros/ros.h"
#include <ceres/ceres.h>
#include <math.h>
#include <rosbag/bag.h>

#include "ba.hpp"
#include "mypcl.hpp"
#include "tools.hpp"

using namespace std;
using namespace Eigen;

bool computePairNum(std::string pair1, std::string pair2) {
  std::string num1 = pair1.substr(pair1.find_last_of('/'));
  num1 = num1.substr(num1.find_first_of("0123456789"));
  num1 = num1.substr(0, num1.find_last_of('.'));

  std::string num2 = pair2.substr(pair2.find_last_of('/'));
  num2 = num2.substr(num2.find_first_of("0123456789"));
  num2 = num2.substr(0, num2.find_last_of('.'));
  return std::stod(num1) < std::stod(num2);
}

void sort_filelists(std::vector<std::filesystem::path> &filists) {
  if (filists.empty())
    return;
  std::sort(filists.begin(), filists.end(), computePairNum);
}

void load_file_list(const std::string &dir_path,
                    std::vector<std::filesystem::path> &out_filelsits) {
  for (const auto &entry : std::filesystem::directory_iterator(dir_path)) {
    out_filelsits.emplace_back(entry.path().string());
  }
  sort_filelists(out_filelsits);
  for (auto &name : out_filelsits) {
    std::cout << name << "\n";
  }

  std::cout << "Load " << out_filelsits.size() << " data." << "\n";
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "visualize");
  ros::NodeHandle nh("~");

  ros::Publisher pub_map =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_map", 100);
  ros::Publisher pub_debug =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_debug", 100);
  ros::Publisher pub_pose =
      nh.advertise<geometry_msgs::PoseArray>("/poseArrayTopic", 10);
  ros::Publisher pub_trajectory =
      nh.advertise<visualization_msgs::Marker>("/trajectory_marker", 100);
  ros::Publisher pub_pose_number =
      nh.advertise<visualization_msgs::MarkerArray>("/pose_number", 100);

  string pcl_path, pose_path;
  int pose_type = 0;
  double downsample_size, marker_size;
  int pcd_name_fill_num;
  int output_merge_pcl;

  nh.getParam("pcl_path", pcl_path);
  nh.getParam("pose_path", pose_path);
  nh.getParam("pose_type", pose_type);
  nh.getParam("output_merge_pcl", output_merge_pcl);
  nh.getParam("downsample_size", downsample_size);
  nh.getParam("pcd_name_fill_num", pcd_name_fill_num);
  nh.getParam("marker_size", marker_size);

  sensor_msgs::PointCloud2 debugMsg, cloudMsg, outMsg;
  vector<mypcl::pose> pose_vec;
  std::vector<std::filesystem::path> pcl_filelsits;

  pose_vec = mypcl::read_pose(pose_path, pose_type);
  load_file_list(pcl_path, pcl_filelsits);
  size_t pose_size = pose_vec.size();
  cout << "pose size " << pose_size << endl;

  pcl::PointCloud<PointType>::Ptr pc_surf(new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_full(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  ros::Time cur_t;
  geometry_msgs::PoseArray parray;
  parray.header.frame_id = "camera_init";
  parray.header.stamp = cur_t;
  visualization_msgs::MarkerArray markerArray;

  cout << "push enter to view" << endl;
  getchar();
  pcl::PointCloud<PointType>::Ptr pc_filtered_all(
      new pcl::PointCloud<PointType>);
  for (size_t i = 0; i < pose_size; i++) {
    mypcl::loadPointCloud(pcl_filelsits, pc_surf, i);

    pcl::PointCloud<PointType>::Ptr pc_filtered(new pcl::PointCloud<PointType>);
    pc_filtered->resize(pc_surf->points.size());
    int cnt = 0;
    for (size_t j = 0; j < pc_surf->points.size(); j++) {
      pc_filtered->points[cnt] = pc_surf->points[j];
      cnt++;
    }
    pc_filtered->resize(cnt);

    mypcl::transform_pointcloud(*pc_filtered, *pc_filtered, pose_vec[i].t,
                                pose_vec[i].q);
    downsample_voxel(*pc_filtered, downsample_size);
    if (output_merge_pcl) {
      // append to all point cloud
      *pc_filtered_all += *pc_filtered;
    }

    pcl::toROSMsg(*pc_filtered, cloudMsg);
    cloudMsg.header.frame_id = "camera_init";
    cloudMsg.header.stamp = cur_t;
    pub_map.publish(cloudMsg);

    geometry_msgs::Pose apose;
    apose.orientation.w = pose_vec[i].q.w();
    apose.orientation.x = pose_vec[i].q.x();
    apose.orientation.y = pose_vec[i].q.y();
    apose.orientation.z = pose_vec[i].q.z();
    apose.position.x = pose_vec[i].t(0);
    apose.position.y = pose_vec[i].t(1);
    apose.position.z = pose_vec[i].t(2);
    parray.poses.push_back(apose);
    pub_pose.publish(parray);

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(pose_vec[i].t(0), pose_vec[i].t(1),
    // pose_vec[i].t(2))); tf::Quaternion q(pose_vec[i].q.x(),
    // pose_vec[i].q.y(), pose_vec[i].q.z(), pose_vec[i].q.w());
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
    // "camera_init", "turtle_name"));

    // publish pose trajectory
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = cur_t;
    marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = pose_vec[i].t(0);
    marker.pose.position.y = pose_vec[i].t(1);
    marker.pose.position.z = pose_vec[i].t(2);
    pose_vec[i].q.normalize();
    marker.pose.orientation.x = pose_vec[i].q.x();
    marker.pose.orientation.y = pose_vec[i].q.y();
    marker.pose.orientation.z = pose_vec[i].q.x();
    marker.pose.orientation.w = pose_vec[i].q.w();
    marker.scale.x = marker_size; // Set the scale of the marker -- 1x1x1 here
                                  // means 1m on a side
    marker.scale.y = marker_size;
    marker.scale.z = marker_size;
    marker.color.r = float(1 - float(i) / pose_size);
    marker.color.g = float(float(i) / pose_size);
    marker.color.b = float(float(i) / pose_size);
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    pub_trajectory.publish(marker);

    // publish pose number
    visualization_msgs::Marker marker_txt;
    marker_txt.header.frame_id = "camera_init";
    marker_txt.header.stamp = cur_t;
    marker_txt.ns = "marker_txt";
    marker_txt.id = i; // Any marker sent with the same namespace and id will
                       // overwrite the old one
    marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ostringstream str;
    str << i;
    marker_txt.text = str.str();
    marker.action = visualization_msgs::Marker::ADD;
    marker_txt.action = visualization_msgs::Marker::ADD;
    marker_txt.pose.position.x = pose_vec[i].t(0) + marker_size;
    marker_txt.pose.position.y = pose_vec[i].t(1) + marker_size;
    marker_txt.pose.position.z = pose_vec[i].t(2);
    marker_txt.pose.orientation.x = pose_vec[i].q.x();
    marker_txt.pose.orientation.y = pose_vec[i].q.y();
    marker_txt.pose.orientation.z = pose_vec[i].q.x();
    marker_txt.pose.orientation.w = 1.0;
    marker_txt.scale.x = marker_size;
    marker_txt.scale.y = marker_size;
    marker_txt.scale.z = marker_size;
    marker_txt.color.r = 1.0f;
    marker_txt.color.g = 1.0f;
    marker_txt.color.b = 1.0f;
    marker_txt.color.a = 1.0;
    marker_txt.lifetime = ros::Duration();
    if (i % GAP == 0)
      markerArray.markers.push_back(marker_txt);
    pub_pose_number.publish(markerArray);

    // ros::Duration(0.001).sleep();
  }
  if (output_merge_pcl) {
    // save all point cloud
    auto out_file =
        std::filesystem::path(pose_path).replace_extension().string() + ".ply";
    pcl::io::savePLYFileASCII(out_file, *pc_filtered_all);
    std::cout << "Saved merged point cloud with "
              << pc_filtered_all->points.size() << " points to " << out_file
              << std::endl;
  }

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}