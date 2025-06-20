#ifndef MYPCL_HPP
#define MYPCL_HPP

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
    vector_vec3d;
typedef std::vector<Eigen::Quaterniond,
                    Eigen::aligned_allocator<Eigen::Quaterniond>>
    vector_quad;
// typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZ PointType;
// typedef pcl::PointXYZI PointType;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace mypcl {
struct pose {
  pose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
       Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0))
      : q(_q), t(_t) {}
  Eigen::Quaterniond q;
  Eigen::Vector3d t;
};

void loadPLY(std::string filePath, int pcd_fill_num,
             pcl::PointCloud<PointType>::Ptr &pc, int num,
             std::string prefix = "") {
  std::stringstream ss;
  if (pcd_fill_num > 0)
    ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
  else
    ss << num;
  pcl::io::loadPLYFile(filePath + prefix + ss.str() + ".ply", *pc);
}

void loadPCD(std::string filePath, int pcd_fill_num,
             pcl::PointCloud<PointType>::Ptr &pc, int num,
             std::string prefix = "") {
  std::stringstream ss;
  if (pcd_fill_num > 0)
    ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
  else
    ss << num;
  pcl::io::loadPCDFile(filePath + prefix + ss.str() + ".pcd", *pc);
}

// Universal point cloud loader that automatically detects file format
void loadPointCloud(std::vector<std::filesystem::path> &pcl_filelsits,
                    pcl::PointCloud<PointType>::Ptr &pc, int num) {
  auto file = pcl_filelsits[num];

  // Check if PCD file exists first (since it's the default)
  if (file.extension() == ".pcd") {
    pcl::io::loadPCDFile(file, *pc);
    return;
  } else if (file.extension() == ".ply") {
    pcl::io::loadPLYFile(file, *pc);
    return;
  }
  // If neither exists, throw an error
  std::cerr << "Error: Neither PCD file (" << file << ") nor PLY file (" << file
            << ") found!" << std::endl;
  throw std::runtime_error("Point cloud file not found");
}

void savdPCD(std::string filePath, int pcd_fill_num,
             pcl::PointCloud<PointType>::Ptr &pc, int num) {
  std::stringstream ss;
  if (pcd_fill_num > 0)
    ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
  else
    ss << num;
  pcl::io::savePCDFileBinary(filePath + ss.str() + ".pcd", *pc);
}

std::vector<pose> read_pose(std::string filename, int pose_type = 0,
                            Eigen::Quaterniond qe = Eigen::Quaterniond(1, 0, 0,
                                                                       0),
                            Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0)) {
  std::vector<pose> pose_vec;
  std::fstream file;
  file.open(filename);
  if (pose_type == 0) {
    double tx, ty, tz, w, x, y, z;
    while (!file.eof()) {
      file >> tx >> ty >> tz >> w >> x >> y >> z;
      Eigen::Quaterniond q(w, x, y, z);
      Eigen::Vector3d t(tx, ty, tz);
      pose_vec.push_back(pose(qe * q, qe * t + te));
    }
  } else {
    // pose_type: 0 for 4x4 matrix, 4 col a line
    std::vector<float> values;
    float value;
    Eigen::Matrix4f transform_matrix;
    while (!file.eof()) {
      file >> value;
      values.push_back(value);
      if (values.size() == 16) {
        // Extract 4x4 transformation matrix from values
        transform_matrix << values[0], values[1], values[2], values[3],
            values[4], values[5], values[6], values[7], values[8], values[9],
            values[10], values[11], values[12], values[13], values[14],
            values[15];

        // Extract translation from the last column
        Eigen::Vector3d t(transform_matrix(0, 3), transform_matrix(1, 3),
                          transform_matrix(2, 3));

        // Extract rotation matrix and convert to quaternion
        Eigen::Matrix3f rotation_matrix = transform_matrix.block<3, 3>(0, 0);
        Eigen::Quaternionf q_float(rotation_matrix);
        Eigen::Quaterniond q(q_float.w(), q_float.x(), q_float.y(),
                             q_float.z());
        pose_vec.push_back(pose(qe * q, qe * t + te));
        values.clear();
      }
    }
  }
  file.close();
  return pose_vec;
}

void transform_pointcloud(pcl::PointCloud<PointType> const &pc_in,
                          pcl::PointCloud<PointType> &pt_out, Eigen::Vector3d t,
                          Eigen::Quaterniond q) {
  size_t size = pc_in.points.size();
  pt_out.points.resize(size);
  for (size_t i = 0; i < size; i++) {
    Eigen::Vector3d pt_cur(pc_in.points[i].x, pc_in.points[i].y,
                           pc_in.points[i].z);
    Eigen::Vector3d pt_to;
    // if(pt_cur.norm()<0.3) continue;
    pt_to = q * pt_cur + t;
    pt_out.points[i].x = pt_to.x();
    pt_out.points[i].y = pt_to.y();
    pt_out.points[i].z = pt_to.z();
    // pt_out.points[i].r = pc_in.points[i].r;
    // pt_out.points[i].g = pc_in.points[i].g;
    // pt_out.points[i].b = pc_in.points[i].b;
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
append_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1,
             pcl::PointCloud<pcl::PointXYZRGB> pc2) {
  size_t size1 = pc1->points.size();
  size_t size2 = pc2.points.size();
  pc1->points.resize(size1 + size2);
  for (size_t i = size1; i < size1 + size2; i++) {
    pc1->points[i].x = pc2.points[i - size1].x;
    pc1->points[i].y = pc2.points[i - size1].y;
    pc1->points[i].z = pc2.points[i - size1].z;
    pc1->points[i].r = pc2.points[i - size1].r;
    pc1->points[i].g = pc2.points[i - size1].g;
    pc1->points[i].b = pc2.points[i - size1].b;
    // pc1->points[i].intensity = pc2.points[i-size1].intensity;
  }
  return pc1;
}

pcl::PointCloud<PointType>::Ptr
append_cloud(pcl::PointCloud<PointType>::Ptr pc1,
             pcl::PointCloud<PointType> pc2) {
  size_t size1 = pc1->points.size();
  size_t size2 = pc2.points.size();
  pc1->points.resize(size1 + size2);
  for (size_t i = size1; i < size1 + size2; i++) {
    pc1->points[i].x = pc2.points[i - size1].x;
    pc1->points[i].y = pc2.points[i - size1].y;
    pc1->points[i].z = pc2.points[i - size1].z;
    // pc1->points[i].r = pc2.points[i-size1].r;
    // pc1->points[i].g = pc2.points[i-size1].g;
    // pc1->points[i].b = pc2.points[i-size1].b;
    // pc1->points[i].intensity = pc2.points[i-size1].intensity;
  }
  return pc1;
}

double compute_inlier_ratio(std::vector<double> residuals, double ratio) {
  std::set<double> dis_vec;
  for (size_t i = 0; i < (size_t)(residuals.size() / 3); i++)
    dis_vec.insert(fabs(residuals[3 * i + 0]) + fabs(residuals[3 * i + 1]) +
                   fabs(residuals[3 * i + 2]));

  return *(std::next(dis_vec.begin(), (int)((ratio)*dis_vec.size())));
}

void write_pose(std::vector<pose> &pose_vec, std::filesystem::path path,
                int pose_type = 0, std::string suffix = "") {
  std::string file_name =
      path.replace_extension().string() + suffix + path.extension().string();
  std::cout << file_name << std::endl;
  std::ofstream file;
  file.open(file_name, std::ofstream::trunc);
  file.close();
  Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(),
                        pose_vec[0].q.z());
  Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
  file.open(file_name, std::ofstream::app);

  if (pose_type == 0) {
    for (size_t i = 0; i < pose_vec.size(); i++) {
      pose_vec[i].t << q0.inverse() * (pose_vec[i].t - t0);
      pose_vec[i].q.w() = (q0.inverse() * pose_vec[i].q).w();
      pose_vec[i].q.x() = (q0.inverse() * pose_vec[i].q).x();
      pose_vec[i].q.y() = (q0.inverse() * pose_vec[i].q).y();
      pose_vec[i].q.z() = (q0.inverse() * pose_vec[i].q).z();
      file << pose_vec[i].t(0) << " " << pose_vec[i].t(1) << " "
           << pose_vec[i].t(2) << " " << pose_vec[i].q.w() << " "
           << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
           << pose_vec[i].q.z();
      if (i < pose_vec.size() - 1)
        file << "\n";
    }
  } else {
    for (size_t i = 0; i < pose_vec.size(); i++) {
      pose_vec[i].t << q0.inverse() * (pose_vec[i].t - t0);
      pose_vec[i].q.w() = (q0.inverse() * pose_vec[i].q).w();
      pose_vec[i].q.x() = (q0.inverse() * pose_vec[i].q).x();
      pose_vec[i].q.y() = (q0.inverse() * pose_vec[i].q).y();
      pose_vec[i].q.z() = (q0.inverse() * pose_vec[i].q).z();

      // pose_type: 1 for 4x4 matrix, 4 col a line
      Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
      transform_matrix.block<3, 3>(0, 0) = pose_vec[i].q.toRotationMatrix();
      transform_matrix(0, 3) = pose_vec[i].t(0);
      transform_matrix(1, 3) = pose_vec[i].t(1);
      transform_matrix(2, 3) = pose_vec[i].t(2);
      for (int j = 0; j < 4; j++) {
        for (int k = 0; k < 4; k++) {
          file << std::setprecision(6) << transform_matrix(j, k);
          if (j < 3 || k < 3)
            file << " ";
        }
        if (i < pose_vec.size() - 1 || j < 3)
          file << "\n";
      }
    }
  }
  file.close();
}

void writeEVOPose(std::vector<double> &lidar_times, std::vector<pose> &pose_vec,
                  std::string path) {
  std::ofstream file;
  file.open(path + "evo_pose.txt", std::ofstream::trunc);
  for (size_t i = 0; i < pose_vec.size(); i++) {
    file << std::setprecision(18) << lidar_times[i] << " "
         << std::setprecision(6) << pose_vec[i].t(0) << " " << pose_vec[i].t(1)
         << " " << pose_vec[i].t(2) << " " << pose_vec[i].q.x() << " "
         << pose_vec[i].q.y() << " " << pose_vec[i].q.z() << " "
         << pose_vec[i].q.w();
    if (i < pose_vec.size() - 1)
      file << "\n";
  }
  file.close();
}
} // namespace mypcl

#endif