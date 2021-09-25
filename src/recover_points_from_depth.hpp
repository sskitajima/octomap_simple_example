#ifndef RECOVER_POINTS_FROM_DEPTH_H_
#define RECOVER_POINTS_FROM_DEPTH_H_

#include <octomap/octomap.h>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <vector>
#include <Eigen/Core>

void read_cloud_from_depth(const cv::Mat& depth_img, octomap::Pointcloud& cloud);
Eigen::Vector4d recover_points(const double depth, const double ux, const double uy );

void insert_cloud_to_octomap(octomap::OcTree * const octomap_tree, const octomap::Pointcloud& point_cloud);

bool dump_points(const octomap::Pointcloud& point_cloud, const std::string& out_point_path);
bool dump_octomap(octomap::OcTree * const octomap_tree, const std::string& out_octomap_path);

#endif