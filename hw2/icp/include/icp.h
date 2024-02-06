#pragma once
#include <eigen3/Eigen/Dense>
using Eigen::Matrix4d;
using Eigen::Vector3d;
using std::vector;
using Eigen::Isometry3d;

using PointCloud=Eigen::Matrix<double, 3, Eigen::Dynamic>;

// An alias for Eigen::Transform<double, 3, Eigen::Affine>
Isometry3d icp(const PointCloud &target, const PointCloud &source, int max_iter);