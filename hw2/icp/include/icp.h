#pragma once
#include <eigen3/Eigen/Dense>
using Eigen::Matrix4d;
using Eigen::Vector3d;
using std::vector;
using Eigen::Isometry3d;

// An alias for Eigen::Transform<double, 3, Eigen::Affine>
Isometry3d icp(const vector<Vector3d> &target, const vector<Vector3d> &source, int max_iter=100);