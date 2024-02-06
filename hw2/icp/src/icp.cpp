#include "icp.h"
#include "nanoflann.hpp"
#include <iostream>

using namespace nanoflann;

// using PointCloud=Eigen::Matrix<double, 3, Eigen::Dynamic>;


void find_correspondences(const PointCloud &target, const PointCloud &source, vector<Eigen::Index> &correspondences){
    const int num_points = target.rows();
    
    // Init KD Tree
    typedef KDTreeEigenMatrixAdaptor<PointCloud, 3, nanoflann::metric_L2, false> KDTree;
    KDTree target_tree(3, target, 10);

    // Find the nearest neighbor for each point in source
    for (int i = 0; i < num_points; i++){
        Vector3d point = source.col(i);
        Eigen::Index *indices;
        double min_dist;
        target_tree.index->knnSearch(&point[0], 1, indices, &min_dist);
        correspondences[i] = indices[0];
    }
}

Isometry3d estimate_transform(const PointCloud &target, const PointCloud &source){
    const int num_points = target.rows();
    Eigen::Vector3d target_mean = target.rowwise().mean(); // Mean of 3xn matrix is a 3x1 column vector
    Eigen::Vector3d source_mean = source.rowwise().mean();
    PointCloud centered_target = target.colwise() - target_mean; // Subtract the mean from each column
    PointCloud centered_source = source.colwise() - source_mean;

    // Compute the covariance matrix (sum of x_i * y_i^T for every pair of corresponding points)
    Eigen::Matrix3d covariance = target*centered_source.transpose(); // 3xn * nx3 = 3x3

    // Compute the SVD of the covariance matrix
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // Get that transform
    Eigen::Matrix3d R = V*U.transpose(); // Compute the rotation matrix
    Eigen::Vector3d t = target_mean - R*source_mean; // Compute the translation vector

    Isometry3d transform = Isometry3d::Identity();
    transform.linear() = R;
    transform.translation() = t;

    return transform;

}

Isometry3d icp(const PointCloud &target, const PointCloud &source, int max_iter){
    const size_t num_points = target.rows();

    PointCloud working_source = source;
    vector<Eigen::Index> correspondences(num_points);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

    for (int i = 0; i < max_iter; i++){
        // Find correspondences
        find_correspondences(target, working_source, correspondences);

        // Get the re-indexed source points (how's this function for some eigen-fu?)
        // Might be faster to pass correspondences to estimate_transform add do add-math with it directly (no copy)
        PointCloud reindexed_source = working_source(Eigen::all, correspondences);

        // Estimate the transform
        Isometry3d delta_transform = estimate_transform(target, reindexed_source);

        // Update the source points
        working_source = delta_transform*working_source;

        // Update the total transform
        transform = delta_transform*transform;

        // Check for convergence with magnitude of the translation vector angle angle of axis-angle rotation
        double mag_translation = delta_transform.translation().norm();
        double mag_rotation = Eigen::AngleAxisd(delta_transform.linear()).angle();
        std::cout << "Iteration " << i << " translation: " << mag_translation << " rotation: " << mag_rotation << std::endl;
        if (mag_translation < 1e-6 && mag_rotation < 1e-6){
            break;
        }
    }

    return transform;
}
