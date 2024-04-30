#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <string>
#include <algorithm> // find
#include <stdio.h>   /* printf, scanf, puts, NULL */
#include <stdlib.h>  /* srand, rand */
#include <time.h>    /* time */
#include <random>

// OpenCV:
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp> // Viz & VTK visualization
#include <opencv2/viz/types.hpp>

// Open3D:
#include "open3d/Open3D.h"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// SFM
// #include "frame.h"

std::vector<Eigen::Vector3d> colors = {
    Eigen::Vector3d(0.1, 0.1, 0.7),
    Eigen::Vector3d(0.1, 0.7, 0.1),
    Eigen::Vector3d(0.7, 0.1, 0.1),
    Eigen::Vector3d(0.7, 0.7, 0.1),
    Eigen::Vector3d(0.7, 0.1, 0.7),
    Eigen::Vector3d(0.1, 0.7, 0.7),
    Eigen::Vector3d(0.7, 0.7, 0.7),
    Eigen::Vector3d(0.1, 0.1, 0.1),
    Eigen::Vector3d(0.7, 0.7, 0.7),
    Eigen::Vector3d(0.1, 0.1, 0.1),
};


// ----------------------------------------------------------------------------
int main()
{

    // rgbd data
    std::string base_name = "cloud_bin_";
    const std::string file_dir = "data/rgbd/";

    // lidar data
    // std::string base_name = "Hokuyo_";
    // const std::string file_dir = "data/lidar/";

    const int num_files = 10; // number of point clouds in the dataset

    // file names
    std::vector<std::string> file_names;
    for (int i = 0; i < num_files; ++i)
    {
        std::string name = base_name + std::to_string(i) + ".ply";
        file_names.emplace_back(name);
    }

    // std::shared_ptr<open3d::geometry::PointCloud> prev_downpcd;
    // std::shared_ptr<open3d::geometry::PointCloud> prev_pcd;
    std::shared_ptr<open3d::geometry::PointCloud> global_small;
    std::shared_ptr<open3d::pipelines::registration::Feature> prev_fphf;
    Eigen::Matrix4d total_transform = Eigen::Matrix4d::Identity();
    std::shared_ptr<open3d::geometry::PointCloud> global_pcd = std::make_shared<open3d::geometry::PointCloud>();
    // ICP Params
    double voxel_size = 0.075;
    // double threshold = voxel_size*2;
    double threshold = 0.2;
    double sigma = 0.2; // Tukey loss parameter is std dev of noise. Essentially, inlier cutoff distance.
    auto loss = std::make_shared<open3d::pipelines::registration::TukeyLoss>(sigma);
    auto p2l = open3d::pipelines::registration::TransformationEstimationPointToPlane(loss);
    Eigen::Matrix4d trans_init = Eigen::Matrix4d::Identity();

    // RANSAC Registration params
    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>>correspondence_checker;
    auto correspondence_checker_edge_length =
            open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
                    0.9);
    auto correspondence_checker_distance =
            open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(
                        0.05);
    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);


    // load point clouds and display them
    for (int i = 0; i < num_files; ++i)
    {
        std::cout << "Processing file: " << file_names[i] << " (" << i + 1 << "/" << num_files << ")" << std::endl;
        std::string file_path = file_dir + file_names[i];

        // read point cloud data
        auto point_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
        std::cout << "Read and downsample point cloud from " << file_path << std::endl;
        open3d::io::ReadPointCloud(file_path, *point_cloud_ptr);
        // point_cloud_ptr->PaintUniformColor(colors[i]);
        point_cloud_ptr->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size*3, 30));
        auto downpcd = point_cloud_ptr->VoxelDownSample(voxel_size);
        int downsize = downpcd->points_.size();
        int ogsize = point_cloud_ptr->points_.size();
        std::cout << "downsample/original " << downsize << "/" << ogsize << " (" << ((float)downsize/ogsize)*100 << "%)" <<std::endl;

        // compute normals and features

        if(i > 0)
        {
            auto start_time = std::chrono::system_clock::now();
            auto search_param = open3d::geometry::KDTreeSearchParamHybrid(voxel_size*10, 100);
            auto fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*downpcd, search_param);
            auto global_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*global_small, search_param);
            // compute correspondences (NOTE: try skipping global registration)
            auto result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
                *downpcd, *global_small, *fpfh, *global_fpfh, true, 0.1,
                open3d::pipelines::registration::TransformationEstimationPointToPoint(false), 3,
                correspondence_checker,
                open3d::pipelines::registration::RANSACConvergenceCriteria(10000000, 0.999)
            );
            std::cout << "RANSAC Registration Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() << "ms" << std::endl;
            std::cout << "RANSAC Inlier RMSE: " << result.inlier_rmse_ << std::endl;
            trans_init = result.transformation_;

            // Robust ICP (Use dense point cloud for fine-tuning,... actually no that's expensive, use downsample)
            // Update - ok so apparently it (sometimes) takes the sparse point cloud longer to converge? Wild.
            start_time = std::chrono::system_clock::now();
            std::cout << "Robust point-to-plane ICP, threshold=" << threshold << ":" << std::endl;
            auto reg_p2l = open3d::pipelines::registration::RegistrationICP(*point_cloud_ptr, *global_pcd, threshold, trans_init, p2l);
            std::cout << reg_p2l.transformation_ << std::endl;
            std::cout << "Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() << "ms" << std::endl;

            // update total transformation
            // total_transform *= reg_p2l.transformation_;
            total_transform = reg_p2l.transformation_; // For ICP relative to global cloud, transformations are already global
            // trans_init = reg_p2l.transformation_; // Assume constant motion between frames
        }

        // prev_pcd = point_cloud_ptr;
        // prev_downpcd = downpcd;
        // Add point_cloud_ptr to global_pcd with transformation
        auto copy = std::make_shared<open3d::geometry::PointCloud>(*point_cloud_ptr);
        copy->Transform(total_transform);
        *global_pcd += *copy;
        // Keep global dense, but not *that* dense
        global_pcd = global_pcd->VoxelDownSample(0.02);
        global_small = global_pcd->VoxelDownSample(voxel_size);
        
        // prev_fphf = fpfh;

        // create visualization window
        // open3d::visualization::Visualizer visualizer;
        // visualizer.CreateVisualizerWindow("Point Cloud Viewer", 1024, 768);

        // // add point cloud to the window
        // // visualizer.AddGeometry(point_cloud_ptr);
        // visualizer.AddGeometry(global_small);


        // // run the visualization
        // visualizer.Run();
    }
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Point Cloud Viewer", 1024, 768);

    // add point cloud to the window
    // visualizer.AddGeometry(point_cloud_ptr);
    visualizer.AddGeometry(global_pcd);


    // run the visualization
    visualizer.Run();

    return 0;
}
