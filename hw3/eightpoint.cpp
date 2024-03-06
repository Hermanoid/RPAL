#include <iostream>
#include <eigen3/Eigen/Dense>

 

Eigen::Matrix3d eight_point_algo(vector<Vector3d> &pts1, vector<Vector3d> &pts2){
    // Normalize the points
    int num_points = pts1.size();
    Eige
    Eigen::Vector3d mean1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean2 = Eigen::Vector3d::Zero();
    for (int i = 0; i < num_points; i++){
        mean1 += pts1[i];
        mean2 += pts2[i];
    }
    mean1 /= num_points;
    mean2 /= num_points;
    double scale1 = 0;
    double scale2 = 0;
    for (int i = 0; i < num_points; i++){
        pts1[i] -= mean1;
        pts2[i] -= mean2;
        scale1 += pts1[i].squaredNorm();
        scale2 += pts2[i].squaredNorm();
    }
    scale1 = sqrt(scale1/num_points);
    scale2 = sqrt(scale2/num_points);
    for (int i = 0; i < num_points; i++){
        pts1[i] /= scale1;
        pts2[i] /= scale2;
    }

    // Construct the A matrix
    Eigen::Matrix<double, Eigen::Dynamic, 9> A(num_points, 9);
    for (int i = 0; i < num_points; i++){
        A(i, 0) = pts1[i][0]*pts2[i][0];
        A(i, 1) = pts1[i][0]*pts2[i][1];
        A(i, 2) = pts1[i][0];
        A(i, 3) = pts1[i][1]*pts2[i][0];
        A(i, 4) = pts1[i][1]*pts2[i][1];
        A(i, 5) = pts1[i][1];
        A(i, 6) = pts2[i][0];
        A(i, 7) = pts2[i][1];
        A(i, 8) = 1;
    }

    // Compute the SVD of A
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<double, 9, 1> F = svd.matrixV().col(8);

    //