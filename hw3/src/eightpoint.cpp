#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix3d;
using std::vector;

// Pts Set (2D homogenous):
// x1 x2 ... x8
// y1 y2 ... y8
// 1  1  ...  1
using EightPointSet = Eigen::Matrix<double, 3, 8>;
// "A" Matrix (9x8)
// Has a row for each point. Each is effectively the flattened Covariance matrix y'^t * y
// Each column represents a funky equation of all points
using AMat = Eigen::Matrix<double, 8, 9>;

Matrix3d precondition_matrix(EightPointSet &pts){
    // Compute the centroid of the points
    Vector3d centroid = pts.rowwise().mean();

    // Compute the distance from the centroid to each point
    Eigen::Vector<double, 8> dist = (pts.colwise() - centroid).colwise().norm();

    // Compute the scaling factor (so that the average distance from the origin is sqrt(2))
    double scale = sqrt(2.0)/dist.mean();

    // Compute the preconditioning matrix
    Matrix3d T;
    T << scale, 0, -scale*centroid(0),
         0, scale, -scale*centroid(1),
         0, 0, 1;

    return T;
}

AMat compute_A_matrix(EightPointSet &pts_in, EightPointSet &pts_corr){
    AMat A;
    // for(int point = 0; point < 8; point++){
    //     // Each row equals the flattened covariance matrix y'^t * y
    //     // y'_1 * y_1, y'_1 * y_2, y'_1 * 1, y'_2* y_1 ....
    //     // ^^ UPDATE: I don't feel like rewriting that but I swapped it y' with y
    //     auto test = pts_corr.col(point).transpose() * pts_in.col(point);
    //     // A.row(point) = (pts_corr.col(point).transpose() * pts_in.col(point)).reshaped();
    // }


    // This is not the best way but it's a more direct transcription of the python I'm using as a reference
    // (bc my attempts above don't seem right)
    A << (pts_in.row(0).array() * pts_corr.row(0).array()).transpose(),
        (pts_in.row(0).array() * pts_corr.row(1).array()).transpose(),
        pts_in.row(0).transpose(),
        (pts_in.row(1).array() * pts_corr.row(0).array()).transpose(),
        (pts_in.row(1).array() * pts_corr.row(1).array()).transpose(),
        pts_in.row(1).transpose(),
        pts_corr.row(0).transpose(),
        pts_corr.row(1).transpose(),
        Eigen::VectorXd::Ones(pts_in.cols());
    return A;
}


Matrix3d eight_point_algo(EightPointSet &pts_in, EightPointSet &pts_corr){
    // Normalize the points
    int num_points = pts_in.cols();
    if(num_points != pts_corr.cols() || num_points != 8){
        throw std::invalid_argument("pts_in and pts_corr must have 8 points");
    }


    // Precondition the points
    Matrix3d T_in = precondition_matrix(pts_in);
    EightPointSet pts_in_norm = T_in * pts_in;
    Matrix3d T_corr = precondition_matrix(pts_corr);
    EightPointSet pts_corr_norm = T_corr * pts_corr;

    // Compute the A matrix
    AMat A = compute_A_matrix(pts_in_norm, pts_corr_norm);
    // The (optimal?) solution to Ax=0 is the singular vector of A corresponding to the
    // smallest singular value; that is, the last column of V in A=UDV (but V^T)
    Eigen::JacobiSVD<AMat> svd(A, Eigen::ComputeFullV);
    Eigen::Vector<double, 9> x = svd.matrixV().transpose().col(8);
    // Reshape into a 3x3 matrix (there should be a better way to do this)
    Matrix3d EUnconditioned;
    EUnconditioned << x(0), x(1), x(2),
                      x(3), x(4), x(5),
                      x(6), x(7), x(8);


    // Post-condition by forcing rank=2 and equal eigenvalues of 1
    // Start by taking the SVD (again)
    Eigen::JacobiSVD<Matrix3d> svd_E(EUnconditioned, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3d EScaled = svd_E.matrixU() * Vector3d({1, 1, 0}).asDiagonal() * svd_E.matrixV().transpose();
    
    // Undo the preconditioning
    Matrix3d E = T_in.transpose() * EScaled * T_corr;

    return E;
}

int main(){
    // Test the eight_point_algo by creating a known E matrix and then
    // seeing if we can recover it from noisy points
    // Create a known E matrix as a combination of rotation and translation
    // Use definition that E = (Skew Symmetric Translation) * Rotation
    Vector3d translation = Vector3d({1, 2, 3});
    Matrix3d translation_skewsymmetric;
    translation_skewsymmetric << 0, -translation(2), translation(1),
                                 translation(2), 0, -translation(0),
                                 -translation(1), translation(0), 0;
    Matrix3d rotation = (Eigen::AngleAxisd(M_PI/4, Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(M_PI/3, Vector3d::UnitY()) *
                         Eigen::AngleAxisd(M_PI/6, Vector3d::UnitX())).toRotationMatrix();
    Matrix3d E = translation_skewsymmetric * rotation;
    std::cout << "Translation: " << std::endl <<  translation << std::endl;
    std::cout << "Rotation: " << std::endl << rotation << std::endl;
    std::cout << "E: " << std::endl << E << std::endl;

    // Dump the singular values (for science)
    Eigen::JacobiSVD<Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "Singular values of E: " << std::endl << svd.singularValues() << std::endl;

    // Form translation + rotation into a homogenous 3D transform
    Eigen::Matrix4d T;
    T.block<3, 3>(0, 0) = rotation;
    T.block<3, 1>(0, 3) = translation;
    T.block<1, 3>(3, 0) = Eigen::Vector3d::Zero().transpose();
    T(3, 3) = 1;


    // Form 3D homogenous to 2d homogenous Projection matrices for the cameras where camera 1 is at the origin and camera 2 is at T
    Eigen::Matrix<double, 3, 4> P1, P2;
    P1 << Matrix3d::Identity(), Vector3d::Zero();
    P2 << rotation, translation;

    std::cout << "Projection matrix for camera 1: " << std::endl << P1 << std::endl;
    std::cout << "Projection matrix for camera 2: " << std::endl << P2 << std::endl;

    EightPointSet pts_in;
    EightPointSet pts_corr;
    for(int i = 0; i < 8; i++){
        Vector4d pt_in = Eigen::Vector3d::Random().homogeneous();
        Vector4d pt_corr = T * pt_in;
        // pt_corr += 0.1 * Eigen::Vector3d::Random();
        // Rescale to homogenous (technically not, I guess?? homogenous coordinates are wild)
        Vector3d pt_in_proj = P1 * pt_in;
        pt_in_proj /= pt_in_proj(2);
        Vector3d pt_corr_proj = P2 * pt_corr;
        pt_corr_proj /= pt_corr_proj(2);
        pts_in.col(i) = pt_in_proj;
        pts_corr.col(i) = pt_corr_proj;
    }

    std::cout << "pts_in: " << std::endl << pts_in << std::endl;
    std::cout << "pts_corr: " << std::endl << pts_corr << std::endl;


    // Recover the E matrix
    Matrix3d E_recovered = eight_point_algo(pts_in, pts_corr);
    std::cout << "E_recovered: " << std::endl << E_recovered << std::endl;
    
    // Extract translation and rotation
    Eigen::JacobiSVD<Matrix3d> svd_E(E_recovered, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Translation is the last column of U (the nullspace, singular value = 0)
    Vector3d translation_recovered = svd_E.matrixU().col(2);
    // I do not understand the derivation for this
    Matrix3d Y;
    Y << 0, 1, 0,
         -1, 0, 0,
         0, 0, 1;
    Matrix3d rotation_recovered = svd_E.matrixU() * Y * svd_E.matrixV().transpose();
    std::cout << "Translation recovered: " << std::endl << translation_recovered << std::endl;
    std::cout << "Rotation recovered: " << std::endl << rotation_recovered << std::endl;

}