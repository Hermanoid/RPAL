#include "icp.h"
#include <Eigen/Dense>
#include <Eigen/Jacobi>
#include <iostream>
#include <nanoflann.hpp>
#include <pthread.h>




Isometry3d icp(const vector<Vector3d> &target, const vector<Vector3d> &source, int max_iter){
    
    return Isometry3d::Identity();
}
