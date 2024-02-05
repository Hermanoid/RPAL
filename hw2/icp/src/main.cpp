#include "icp.h"
#include <iostream>
using std::cout;
using std::endl;

void print_points(const vector<Vector3d> &points, int num_points=10){
    for (int i = 0; i < num_points; i++){
        cout << points[i].transpose() << endl;
    }
}

int main(){
    cout << "Hello, World!" << endl;

    const int num_points = 10000;
    const Isometry3d transform = 
        Isometry3d(Eigen::AngleAxisd(0.5, Vector3d(0.2, 0.3, 0.1).normalized()) 
        * Eigen::Translation3d(Vector3d(1, 0.1, 0.1)));

    vector<Vector3d> target = vector<Vector3d>(num_points);
    // Generate random points
    for (int i = 0; i < num_points; i++){
        target[i] = Vector3d::Random()*100;
    }
    cout << "Generated points:" << endl;
    print_points(target, 10);

    vector<Vector3d> source = vector<Vector3d>(num_points);
    // Transform the points
    for (int i = 0; i < num_points; i++){
        source[i] = transform*target[i];
    }
    // Print the first 10 points
    cout << "Transformed points:" << endl;
    print_points(source, 10);

    Isometry3d icp_transform = icp(target, source, 100);

    cout << "Estimated transform:" << endl;
    cout << icp_transform.matrix() << endl;
    cout << "True transform:" << endl;
    cout << transform.matrix() << endl;
    return 0;
}