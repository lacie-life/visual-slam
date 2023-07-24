//
// Created by lacie on 14/05/2021.
//

#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv){
    // Quaternion rotation matrix
    Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);

    // Normalize
    q1.normalize();
    q2.normalize();

    // Translation vector
    Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);

    // Point p
    Vector3d p1(0.5, 0, 0.2);

    // Get transform robot 1 and robot 2 to world coordinate
    Isometry3d T1w(q1), T2w(q2);
    T1w.pretranslate (t1);
    T2w.pretranslate (t2);

    // Transform p from robot 1's sys to robot 2's sys
    Vector3d p2 = T2w * T1w.inverse() * p1;
    cout << endl << p2.transpose() << endl;

    return 0;
}

