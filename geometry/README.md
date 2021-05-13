## Eigen transform matrix

• Rotation matrix ( 3 × 3 ): Eigen::Matrix3d.

• Rotation vector ( 3 × 1 ): Eigen::AngleAxisd.

• Euler angle ( 3 × 1 ): Eigen::Vector3d.

• Quaternion ( 4 × 1 ): Eigen::Quaterniond.

• Euclidean transformation matrix ( 4 × 4 ): Eigen::Isometry3d.

• Affine transform ( 4 × 4 ): Eigen::Affine3d.

• Perspective transformation ( 4 × 4 ): Eigen::Projective3d.


### ![Example 1](https://github.com/lacie-life/visual-slam/blob/master/geometry/images/example-1.png?raw=true)

## Visualize Geometry Example
1. How to compile this program:

* use pangolin: slambook/3rdpart/Pangolin or download it from github: https://github.com/stevenlovegrove/Pangolin

* install dependency for pangolin (mainly the OpenGL):
  sudo apt-get install libglew-dev

* compile and install pangolin
  cd Pangolin
  mkdir build
  cd build
  cmake ..
  make
  sudo make install

* compile this program:
  mkdir build
  cd build
  cmake ..
  make

* run the build/visualizeGeometry

2. How to use this program:

The UI in the left panel displays different representations of T_w_c ( camera to world ). It shows the rotation matrix, tranlsation vector, euler angles (in roll-pitch-yaw order) and the quaternion.
Drag your left mouse button to move the camera, right button to rotate it around the box, center button to rotate the camera itself, and press both left and right button to roll the view.
Note that in this program the original X axis is right (red line), Y is up (green line) and Z in back axis (blue line). You (camera) are looking at (0,0,0) standing on (3,3,3) at first.

3. Problems may happen:
* I found that in virtual machines there may be an error in pangolin, which was solved in its issue: https://github.com/stevenlovegrove/Pangolin/issues/74 . You need to comment the two lines mentioned by paulinus, and the recompile and reinstall Pangolin, if you happen to find this problem
