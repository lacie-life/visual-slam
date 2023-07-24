#  From image to point cloud
## Description
In this lecture, we will learn about to convert an image into a point cloud, two type data read from the camera is color image and the depth image
## Note

The actual RGB image and depth image directly collected in Kinect (or other rgb-d cameras) may have some minor problems
- Will have some little difference with the RGB image and the depth image
- The center of the aperture is not aligned. Because the depth is obtained by another camera after all, the parameters of the depth sensor and the color sensor may be inconsistent.
- Will have many "holds" in the deep map, because the camera can't catch the object to far or invisible

The deep map: is the distance between each pixel in the color image and the sensor
## Theory
### From 2D to 3D (Mathematics part)
Any point cloud X = {x1, … , xn} will have 6 components r, g, b, x, y, z representing their color and spatial position

The color: can recorder from the color of the image

The spatial position: can calculated from the image, camera model, and posture together
- The modul used in SLAM is conventional cameras the pinhole
#### Mathematics part
![illustration](https://github.com/lacie-life/RGB-D-SLAM-Tutorial/blob/main/RGBD-2/resources/camera.jpeg?raw=true)

The line through O perpendicular to the I(Image plane)  plane and and the focal plane is called the optical axis.The optical axis intersects the image plane I at c – called the Principal point.
fx, fy is refers to camra at x, y The focal length on the two axes

cx, cy is the coordinate of the line optical axis intersects image plane I at c

s is the zoom factor of the deep map

Firt we have the point in space coordinates [x, y, z] is pixel coordinates in the image [u, v, d] is as follows:
- u = ( x * fx ) / z + cx
- v = ( y * fy ) / z + cy
- d = z * s
  
The fx, fy, cx, cy is the parameter inside the camera
+ The intrinsic matrix camera

An object in space is usually represented by any coordinate system - not the camera coordinate system. Therefore, to calculate on these coordinate systems, the simplest way is to convert it to the camera coordinate system.

And the intrinsic matrix camera is the matrix of intrinsic parameter of camera use to correction the formula

And the formula to calculate is as the follow:

![formula](https://github.com/lacie-life/RGB-D-SLAM-Tutorial/blob/main/RGBD-2/resources/c.jpeg?raw=true)

s is the depth of the depth map

[ u, v, 1 ] is coordinate from the coordinate system O1

C is factor of the formula

R is the intrinsic matrix

[ x, y, z ] is the coordinate from the coordinate system O2

t is the vector from O1 to O2

### From 2D to 3D (programming par)
The code will in the folder src, here will be the manual:

We use OpenCV's imread function to read pictures. In OpenCV2, the image is based on the matrix (cv::MAt) as the basic data
structure. The Mat structure can not only help you manage memory and pixel information, but also supports some common matrix
operations, which is a very convenient structure.

The color will have 3 channels is R, G, B, each channels will occupies 8 bit, so it is called 8UC3 (8-bit unsigend char) structure

The depth map is a single-channel image (that is, unsigned short in C++), each pixel is composed of 16 bits,  and the value of the pixel represents the distance of the point from the
sensor. Usually the value of 1000 represents 1 meter, so we set the camera_factor to 1000. In this way, the reading of each pixel in the
depth map is divided by 1000, which is its true distance from you.

