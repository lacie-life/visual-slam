# RGB-D-SLAM Tutorial 4
## Description 
In this part, we will learn about how use to these
two vectors to stitch the point clouds of the two images together to form a larger point cloud.

In the part 2 we know about the the formula to convert the vector in coordicate system O1 to coordicate system O2, In this part we will optimal it for the formula more economical and practical way of expression

![formula](https://f41-zpg.zdn.vn/982246293489619223/acfd0e5aa57b5025096a.jpg)

## Stitching point clouds
The splicing of point clouds is essentially a process of transforming the point clouds. This transformation is often described by a
transform matrix:

![formula](https://f33-zpg.zdn.vn/6093985735113580976/1041477ea54f5011095e.jpg)

The upper left part of the matrix is a The rotation matrix is an orthogonal matrix. The upper right part is The displacement vector.
Bottom left is The zoom vector of is usually taken as 0 in SLAM, because things in the environment are unlikely to suddenly become
larger or smaller (and there is no shrinking light). The lower right corner is 1. Such a matrix can perform homogeneous transformations on
points or other things:

![formula](https://f17-zpg.zdn.vn/8916645180686733214/9573b6185429a177f838.jpg)


