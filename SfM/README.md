# Simple SfM Pipeline

This is a simple SfM pipeline that takes a set of images and outputs a 3D point cloud and camera poses. 

## Dependencies

* OpenCV 4
* Ceres Solver 2.0
* Eigen3

## Build

## Usage
```bash
# Step 1 : feature extraction
./FeatureExtraction ../config/NEU.yaml

# Step 2 : Compute matching 
# (according to different data sets, decide to use **sequential matching** or **violent matching**, 
# by modifying the parameters of the configuration file in config)
./ComputeMatches ../config/NEU.yaml

# Step 3 : Check matches, to confirm that the first two steps are correct by displaying matching pairs between different images.
# (Optional)
./CheckMatches  ../config/NEU.yaml

# Step 4 : Incremental SfM
./Reconstruction ../config/NEU.yaml
```
## Results

## TODO

- [ ] Complete README
- [ ] Full Pipeline application
- [ ] GPU implementation
- [ ] Add Qt GUI [SimpleSfMViz](https://github.com/lacie-life/SimpleSfMViz)

## References

[1] Snavely N, Seitz S M, Szeliski R. Photo Tourism: Exploring Photo Collections In 3D. Acm Transactions on Graphics, 2006, 25(3):págs. 835-846.

[2] Wu C. Towards Linear-Time Incremental Structure from Motion// International Conference on 3d Vision. IEEE Computer Society, 2013:127-134.

[3] Schönberger J L, Frahm J M. Structure-from-Motion Revisited// Computer Vision and Pattern Recognition. IEEE, 2016.

[4] [3dv_tutorial](https://github.com/mint-lab/3dv_tutorial)

[5] [Mastering OpenCV with Practical Computer Vision Projects](https://github.com/MasteringOpenCV/code/tree/master)




