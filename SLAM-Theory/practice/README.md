# End of the SLAMBOOK-2 

## Stereo VO

1. Data Structure

### What kind of data does VO need to process? What are the critical algorithms involed? What is the relationship between them?
- The most basic unit we deal with is the image. In stereo VO, that is a pair of
images. We might as well call it a frame.
- We will detect visual features on the frame. These features are many 2D pixels.
- We look for the association of features between images. If we see a feature
multiple times, we use the triangulation method to calculate its 3D position,
which forms the landmarks or map points.
- Relationship: 

![Data relationship](https://github.com/lacie-life/visual-slam/blob/main/SLAM-Theory/practice/resources/data_relationship.png?raw=true)

#### frame.h and frame.cpp
#### feature.h and feature.cpp
#### mappoint.h and mappoint.cpp
#### map.h and map.cpp

2. Pipeline

### which algorithms are responsible for feature extraction, which algorithms are accountable for triangulation, and which algorithms to deal with optimization problems?

- Frontend. We get an image frame from the sensor, and the frontend is responsible for extracting the features in the image. It then performs optical
flow tracking or feature matching with the previous frame and calculates the
frame’s position based on the optical flow result. If necessary, new feature
points should be added and triangulated. The result of the frontend processing will be used as the initial value of the backend optimization.

- Backend. The backend is a slower thread. It gets the processed keyframes
and landmark points, optimizes them, and then returns the optimized results.
The backend should control the optimization problem’s scale within a certain
range and cannot keep growing over time.

(1) The frontend adds new data to the map after finding a new keyframe.

(2) When the backend detects that the map has new data, it runs an optimization
routine and then resets the map scale. The old keyframes and map points are
removed if necessary. 

![Pipeline](https://github.com/lacie-life/visual-slam/blob/main/SLAM-Theory/practice/resources/pipeline.png?raw=true)

###  Peripheral modules

- We should have a camera class to manage the intrinsic and extrinsics as well
as the projection functions.
- We need a configuration file management class to facilitate reading content
from configuration files. Some critical parameters can be store in the configuration file for quick debugging;
- Because the algorithm runs on the Kitti dataset, we need to read the image
data according to Kitti’s storage format, which should also be handled by a
separate class.
- We need a visualization module to observe the running status of the system.
Otherwise, we have to scratch our heads against a series of numeric values.


3. Implement the Frontend

-- The frontend has three states: initialization, normal tracking, and tracking
lost

-- In the initialization state, we do the triangulation according to the optical flow
matching between the left and right eyes. We will establish the initial map
when successful.

-- In the tracking phase, the front end calculates the optical flow from the previous frame to the current frame and estimates the image pose based on the
optical flow result. This optical flow is used only for the left eye image to save
the computation resource.

-- If the tracked features are fewer than a threshold, we set the current frame as
a keyframe. For keyframes, do the following things:

+ Extract new feature points;
+ Find the corresponding points of these points on the right, and use triangulation to create new landmarks;
+ Add new keyframes and landmarks to the map and trigger a backend
optimization.
+ If the tracking is lost, reset the frontend system and reinitialize it.

4. Implement the Backend

-- After the backend is started, it will wait for the condition variable of map_
update_. 

-- When the map update is triggered, take the activated keyframes and map
points from the map and perform optimization




