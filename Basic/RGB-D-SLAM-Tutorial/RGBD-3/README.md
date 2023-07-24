# Feature Extraction and Registration
#Description

SLAM is composed of two parts, "Localization" and "Mapping". Now let’s look at the positioning problem. To solve the robot's motion, we
must solve first the problem: given two images, how to know the motion relationship of the images?
So the method is based on feature points need to know the one to one correspondence between there features

Suppose we have two frames: F1 with F2 And, we have obtained two sets of one-to-one corresponding feature points:
- P = {p1, ... pn} in  F1
- Q = {q1, ... qn} in F2
  
Our purpose is to find a rotation matrix R, and displacement vector t, to calculate the formula:

∀ i , pi = Rqi + t 

But is will have some errors
-> To solve the minimizing problem  

![image](https://github.com/lacie-life/RGB-D-SLAM-Tutorial/blob/main/RGBD-3/resources/c.jpeg?raw=true)

This problem can be solved by the classic ICP algorithm. The core is singular value decomposition (SVD). so we can fix the problem by use image feature matching
### Image registration progamming realization
To do it, we need match two pairs of image and calculate their displacement relation
## Method 
We use the ransac method to estimate the motion of the image

