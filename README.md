# PCL Libraries
This repo contains all the functions, libraries which were used in the previous projects. There are some main functions such as: \
(1) planar filter, \
(2) segmentation, \
(3) Centroid for each in a cluster set \
(4) Point Cloud Boundary \
(5) Area check \
(6) Pose Estimation \
(7) Plane's Height checkt
(8) Path Availability in Mobile Transformation
- To get a stable point cloud data, we should combine the data in several time, then mesh the pointclouds into one. THe processing on the aggregated one is more stable than a single one.\


The point cloud data input as the image below (Depth data - left, RGB image - right):
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/cvUnr2019/pcdInput.png" width="220">
  <img src="https://github.com/buivn/images/blob/master/cvUnr2019/4objectnew1.jpg" width="250">
</p>

Objects obtained after preprocessing: 
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/cvUnr2019/planerFiltering1.png" width="250">
</p>
The centroids of four objects in the point cloud:
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/cvUnr2019/centroid3D.png" width="300">
</p>
The objects' 3D centroids in point cloud are converted to 2D centroids in RGB image:
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/cvUnr2019/2Dcentroids.png" width="250">
</p>


