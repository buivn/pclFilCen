**pclFilCen**\
Th code summuries some common pre-processing algorithm to deal with the raw point cloud data from RGB-D camera. The order of processes as following: (1) voxelize, (2) passThroughFilter - distance filter, (3) planar filter, (4) segmentation, (5) centroid Identification.\
The point cloud data input as the image below (Depth data - left, RGB image - right):
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/pcdInput.png" width="222">
  <img src="https://github.com/buivn/images/blob/master/4objectnew1.jpg" width="250">
</p>
The point cloud data after distance filtering (passThrough Filtering):
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/passThrough061319.png" width="250">
</p>

The point cloud data after downsampling by VoxelGrid:
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/Downsampling061319.png" width="250">
</p>

After planar filtering: 
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/planerFiltering1.png" width="250">
</p>
After segmentation (presenting two objects):
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/object1061319.png" width="160">
  <img src="https://github.com/buivn/images/blob/master/object2061319.png" width="160">
  <img src="https://github.com/buivn/images/blob/master/object3061319.png" width="160">
  <img src="https://github.com/buivn/images/blob/master/object4061319.png" width="160">
</p>
The centroids of four objects in the point cloud:
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/centroid3D.png" width="300">
</p>
The objects' 3D centroids in point cloud are converted to 2D centroids in RGB image:
<p align="center">
  <img src="https://github.com/buivn/images/blob/master/2Dcentroids.png" width="250">
</p>
