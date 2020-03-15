# PCL Libraries
This repo contains all the functions, libraries which were used in the previous projects. There are some main functions such as: 



To run the program, user should adjust the link of input file (in main program) to file "4objectnew1.pcd".\
Th code summuries some common pre-processing algorithm to deal with the raw point cloud data from RGB-D camera. The order of processes as following: (1) passThrough - distance Filter, (1) Downsampling by VoxelGrid, (3) planar filter, (4) segmentation, (5) Centroid Identification.\
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

**Meshing Two pointcloud of single object from two camera**
