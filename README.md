**pclFilCen**\
Th code summuries some common pre-processing algorithm to deal with the raw point cloud data from RGB-D camera. The order of processes as following: (1) voxelize, (2) passThroughFilter - distance filter, (3) planar filter, (4) segmentation, (5) centroid Identification.\
The point cloud data input as the image below:\
<img src="https://github.com/buivn/images/blob/master/pcdInput.png" width="300">\
<img src="https://github.com/buivn/images/blob/master/4objectnew1.jpg" width="300">\
The point cloud data after distance filtering (passThrough Filtering):\
<img src="https://github.com/buivn/images/blob/master/afterPassthroughFilter.png" width="300">\

The point cloud after planar filtering: \

<img src="https://github.com/buivn/images/blob/master/pcd_filter_afterfiltering.png" width="300">\

The point cloud after segmentation:\
<img src="https://github.com/buivn/images/blob/master/FirstCloud.png" width="300">\
<img src="https://github.com/buivn/images/blob/master/SecondCloud.png" width="300">\

The centroids of four objects in the point cloud:\

<img src="https://github.com/buivn/images/blob/master/object_centroids.png" width="300">\
