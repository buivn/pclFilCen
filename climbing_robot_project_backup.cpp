//#include <gpg/cloud_camera.h>
#include <iostream>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include "ros/ros.h"

#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/our_cvfh.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include "geometry_msgs/Pose.h"

#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include "sensor_msgs/PointCloud2.h"
#include <math.h>
#include <string>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZRGB> pcd_read(const std::string& filename)
{
    //pcd_original = loadPointCloudFromFile(filename);
  
    pcl::PointCloud<pcl::PointXYZRGB> cloud; 
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, cloud)) //* load the file
    //if (pcl::FileReader::read(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read the point cloud data file \n");
        //return (-1);
    }
    std::cout << "Loaded "
            << cloud.width * cloud.height
            << " data points from the pcd file with the following fields: "
            << std::endl;
  return cloud;
}

// passThrough filter
pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> distance_pass;
    distance_pass.setInputCloud (cloud);
    distance_pass.setFilterFieldName ("z");
    distance_pass.setFilterLimits (0.2, 0.9);
    //pass.setFilterLimitsNegative (true);
    distance_pass.filter (*cloud_filtered);

    std::cerr << "Cloud after passthrough filtering: " << cloud_filtered->size() << std::endl;

return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // this code run with input as argument of point cloud data 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    // define the size of the the leaf (in meter) = voxel size
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    // perform the downsampling filter then save the new data to cloud_filtered
    sor.filter (*cloud_filtered);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_planes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());  // variable to store the point cloud belong to the desired object
    
    // Create the segmentation object -> filter the desired point belong to the object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true); //> set this function using refine optimize Coefficient
    // Mandatory - set parameter for the planar filter
    seg.setModelType (pcl::SACMODEL_PLANE); // -> segment the plane in the point cloud
    // -> RANSAC = Random Sample Consensus = estimator method
    seg.setMethodType (pcl::SAC_RANSAC);    //-> use 2 step to determine the outliers or inliers
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);    // -> tolerate range to define the inliers & outlier
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int nr_points = (int) cloud->points.size ();
    int cloudsize = 0;
    
    // While 30% of the original cloud is still there
    while (cloud->points.size() > 0.3*nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        // seg.segment() result in the inliers point and coefficients
        seg.segment (*inliers, *coefficients); //save all points belong to the plane to inliners
        // outlier -> mau ngoai lai, out of observation range -> skew the estimation result
        // inlier -> sample lie inside the range -> fit with the model 
        if (inliers->indices.size () == 0) // -> if there is no inliers -> no point fit with the object
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // input the pointcloud into Extract() function
        extract.setInputCloud (cloud);
        // input a pointer to vector of indices (inliers)->represent the input data
        extract.setIndices(inliers);
        // set whether the indices should be returned, or all points_except_the indices
        extract.setNegative (false); //false -> return the indices
        // save the result to the cloud_p
        extract.filter (*cloud_p);
        // if no change in the indices -> stop
        if (cloudsize == cloud_p->width*cloud_p->height)
            break;
        // std::cerr <<"PointCloud representing the planar component: "<< cloud_p->width*cloud_p->height <<" data points."<< std::endl;
        cloud.swap (cloud_p);
        cloudsize = cloud_p->width*cloud_p->height;
    }
    return cloud;
}

void present_setOfPointasPointCloud(std::vector<Eigen::Vector3f> setOfPoint)
{
  
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudp;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width = setOfPoint.size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  // Eigen::Vector3f getData;
  // getData = setOfPoint[0];

  for (std::size_t i = 0; i < cloud.points.size (); ++i)
  {
  //   getData = setOfPoint[i];
    cloud.points[i].x = setOfPoint[i][0];
    cloud.points[i].y = setOfPoint[i][1];
    cloud.points[i].z = setOfPoint[i][2];
  }
  // for (std::size_t i = 0; i < cloudp->points.size (); ++i)
  //   std::cerr << "    " << cloudp->points[i].x << " " << cloudp->points[i].y << " " << cloudp->points[i].z << std::endl;
  pcl::visualization::CloudViewer viewer ("BoundaryPoint");
  viewer.showCloud(cloud.makeShared());

  while (!viewer.wasStopped())
  {
  }
}

void show_pcd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int clus_order, const std::string& titlename)
{
    int v1(0);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (titlename));
    viewer->setBackgroundColor (0, 0, 0);
    if (clus_order != 0)
    {   
        std::stringstream ss1;
        // convert clus_order to string
        ss1 << clus_order;       
        std::string ss = "Cluster " + ss1.str();
        viewer->addText(ss, 80, 50, 20, 120, 120, 200, "v1 text", v1);
    }   
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100);
    }
}

void show_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::MatrixXf& centroid3D)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Showing Centroid"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  int clusterNumber1 = centroid3D.rows();
  std::string id[clusterNumber1];
  pcl::ModelCoefficients line_coeff[clusterNumber1]; //, line_coeff2;

  for (int i=0; i < clusterNumber1; ++i)
  {
      line_coeff[i].values.resize(6);    // We need 6 values
      line_coeff[i].values[0] = 0;
      line_coeff[i].values[1] = 0;
      line_coeff[i].values[2] = 0; 
      line_coeff[i].values[3] = centroid3D(i,0);
      line_coeff[i].values[4] = centroid3D(i,1);
      line_coeff[i].values[5] = centroid3D(i,2);
  
      id[i] = "line"+i;
      viewer->addLine(line_coeff[i], id[i], 0);
  }
  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
}

void show_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr normals)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Showing Normals"));
    viewer->setBackgroundColor (0, 0, 0);
    // Add text to the screen
    //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (0.4);
    //Para: cloud   the input point cloud dataset containing the XYZ data
      //normals   the input point cloud dataset containing the normal data
      //level = 1   display only every level'th point (default: 100)
      // scale = 0.2  the normal arrow scale (default: 0.02m)
      //id = "normals"  the point cloud object id (default: cloud) 
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(cloud, normals, 1, 0.04, "normals"); 
  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
}

void show_aver_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Vector3f normals, Eigen::Vector4f centroid)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Showing Average Normals"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.4);
  
  pcl::ModelCoefficients line_coeff;
  
  line_coeff.values.resize (6);
  // the first point the line will go through
  line_coeff.values[0] = centroid[0];
  line_coeff.values[1] = centroid[1];
  line_coeff.values[2] = centroid[2];
  // adding the orientation
  line_coeff.values[3] = normals[0];
  line_coeff.values[4] = normals[1]; 
  line_coeff.values[5] = normals[2]; 

  std::string id = "normal vector";
  viewer->addLine(line_coeff, id, 0);
  // viewer->addLine(line_coeff1, id1, 0);

  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
}

void show_clos_fart_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::VectorXf closfart, Eigen::Vector4f centroid)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Showing Closest and Farthest Points to Center"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.4);
  
  pcl::ModelCoefficients line_coeff[2];
  
  std::string id[2];
  for (int i=0; i < 2; ++i)
  {
      line_coeff[i].values.resize (6);    // We need 6 values
      line_coeff[i].values[0] = centroid[0];
      line_coeff[i].values[1] = centroid[1];
      line_coeff[i].values[2] = centroid[2];
      if (i==0)
      {
        line_coeff[i].values[3] = closfart[0] - centroid[0];
        line_coeff[i].values[4] = closfart[1] - centroid[1];
        line_coeff[i].values[5] = closfart[2] - centroid[2];
        // std::cout << "just checking ----------------" << std::endl;
      }
      else
      {
        line_coeff[i].values[3] = closfart[4] - centroid[0];
        line_coeff[i].values[4] = closfart[5] - centroid[1];
        line_coeff[i].values[5] = closfart[6] - centroid[2];
      }
      id[i] = "line"+i;
      viewer->addLine(line_coeff[i], id[i], 0);
  }

  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
}

void show_TargetCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Matrix4f RotMatrix)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Taget Coordinate Frame"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.4);

  std::vector<pcl::PointXYZ> pointSet;
  pcl::PointXYZ point; 
  std::string id[3];
  for (int i=0; i < 4; ++i)
  {
      if (i==0)
      {
        point.x = RotMatrix(0,3);
        point.y = RotMatrix(1,3);
        point.z = RotMatrix(2,3);
        pointSet.push_back(point);
      }
      if (i==1)
      {
        point.x = RotMatrix(0,3)+RotMatrix(0,0);
        point.y = RotMatrix(1,3)+RotMatrix(1,0);
        point.z = RotMatrix(2,3)+RotMatrix(2,0);
        pointSet.push_back(point);
        id[i-1] = "line"+i;
        viewer->addLine(pointSet[0], pointSet[i], 1, 0, 0, id[i-1], 0);     // select red color
      }
      if (i==2)
      {
        point.x = RotMatrix(0,3)+RotMatrix(0,1);
        point.y = RotMatrix(1,3)+RotMatrix(1,1);
        point.z = RotMatrix(2,3)+RotMatrix(2,1);
        pointSet.push_back(point);
        id[i-1] = "line"+i;
        viewer->addLine(pointSet[0], pointSet[i], 0, 1, 0, id[i-1], 0);  // select green color
      }
      if (i==3)
      {
        point.x = RotMatrix(0,3)+RotMatrix(0,2);
        point.y = RotMatrix(1,3)+RotMatrix(1,2);
        point.z = RotMatrix(2,3)+RotMatrix(2,2);
        pointSet.push_back(point);
        id[i-1] = "line"+i;
        viewer->addLine(pointSet[0], pointSet[i], 0, 0, 1, id[i-1], 0);  // select blue color
      }
  }

  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
  }
}

// this one save unorganined point cloud
void pcd_write_unorganized(const std::string& filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{  
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "prep_"+filename+".pcd";
  writer.write<pcl::PointXYZRGB> (ss.str(), *cloud, false); //*
  std::cout<< "Saved "<< cloud->points.size()<< " data points to file: "<< ss.str() << std::endl;
}


// estimate the normal vector 
pcl::PointCloud<pcl::PointNormal>::Ptr estimate_normal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::PointNormal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  tree->setInputCloud(cloud);
  ne.setSearchMethod (tree);
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
  
  // Output datasets
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.02);

  // Compute the features
  ne.compute (*cloud_normals);

  // std::cout<< "The number of normal vectors: "<< cloud_normals->points.size() << std::endl; //should have the same size as the input cloud->points.size ()*
  return cloud_normals;
}

// normals vector of a surface
Eigen::Vector3f average_normals(pcl::PointCloud<pcl::PointNormal>::Ptr normals)
{
  float sum_x=0.0, sum_y=0.0, sum_z=0.0;
  for (int i=0; i<normals->size(); i++)
  {
    sum_x += normals->points[i].normal_x;

    sum_y += normals->points[i].normal_y;
    sum_z += normals->points[i].normal_z;
    // if ((i==1)or(i ==100)or (i==500))
    // {
    //   std::cout << "normal_x: " << normals->points[i].normal_x << std::endl;
    //   std::cout << "normal_y: " << normals->points[i].normal_y << std::endl;
    //   std::cout << "normal_z: " << normals->points[i].normal_z << std::endl;
    // }
  }
  Eigen::Vector3f aver_nor;
  aver_nor[0] = sum_x/(normals->size());
  aver_nor[1] = sum_y/(normals->size());
  aver_nor[2] = sum_z/(normals->size());
  // std::cout << "the number of point cloud - normal vector: " << normals->size() << std::endl; 
  return aver_nor;
}

// get the maximum and minimum dimension of point cloud according to xyz
Eigen::VectorXf cal_MaxMinxyz(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp)
{
    Eigen::VectorXf max_min(6);
    float max_x, min_x, max_y, min_y, max_z, min_z;
    for (int i = 0; i < cloudp->size(); i++)
    {    
        if (i ==0)
        {
            max_x = cloudp->points[i].x;
            min_x = cloudp->points[i].x;
            max_y = cloudp->points[i].y;
            min_y = cloudp->points[i].y;
            max_z = cloudp->points[i].z;
            min_z = cloudp->points[i].z;
        }
        // For x
        if (cloudp->points[i].x > max_x)
            max_x = cloudp->points[i].x;
        if (cloudp->points[i].x < min_x)
            min_x = cloudp->points[i].x;
        // For y
        if (cloudp->points[i].y > max_y)
            max_y = cloudp->points[i].y;
        if (cloudp->points[i].y < min_y)
            min_y = cloudp->points[i].y;
        // For z
        if (cloudp->points[i].z > max_z)
            max_z = cloudp->points[i].z;
        if (cloudp->points[i].z < min_z)
            min_z = cloudp->points[i].z;
    }
    max_min[0] = min_x;
    max_min[1] = max_x;
    max_min[2] = min_y;
    max_min[3] = max_y;
    max_min[4] = min_z;
    max_min[5] = max_z;
    return max_min;
}

// get the set of boundary points
std::vector<Eigen::Vector3f> cal_BoundaryPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp, Eigen::VectorXf MaxMinxyz)
{
    
    std::vector<Eigen::Vector3f> boundaryPoint;
    Eigen::Vector3f point1(0,0,0), point2(0,0,0);
    float start, distance1, distance2, max_distance;
    Eigen::VectorXf maxmin(6);
    for (int i =0; i < 6; i++)
        maxmin[i] = MaxMinxyz[i];

    int count_check = 0;
    // start by z-direction - find the farthest point couples in each z - slicing
    start = maxmin[4];
    while (start < maxmin[5])
    {   
        for (int i = 0; i< cloudp->size(); i++)
        {
            if (((cloudp->points[i].z - start) < 0.008) and ((cloudp->points[i].z - start) > -0.008))
            {
                // std::cout << cloudp->points[i].x << "" << cloudp->points[i].y << " " << cloudp->points[i].z << std::endl;
                if (count_check ==0)
                {
                    point1[0] = cloudp->points[i].x;
                    point1[1] = cloudp->points[i].y;
                    point1[2] = cloudp->points[i].z;
                    // std::cout << " This come one time ------------------------------------" << std::endl;
                }
                else if (count_check ==1)
                {
                    point2[0] = cloudp->points[i].x;
                    point2[1] = cloudp->points[i].y;
                    point2[2] = cloudp->points[i].z;
                    max_distance = pow((point2[0]-point1[0]),2)+pow((point2[1]-point1[1]),2)+pow((point2[2]-point1[2]),2);
                    // std::cout << " This come one time ++++++++++++++++++++++++++++++++++++" << std::endl;
                }
                else
                {
                    distance1 = pow((point1[0]-cloudp->points[i].x),2)+pow((point1[1]-cloudp->points[i].y),2)+pow((point1[2]-cloudp->points[i].z),2);
                    distance2 = pow((point2[0]-cloudp->points[i].x),2)+pow((point2[1]-cloudp->points[i].y),2)+pow((point2[2]-cloudp->points[i].z),2);
                    if (distance2 < distance1)
                        if (distance1 > max_distance)
                        {
                            max_distance = distance1;
                            point2[0] = cloudp->points[i].x;
                            point2[1] = cloudp->points[i].y;
                            point2[2] = cloudp->points[i].z;
                        }    
                    if (distance2 > distance1)
                    {
                        if (distance2 > max_distance)
                        {
                            max_distance = distance2;
                            point1[0] = cloudp->points[i].x;
                            point1[1] = cloudp->points[i].y;
                            point1[2] = cloudp->points[i].z;
                        }    
                    }
                }
                count_check += 1;
            }
            // std::cout << "The count_check number is: " << count_check << std::endl;
        }
        count_check =0;
        // std::cout << point1[0] << " " << point1[1] << " "<< point1[2] << std::endl;
        // std::cout << point2[0] << " " << point2[1] << " "<< point2[2] << std::endl;
        // std::cout << std::endl;
    
        boundaryPoint.push_back(point1);
        boundaryPoint.push_back(point2);
        start += 0.015;
        point1[0] = 0.0;
        point1[1] = 0.0;
        point1[2] = 0.0;
        point2[0] = 0.0;
        point2[1] = 0.0;
        point2[2] = 0.0;
    }

    // start by y-direction - find the farthest point couples in each y-slicing
    count_check = 0;
    start = maxmin[2];
    point1[0] = 0.0;
    point1[1] = 0.0;
    point1[2] = 0.0;
    point2[0] = 0.0;
    point2[1] = 0.0;
    point2[2] = 0.0;
    while (start < maxmin[3])
    {   
        for (int i = 0; i< cloudp->size(); i++)
        {
            if (((cloudp->points[i].y-start) < 0.008) and ((cloudp->points[i].y-start) > -0.008))
            {
                if (count_check ==0)
                {
                    point1[0] = cloudp->points[i].x;
                    point1[1] = cloudp->points[i].y;
                    point1[2] = cloudp->points[i].z;
                }
                else if (count_check ==1)
                {
                    point2[0] = cloudp->points[i].x;
                    point2[1] = cloudp->points[i].y;
                    point2[2] = cloudp->points[i].z;
                    max_distance = pow((point2[0]-point1[0]),2)+pow((point2[1]-point1[1]),2)+pow((point2[2]-point1[2]),2);
                }
                else
                {
                    distance1 = pow((point1[0]-cloudp->points[i].x),2)+pow((point1[1]-cloudp->points[i].y),2)+pow((point1[2]-cloudp->points[i].z),2);
                    distance2 = pow((point2[0]-cloudp->points[i].x),2)+pow((point2[1]-cloudp->points[i].y),2)+pow((point2[2]-cloudp->points[i].z),2);
                    if (distance2 < distance1)
                    {
                        if (distance1 > max_distance)
                        {
                            max_distance = distance1;
                            point2[0] = cloudp->points[i].x;
                            point2[1] = cloudp->points[i].y;
                            point2[2] = cloudp->points[i].z;
                        }    
                    }
                    if (distance2 > distance1)
                    {
                        if (distance2 > max_distance)
                        {
                            max_distance = distance2;
                            point1[0] = cloudp->points[i].x;
                            point1[1] = cloudp->points[i].y;
                            point1[2] = cloudp->points[i].z;
                        }    
                    }
                }
                count_check += 1;
            }
        }
        count_check = 0;
        boundaryPoint.push_back(point1);
        boundaryPoint.push_back(point2);
        start += 0.015;
        point1[0] = 0.0;
        point1[1] = 0.0;
        point1[2] = 0.0;
        point2[0] = 0.0;
        point2[1] = 0.0;
        point2[2] = 0.0;
    }

    // start by x-direction - find the farthest point couples in each x-slicing
    count_check = 0;
    point1[0] = 0.0;
    point1[1] = 0.0;
    point1[2] = 0.0;
    point2[0] = 0.0;
    point2[1] = 0.0;
    point2[2] = 0.0;
    start = maxmin[0];
    while (start < maxmin[1])
    {   
        for (int i = 0; i< cloudp->size(); i++)
        {
            if (((cloudp->points[i].x-start)< 0.008) and ((cloudp->points[i].x-start) > -0.008))
            {
                if (count_check ==0)
                {
                    point1[0] = cloudp->points[i].x;
                    point1[1] = cloudp->points[i].y;
                    point1[2] = cloudp->points[i].z;
                }
                else if (count_check ==1)
                {
                    point2[0] = cloudp->points[i].x;
                    point2[1] = cloudp->points[i].y;
                    point2[2] = cloudp->points[i].z;
                    max_distance = pow((point2[0]-point1[0]),2)+pow((point2[1]-point1[1]),2)+pow((point2[2]-point1[2]),2);
                }
                else
                {
                    distance1 = pow((point1[0]-cloudp->points[i].x),2)+pow((point1[1]-cloudp->points[i].y),2)+pow((point1[2]-cloudp->points[i].z),2);
                    distance2 = pow((point2[0]-cloudp->points[i].x),2)+pow((point2[1]-cloudp->points[i].y),2)+pow((point2[2]-cloudp->points[i].z),2);
                    if (distance2 < distance1)
                    {
                        if (distance1 > max_distance)
                        {
                            max_distance = distance1;
                            point2[0] = cloudp->points[i].x;
                            point2[1] = cloudp->points[i].y;
                            point2[2] = cloudp->points[i].z;
                        }    
                    }
                    if (distance2 > distance1)
                    {
                        if (distance2 > max_distance)
                        {
                            max_distance = distance2;
                            point1[0] = cloudp->points[i].x;
                            point1[1] = cloudp->points[i].y;
                            point1[2] = cloudp->points[i].z;
                        }    
                    }
                }
                count_check += 1;
            }
        }
        boundaryPoint.push_back(point1);
        boundaryPoint.push_back(point2);
        start += 0.015;
        count_check = 0;
        point1[0] = 0.0;
        point1[1] = 0.0;
        point1[2] = 0.0;
        point2[0] = 0.0;
        point2[1] = 0.0;
        point2[2] = 0.0;
    }
    return boundaryPoint;
}
// get the closest and farthest points to the centroid
Eigen::VectorXf cal_Clos_Fart_PointstoCentroid(std::vector<Eigen::Vector3f> B_Point, Eigen::Vector4f centroid)
{
    Eigen::VectorXf clos_fart(8);
    float clos_x=0.0, clos_y=0.0, clos_z=0.0, fart_x=0.0, fart_y=0.0, fart_z = 0.0;
    float max_distance = 0.0, min_distance =0.0, distance = 0.0;
    for (int i = 0; i < B_Point.size(); i++)
    {
        distance = pow((centroid[0]-B_Point[i][0]),2)+pow((centroid[1]-B_Point[i][1]),2)+pow((centroid[2]-B_Point[i][2]),2);
        // std::cout << "The distance is: " << sqrt(distance) << std::endl;
        if (i == 0)
        {
            clos_x = B_Point[i][0];
            clos_y = B_Point[i][1];
            clos_z = B_Point[i][2];
            fart_x = B_Point[i][0];
            fart_y = B_Point[i][1];
            fart_z = B_Point[i][2];
            max_distance = distance;
            min_distance = distance;
        }
        // Farthest point
        if (distance > max_distance)
        {
            max_distance = distance;
            fart_x = B_Point[i][0];
            fart_y = B_Point[i][1];
            fart_z = B_Point[i][2];
        }
        // Closest point
        if (distance < min_distance)
        {
            min_distance = distance;
            clos_x = B_Point[i][0];
            clos_y = B_Point[i][1];
            clos_z = B_Point[i][2];
        }
    }
    // convert back to regular distance
    min_distance = sqrt(min_distance);
    max_distance = sqrt(max_distance);
    clos_fart[0] = clos_x;
    clos_fart[1] = clos_y;
    clos_fart[2] = clos_z;
    clos_fart[3] = min_distance;
    clos_fart[4] = fart_x;
    clos_fart[5] = fart_y;
    clos_fart[6] = fart_z;
    clos_fart[7] = max_distance;
    // std::cout << "The max distance is: "<< max_distance << std::endl;
    // std::cout << "The min distance is: "<< min_distance << std::endl;
    return clos_fart;
}


// calculate the area by vector cross product
float cal_AreaPolygon(const pcl::PointCloud<pcl::PointXYZRGB> &polygon )
{
  float area=0.0;
  int num_points = polygon.size();
  int j = 0;
  Eigen::Vector3f va,vb,res;
  res(0) = res(1) = res(2) = 0.0f;
  for (int i = 0; i < num_points; ++i)
  {
    j = (i + 1) % num_points;
    // convert a point to a vector form
    va = polygon[i].getVector3fMap();
    vb = polygon[j].getVector3fMap();
    res += va.cross(vb);
  }
  // calculate the parallelogram
  area=res.norm();
  return area*0.5;
}

// calculate the Quaternions
geometry_msgs::Pose cal_Pose(Eigen::Vector4f centroid, Eigen::VectorXf clos_fart, Eigen::Vector3f ave_normal)
{
    geometry_msgs::Pose robot_pose;
    Eigen::Matrix3f orie_matrix;
    float norm=0.0;
    // z- axis
    orie_matrix(0,2) = ave_normal(0);
    orie_matrix(1,2) = ave_normal(1);
    orie_matrix(2,2) = ave_normal(2);
    // normalize
    norm = sqrt(pow(orie_matrix(0,2), 2) + pow(orie_matrix(1,2), 2) + pow(orie_matrix(2,2), 2));
    orie_matrix(0,2) = orie_matrix(0,2)/norm;
    orie_matrix(1,2) = orie_matrix(1,2)/norm;
    orie_matrix(2,2) = orie_matrix(2,2)/norm;
    
    // x axis
    orie_matrix(0,0) = clos_fart(0) - centroid(0);
    orie_matrix(1,0) = clos_fart(1) - centroid(1);
    orie_matrix(2,0) = clos_fart(2) - centroid(2);
    // normalize
    norm = sqrt(pow(orie_matrix(0,0), 2) + pow(orie_matrix(1,0), 2) + pow(orie_matrix(2,0), 2));
    orie_matrix(0,0) = orie_matrix(0,0)/norm;
    orie_matrix(1,0) = orie_matrix(1,0)/norm;
    orie_matrix(2,0) = orie_matrix(2,0)/norm;
    
    // y azis - using cross product
    orie_matrix(0,1) = orie_matrix(1,2)*orie_matrix(2,0) - orie_matrix(2,2)*orie_matrix(1,0);
    orie_matrix(1,1) = orie_matrix(2,2)*orie_matrix(0,0) - orie_matrix(0,2)*orie_matrix(2,0);
    orie_matrix(2,1) = orie_matrix(0,2)*orie_matrix(1,0) - orie_matrix(1,2)*orie_matrix(0,0);
    // normalize
    norm = sqrt(pow(orie_matrix(0,1), 2) + pow(orie_matrix(1,1), 2) + pow(orie_matrix(2,1), 2));
    orie_matrix(0,1) = orie_matrix(0,1)/norm;
    orie_matrix(1,1) = orie_matrix(1,1)/norm;
    orie_matrix(2,1) = orie_matrix(2,1)/norm;
    // std::cout << orie_matrix(0,0) << " " << orie_matrix(1,0) << " " << orie_matrix(2,0) << std::endl;
    // std::cout << orie_matrix(0,1) << " " << orie_matrix(1,1) << " " << orie_matrix(2,1) << std::endl;
    // std::cout << orie_matrix(0,2) << " " << orie_matrix(1,2) << " " << orie_matrix(2,2) << std::endl;
    // calculate the quaternion
    robot_pose.orientation.w = 0.5*sqrt(1+orie_matrix(0,0)+orie_matrix(1,1)+orie_matrix(2,2));
    robot_pose.orientation.x = 0.25*(orie_matrix(2,1)-orie_matrix(1,2))/robot_pose.orientation.w;
    robot_pose.orientation.y = 0.25*(orie_matrix(0,2)-orie_matrix(2,0))/robot_pose.orientation.w;
    robot_pose.orientation.z = 0.25*(orie_matrix(1,0)-orie_matrix(0,1))/robot_pose.orientation.w;
    robot_pose.position.x = centroid(0);
    robot_pose.position.y = centroid(1);
    robot_pose.position.z = centroid(2);

    return robot_pose;
}

// convert Quaternion to heterousgeneous matrix
Eigen::Matrix4f convert_PosetoRotationMatrix(geometry_msgs::Pose robot_pose)
{
    // geometry_msgs::Pose robot_pose;
    Eigen::Matrix4f hete_matrix;
    
    // for translation
    hete_matrix(0,3) = robot_pose.position.x;
    hete_matrix(1,3) = robot_pose.position.y;
    hete_matrix(2,3) = robot_pose.position.z;
    hete_matrix(3,3) = 1;
    // the last row
    hete_matrix(3,0) = 0;
    hete_matrix(3,1) = 0;
    hete_matrix(3,2) = 0;
    
    // for rotation part
    hete_matrix(0,0) = 1 - 2*pow(robot_pose.orientation.y,2) - 2*pow(robot_pose.orientation.z,2);
    hete_matrix(1,0) = 2*(robot_pose.orientation.x*robot_pose.orientation.y + robot_pose.orientation.z*robot_pose.orientation.w);
    hete_matrix(2,0) = 2*(robot_pose.orientation.x*robot_pose.orientation.z - robot_pose.orientation.y*robot_pose.orientation.w);

    hete_matrix(0,1) = 2*(robot_pose.orientation.x*robot_pose.orientation.y - robot_pose.orientation.z*robot_pose.orientation.w);
    hete_matrix(1,1) = 1 - 2*pow(robot_pose.orientation.x,2) - 2*pow(robot_pose.orientation.z,2);
    hete_matrix(2,1) = 2*(robot_pose.orientation.x*robot_pose.orientation.w + robot_pose.orientation.y*robot_pose.orientation.z);

    hete_matrix(0,2) = 2*(robot_pose.orientation.x*robot_pose.orientation.z + robot_pose.orientation.y*robot_pose.orientation.w);
    hete_matrix(1,2) = 2*(robot_pose.orientation.y*robot_pose.orientation.z - robot_pose.orientation.x*robot_pose.orientation.w);
    hete_matrix(2,2) = 1 - 2*pow(robot_pose.orientation.x,2) - 2*pow(robot_pose.orientation.y,2);

    return hete_matrix;
}


// calculate the area by approximation
float cal_PlaneArea(const Eigen::VectorXf clos_fart)
{
  return 4*clos_fart[3]*sqrt(pow(clos_fart[7],2)-pow(clos_fart[3],2));
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "targetPlane");
    // ros::NodeHandle nh;
    // ros::Subcriber sub = nh.subcribe();

    pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud2; //(new pcl::PointCloud<pcl::PointXYZ>);
    std::string filename, filenamejpg, filenamepcd, root, fileAfterProcess; 
    // filename= "4objectnew1";
    filename= "new_data";
    
    root = "/home/buivn/bui_ws/src/pcd_filter/src/";
    // root = "/home/buivn/bui_ws/";
    filenamepcd = root +filename+".pcd";

    cloud = pcd_read(filenamepcd);
    show_pcd(cloud.makeShared(), 0, "Original point cloud data");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp1 (new pcl::PointCloud<pcl::PointXYZRGB>), cloudp4 (new pcl::PointCloud<pcl::PointXYZRGB>),
              cloudp2 (new pcl::PointCloud<pcl::PointXYZRGB>), cloudp3 (new pcl::PointCloud<pcl::PointXYZRGB>); 
    
    cloudp1 = passthrough_filter(cloud.makeShared());
    // show_pcd(cloudp1, 0, "After passThrough filtering");
    // pcd_write_organized(filename, cloudp1);

    // show_pcd only work with pointer->use cloud.makeShared() to return a pointer
    // downsampling the data - cloudp1 is a pointer
    cloudp1 = downsampling(cloudp1);
    // show_pcd(cloudp1, 0, "After downsampling");
    // fileAfterProcess = "afterprocess";
    // pcd_write(filename, cloudp1);

    // cloudp2 is a pointer -  planar filter
    cloudp2 = get_planes(cloudp1);

    // show_pcd(cloudp2, 0, "After planar filtering");


    // pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals = estimate_normal(cloudp2);
    // show_normals(cloudp2, cloud_normals);
    Eigen::Vector3f ave = average_normals(cloud_normals);
    std::cout << "the average normals: " << ave << std::endl;
    Eigen::Vector4f centroid;
    // calculate the centroid of the plane
    pcl::compute3DCentroid(*cloudp2, centroid);
    Eigen::MatrixXf centroid_vector(1, 4);
    centroid_vector.row(0) = centroid;
    // show_centroid(cloudp2, centroid_vector);
    std::cout << "the centroid of the plane: " << centroid << std::endl;
    Eigen::VectorXf MaxMinxyz(6);
    MaxMinxyz = cal_MaxMinxyz(cloudp2);
    std::vector<Eigen::Vector3f> BoundaryPoint = cal_BoundaryPoint(cloudp2, MaxMinxyz);
    std::cout << BoundaryPoint.size() << std::endl;
    

    present_setOfPointasPointCloud(BoundaryPoint);

    Eigen::VectorXf clos_fart(8);
    
    clos_fart = cal_Clos_Fart_PointstoCentroid(BoundaryPoint, centroid);
    show_clos_fart_points(cloudp2, clos_fart, centroid);
    float area = cal_PlaneArea(clos_fart);
    std::cout << "The area is: " << area << std::endl;
    geometry_msgs::Pose targetPose;
    targetPose = cal_Pose(centroid, clos_fart, ave);
    std::cout << targetPose << std::endl;
    
    Eigen::Matrix4f rota_matrix;
    rota_matrix = convert_PosetoRotationMatrix(targetPose);
    show_TargetCoordinate(cloudp2, rota_matrix);    
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_check (new pcl::PointCloud<pcl::PointXYZ>);
    
    return (0);
}
