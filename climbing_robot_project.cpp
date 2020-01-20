//#include <gpg/cloud_camera.h>
#include <iostream>
#include <stdio.h>
#include <pcl/io/pcd_io.h>

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
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/don.h>


#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <Eigen/Dense>
// #include <opencv2/opencv.hpp>
#include <math.h>
#include <string>

// using namespace cv;

pcl::PointCloud<pcl::PointXYZRGB> pcd_read(const std::string& filename)
{
    //pcd_original = loadPointCloudFromFile(filename);
  
    pcl::PointCloud<pcl::PointXYZRGB> cloud; //(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PCLPointCloud2 cloud;

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
    distance_pass.setFilterLimits (0.2, 1);
    //pass.setFilterLimitsNegative (true);
    distance_pass.filter (*cloud_filtered);

    std::cerr << "Cloud after passthrough filtering: " << cloud_filtered->size() << std::endl;

return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // this code run with input as argument of point cloud data 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::cerr << "PointCloud before downsampling: "<< cloud->width*cloud->height <<" data points." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    // define the size of the the leaf (in meter) = voxel size
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    // perform the downsampling filter then save the new data to cloud_filtered
    sor.filter (*cloud_filtered);


    std::cerr << "PointCloud after downsampling: "<<cloud_filtered->width*cloud_filtered->height<<" data points." << std::endl;

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
        std::cerr <<"PointCloud representing the planar component: "<< cloud_p->width*cloud_p->height <<" data points."<< std::endl;
        cloud.swap (cloud_p);
        cloudsize = cloud_p->width*cloud_p->height;
    }
    return cloud;
}

pcl::visualization::PCLVisualizer::Ptr show_pcd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int clus_order, const std::string& titlename)
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
        //viewer->addText("Cluster "+ ss2, 80, 50, "v1 text", v1);
        //addText (const std::string &text, int xpos, int ypos, int fontsize,double r, double g, double b, const std::string &id="",int viewport=0)
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
        //std::this_thread::sleep_for(100ms);
    }
    return viewer;
}

pcl::visualization::PCLVisualizer::Ptr show_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::MatrixXf& centroid3D)
{
  // int v1(0);
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
      line_coeff[i].values.resize (6);    // We need 6 values
      line_coeff[i].values[0] = 0; //point_on_line.x ();
      line_coeff[i].values[1] = 0;//point_on_line.y ();
      line_coeff[i].values[2] = 0; //point_on_line.z ();
      line_coeff[i].values[3] = centroid3D(i,0);//-0.078; //line_direction.x ();
      line_coeff[i].values[4] = centroid3D(i,1);//0.126; //line_direction.y ();
      line_coeff[i].values[5] = centroid3D(i,2);//0.49; //line_direction.z ();
  
      id[i] = "line"+i;
      viewer->addLine(line_coeff[i], id[i], 0);
  }
  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    //std::this_thread::sleep_for(100ms);
  }
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr show_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr normals)
{
    // int v1(0);
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
    //std::this_thread::sleep_for(100ms);
  }
  return (viewer);
}


int pcd_write(const std::string& filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
  float sum_x=0, sum_y=0, sum_z=0;
  for (int i=0; i<normals->size(); i++)
  {
    sum_x += normals->points[i].normal_x;

    sum_y += normals->points[i].normal_y;
    sum_z += normals->points[i].normal_z;
    // if ((i==1)or(i ==0))
    // {
    //   std::cout << "normal_x: " << sum_x << std::endl;
    //   std::cout << "normal_y: " << sum_y << std::endl;
    //   std::cout << "normal_z: " << sum_z << std::endl;
    // }
  }
  Eigen::Vector3f aver_nor;
  aver_nor[0] = sum_x/(normals->size());
  aver_nor[1] = sum_y/(normals->size());
  aver_nor[2] = sum_z/(normals->size());
  return aver_nor;
}

pcl::visualization::PCLVisualizer::Ptr show_aver_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Vector3f normals, Eigen::Vector4f centroid)
{
  // int v1(0);
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Showing Normals"));
  viewer->setBackgroundColor (0, 0, 0);
  // Add text to the screen
  //viewer->addText("Radius: 0.002", 50, 50, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.4);
  
  pcl::ModelCoefficients line_coeff;
  
  line_coeff.values.resize (6);    // We need 6 values
  line_coeff.values[0] = centroid[0];
  line_coeff.values[1] = centroid[1];
  line_coeff.values[2] = centroid[2]; 
  line_coeff.values[3] = centroid[0]+normals[0];
  line_coeff.values[4] = centroid[1]+normals[1];
  line_coeff.values[5] = centroid[2]+normals[2];

  std::string id = "normal vector";
  viewer->addLine(line_coeff, id, 0);

  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    //std::this_thread::sleep_for(100ms);
  }
  return (viewer);
}

float calculateAreaPolygon(const pcl::PointCloud<pcl::PointXYZ> &polygon )
{
  float area=0.0;
  int num_points = polygon.size();
  int j = 0;
  Eigen::Vector3f va,vb,res;
  res(0) = res(1) = res(2) = 0.0f;
  for (int i = 0; i < num_points; ++i)
  {
    j = (i + 1) % num_points;
    va = polygon[i].getVector3fMap();
    vb = polygon[j].getVector3fMap();
    res += va.cross(vb);
  }
  area=res.norm();
  return area*0.5;
}


int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud2; //(new pcl::PointCloud<pcl::PointXYZ>);
    std::string filename, filenamejpg, filenamepcd, root, fileAfterProcess; 
    filename= "4objectnew1";
    
    root = "/home/buivn/bui_IROS2020/src/pclFilCen/";
    filenamepcd = root +filename+".pcd";

    cloud = pcd_read(filenamepcd);
    // show_pcd(cloud.makeShared(), 0, "Original point cloud data");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp1 (new pcl::PointCloud<pcl::PointXYZRGB>), cloudp4 (new pcl::PointCloud<pcl::PointXYZRGB>),
              cloudp2 (new pcl::PointCloud<pcl::PointXYZRGB>), cloudp3 (new pcl::PointCloud<pcl::PointXYZRGB>); 
    cloudp1 = passthrough_filter(cloud.makeShared());
    // show_pcd(cloudp1, 0, "After passThrough filtering");

    // show_pcd only work with pointer->use cloud.makeShared() to return a pointer
    // downsampling the data - cloudp1 is a pointer
    cloudp1 = downsampling(cloudp1);
    // show_pcd(cloudp1, 0, "After downsampling");
    // fileAfterProcess = "afterprocess";
    // pcd_write(filename, cloudp1);

    // cloudp2 is a pointer -  planar filter
    cloudp2 = get_planes(cloudp1);

    show_pcd(cloudp2, 0, "After planar filtering");


    // pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals = estimate_normal(cloudp2);
    show_normals(cloudp2, cloud_normals);
    Eigen::Vector3f ave = average_normals(cloud_normals);
    std::cout << "the average normals: " << ave << std::endl;
    Eigen::Vector4f centroid;
    // calculate the centroid of the plane
    pcl::compute3DCentroid(*cloudp2, centroid);
    std::cout << "the centroid of the plane: " << centroid << std::endl;
    show_aver_normals(cloudp2, ave, centroid);
    
    // Output datasets
    

    // std::vector<pcl::PointIndices> cluster_indices;
    // cluster_indices = extract_object(cloudp2);

    
    return (0);
}
