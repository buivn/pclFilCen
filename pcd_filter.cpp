//#include <gpg/cloud_camera.h>
#include <iostream>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
//#include <thread>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace cv;




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
    distance_pass.setFilterLimits (0.4, 1.0);
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
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    // perform the downsampling filter then save the new data to cloud_filtered
    sor.filter (*cloud_filtered);


    std::cerr << "PointCloud after downsampling: "<<cloud_filtered->width*cloud_filtered->height<<" data points." << std::endl;

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr planar_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true); //> set this function using refine optimize Coefficient
    // Mandatory - set parameter for the planar filter
    seg.setModelType (pcl::SACMODEL_PLANE); // -> segment the plane on the point cloud
    // -> RANSAC = Random Sample Consensus = estimator method
    seg.setMethodType (pcl::SAC_RANSAC);    //-> use 2 step to determine the outliers or inliers
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.018);    // -> tolerate range to define the inliers & outlier
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int i = 0, nr_points = (int) cloud->points.size ();
    
    // While 30% of the original cloud is still there
    while (cloud->points.size() > 0.3*nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        // seg.segment() result in the inliers point and coefficients
        seg.segment (*inliers, *coefficients); //save all points belong to the plane to inliners
        // outlier -> mau ngoai lai, out of observation range -> skew the estimation result
        // inlier -> sample lie inside the range -> fit with the model 
        if (inliers->indices.size () == 0) // -> if there is no inliers
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud);
        // input a pointer to vector of indices (inliers)->represent the input data
        extract.setIndices (inliers);
        // set whether the indices should be returned, or all points_except_the indices
        //extract.setNegative (false); //false -> return the indices
        // save the result to the cloud_p
        //extract.filter (*cloud_p);
        //std::cerr <<"PointCloud representing the planar component: "<< cloud_p->width*cloud_p->height <<" data points."<< std::endl;

        // Create the filtering object
        extract.setNegative (true); //true -> return all point of data except the input indices
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        i++;
    }
    return cloud;
}

Eigen::Matrix4f object_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
            std::vector<pcl::PointIndices> c_indices, int nCluster)
{
    //typedef Matrix< float, 4, 4 >   Eigen::Matrix4f
    //std::cout << "The number of cluster is: " << nCluster << std::endl;
    Eigen::MatrixXf centroid_vector(nCluster, 4);
    centroid_vector << Eigen::Matrix4f::Zero(nCluster,4);
    // just check the initial value of the matrix
    //std::cout << "The number of cluster is: " << nCluster << std::endl;
    
    int count = 0;
    Eigen::Vector4f centroid;

    // go to each point cloud cluster in the cluster vector - cluster_indices
    for (std::vector<pcl::PointIndices>::const_iterator it = c_indices.begin (); it != c_indices.end (); ++it)
    {     
        pcl::compute3DCentroid(*cloud, *it, centroid);
        // save the centroid to the centroi_vector
        centroid_vector.row(count) = centroid;       
        // just print out the centroid position
        ++count;
    }
    return centroid_vector;
}

//pcl::PointCloud<pcl::PointXYZ>::Ptr extract_object(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
std::vector<pcl::PointIndices> extract_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);

    // create the indices vector for clusters in the point cloud
    std::vector<pcl::PointIndices> cluster_indices;
    // store the cluster extraction of point cloud
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // the radius of the sphere to determine the neighbor points
    ec.setClusterTolerance (0.02); // 2cm
    // minimum of number of point in a cluster
    ec.setMinClusterSize (70);
    // maximum of number of point in a cluster
    ec.setMaxClusterSize (25000);
    // the search method is tree
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    // perform then save the found clusters to cluster_indices
    ec.extract(cluster_indices); // cluster_indices = set of point cloud clusters
    
    return cluster_indices;
    
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr integrate_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
            std::vector<pcl::PointIndices> cluster_indices)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr com_cluster (new pcl::PointCloud<pcl::PointXYZRGB>); 
    // go to each point cloud cluster in the cluster vector - cluster_indices
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {    
        // copy all the point cloud of one cluster to a new point cloud data
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            // copy each point cloud of the cluster to a new point cloud data
            com_cluster->points.push_back(cloud->points[*pit]);
            //std::cout << "Copying something to ........" << *it << std::endl;
        
        com_cluster->width = com_cluster->points.size ();
        com_cluster->height = 1;
        com_cluster->is_dense = true;
    }
    return com_cluster;
}


void show_pcd1(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& titlename)
{ 
   //... populate cloud
   pcl::visualization::CloudViewer viewer(titlename);

   // viewer.showCloud only work with pointer
   viewer.showCloud(cloud, "cloud");
   while (!viewer.wasStopped())
   {
   }
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

    int v1(0);
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


int pcd_write(const std::string& filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{  
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "prep_"+filename+".pcd";
  writer.write<pcl::PointXYZRGB> (ss.str(), *cloud, false); //*
  std::cout<< "Saved "<< cloud->points.size()<< " data points to file: "<< ss.str() << std::endl;
}

Eigen::Vector2i point3Dto_pixel(Eigen::Vector4f pc_coord, Eigen::Matrix3f cmatrix)
{
    Eigen::Vector2i pixel_pos;
    //std::cout << "inside the point3Dto_pixel" << std::endl;
    pixel_pos(0) = (int)(pc_coord(0)*cmatrix(0,0)/pc_coord(2) + cmatrix(0,2));
    pixel_pos(1) = (int)(pc_coord(1)*cmatrix(1,1)/pc_coord(2) + cmatrix(1,2));
    //std::cout << "inside the point3Dto_pixel" << std::endl;
    return pixel_pos;
}

void drawboxrgb(Eigen::MatrixX2i pix_poss, const std::string& filename)
{
    Mat image;
    //image = imread("sample1.jpg", CV_LOAD_IMAGE_COLOR);
    image = imread(filename, CV_LOAD_IMAGE_COLOR);
    Point pt1, pt2;
    int size = 10;

    //std::string ss = pix_poss.row();
    int clusternumber = pix_poss.size()/2;
    //std::cout << "the number of point centroid: " << clusternumber << std::endl;
    for (int it = 0; it < clusternumber; ++it)
    {

        pt1.x = pix_poss(it,0) - size;
        pt1.y = pix_poss(it,1) - size;
        pt2.x = pix_poss(it,0) + size;
        pt2.y = pix_poss(it,1) + size;

        rectangle(image, pt1, pt2, Scalar(0, 255, 255 ), -1, 8);
    }

    namedWindow("Display window", WINDOW_AUTOSIZE);
    imshow("Display window", image);
    waitKey(15000);
    //std::cout << filename << std::endl;
}

int select_object(Eigen::Vector2i grasp_pos, Eigen::Matrix2Xi ob_centroids)
{
    int distance, distance_x, distance_y, threshold, ob_grasp;
    threshold = 40;
    ob_grasp = 20;
    int clusternumber = ob_centroids.size()/2;
    //std::cout << "The value of grasp position: \n" << grasp_pos << std::endl;
    //std::cout << "The value of centroid: \n" << ob_centroids << std::endl;
    for (int it = 0; it < clusternumber; ++it)
    {

        distance_x = abs(grasp_pos(0) - ob_centroids(it,0));
        //std::cout <<  "The distance_x is: " << distance_x << std::endl;

        distance_y = abs(grasp_pos(1) - ob_centroids(it,1));
        //std::cout <<  "The distance_y is: " << distance_y << std::endl;

        distance = pow(distance_x, 2) + pow(distance_y, 2);
        //std::cout <<  "The distance is: " << distance << std::endl;

        if (distance < pow(threshold,2))
        {
            ob_grasp = it;
            std::cout <<  "The object to be grasped is: " << it+1 << std::endl;
            break;
        }
    }
    return ob_grasp;
}


int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud2; //(new pcl::PointCloud<pcl::PointXYZ>);
    std::string filename, filenamejpg, filenamepcd, root; 
    filename= "depthOnly0502";
    //filename= "4object2";
    root = "/home/bui/bui_ws/image_sample/";
    //root = "/home/bui/bui_ws/image_sample/depthOnly0315/";
    filenamepcd = root +filename+".pcd";

    cloud = pcd_read(filenamepcd);
    show_pcd(cloud.makeShared(), 0, "Original point cloud data");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudp1 (new pcl::PointCloud<pcl::PointXYZRGB>), cloudp4 (new pcl::PointCloud<pcl::PointXYZRGB>),
              cloudp2 (new pcl::PointCloud<pcl::PointXYZRGB>), cloudp3 (new pcl::PointCloud<pcl::PointXYZRGB>); 
    //cloud1 = passthrough_filter(cloud.makeShared());

    // show_pcd only work with pointer->use cloud.makeShared() to return a pointer
    // downsampling the data - cloudp1 is a pointer
    cloudp1 = downsampling(cloud.makeShared());
    cloudp4 = passthrough_filter(cloudp1);


    // cloudp2 is a pointer -  planar filter
    cloudp2 = planar_filter(cloudp4);
    show_pcd(cloudp2, 0, "after filtering");

    std::vector<pcl::PointIndices> cluster_indices;
    cluster_indices = extract_object(cloudp2);

    Eigen::Matrix3f camera_matrix;
    // calibration matrix self-calibration
    //camera_matrix << 538.2075, 0.0, 318.3089, 0.0, 538.6995, 231.273, 0.0, 0.0, 1.0;
    // calibration matrix from wed
    //camera_matrix << 570.3422, 0.0, 319.5, 0.0, 570.3422, 239.5, 0.0, 0.0, 1.0;
    // new calibration file 05-02-2019
    camera_matrix << 550.1966, 0.0, 310.8416, 0.0, 551.3066, 243.986948, 0.0, 0.0, 1.0;
    
    // Count number of cluster
    int clusterNumber = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        ++clusterNumber;
    

    Eigen::MatrixXf centroid3D(clusterNumber, 4);
    Eigen::Vector4f oneCentroid3D;
    Eigen::MatrixXi centroid2D(clusterNumber, 2);//(2,2);
    Eigen::Vector2i onecentroid2D;
    std::cout << "Everything untill here is still ok " << clusterNumber << std::endl;

    // calcualte the centroid of the cluster -> just for one object
    if (clusterNumber = 1)
    {
        pcl::compute3DCentroid(*cloudp2, oneCentroid3D);
        centroid3D.row(0) = oneCentroid3D;
    }
    else
        centroid3D = object_centroid(cloudp2, cluster_indices, clusterNumber);
    
    std::cout << "The centroid 3D of the objects \n";
    std::cout << centroid3D << std::endl;
  
    int count = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {    
        //std::cout << "Try to print here to check whether something wrong" << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        // copy all the point cloud of one cluster to a new point cloud data
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            // copy each point cloud of the cluster to a new point cloud data
            cloud_cluster->points.push_back(cloudp2->points[*pit]);
        

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        // convert the centroid from 3D point cloud to pixel value
        onecentroid2D = point3Dto_pixel(centroid3D.row(count), camera_matrix);

        //centroid2D.conservativeResize(centroid2D.rows()+1, 2);
        //centroid2D.row(centroid2D.rows()-1) = onecentroid2D;
        centroid2D.row(count) = onecentroid2D;

        //std::cout << "Try to print here to check whether something wrong 1111111" << std::endl;
        //std::cout << centroid2D.row(count) << std::endl;
        count++;
        show_pcd(cloud_cluster, count, "Displaying individual point cloud cluster");
        std::cout << "The 2D centroid of point cloud cluster " << count << std::endl;
        std::cout << centroid2D.row(count-1) << std::endl;
    }

    cloudp3 = integrate_cluster(cloudp2, cluster_indices);
    pcd_write(filename, cloudp3);
    show_centroid(cloudp3, centroid3D); 

    // filenamejpg = root +filename+".jpg";
    // drawboxrgb(centroid2D, filenamejpg);
    
    //std::cout << "Just check is everything still ok here?" << std::endl;
    return (0);
}