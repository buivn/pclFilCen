#include "ros/ros.h"
#include "pahap/pointcloud_cmd.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_pahap_client");



  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pahap::pointcloud_cmd>("pointcloud_cmd");
  pahap::pointcloud_cmd srv;
  ros::Rate loop_rate(0.2);
  // srv.request.a = atoll(argv[1]);
  // srv.request.b = atoll(argv[2]);
  srv.request.cmd = true;
  srv.request.path = "/home/buivn/bui_ws/src/pahap/pointcloud/";
  srv.request.topic = false;


  int i = 0;
  while(ros::ok())
  {
    if (i<1)
    {
      srv.request.num_name = std::to_string(i);
      client.call(srv);
      if (srv.response.result == 0)
      {
        ROS_INFO("Capture pointcloud frame: %d", i);
        // client.call(srv); 
      }
      else
      {
        ROS_ERROR("Failed to call service pointcloud_cmd");

        return 1;
      }
    }
    i += 1;
    ros::spinOnce();
    loop_rate.sleep();
  } 


  return 0;
}