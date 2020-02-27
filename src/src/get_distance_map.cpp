#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "dynamicvoronoi/dynamicvoronoi.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include "reactive_3d_navigation/distance_map.h"
#include <algorithm>

#include <sstream>

#include "reactive_3d_navigation/OccupancyMap.h"
#include "reactive_3d_navigation/map_helper.h"
#include "reactive_3d_navigation/distancemap_helper.h"

// Size of the circular buffer
#define SIZE_MAP_RADIUS 50
// The resolution (how many steps in 1 meter)
// Area of one cell = (1 / RESOLUTION)^2 m^2
#define RESOLUTION 15

ros::Subscriber laser_subscriber;
//ros::Subscriber odom_subscriber;
ros::Publisher vis_pub;
ros::Publisher cmd_vel_pub;
tf::TransformListener *listener;

//bool DISPLAY_MAP=true;
//bool DISPLAY_OCCUPY=true;

Distance_Map voronoi;
//DynamicVoronoi voronoi;
bool ** grid_map=NULL;
int iter_count = 0;

OccupancyMap occupancyMap(SIZE_MAP_RADIUS, RESOLUTION);

void laser_callback(sensor_msgs::LaserScanConstPtr msg) {
  //updateMap(msg);
  updateMap(msg, &occupancyMap, listener);
  
  update_distance_map(grid_map, occupancyMap, &voronoi);
  
  if ( iter_count % 20 == 0){
    visualize_distance_map(msg, occupancyMap, voronoi, *listener, &vis_pub);
    visualizeMap(msg, occupancyMap, listener, &vis_pub);
    iter_count = 0;
  }
  iter_count++;
}


/**
* Callback function executes when new topic data comes.
* Task of the callback function is to print data to screen.
*/
void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  //ROS_INFO("Seq: [%d]", msg->header.seq);
  //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]",
  //  msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]",
  //  msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  
  //offsX = toCoord(msg->pose.pose.position.x);
  //offsY = toCoord(msg->pose.pose.position.y);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "get_distance_map");

  ros::NodeHandle node_handle;
  
  cmd_vel_pub = node_handle.advertise<geometry_msgs::Twist>( "/mobile_base/commands/velocity", 1 );  
  vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
  laser_subscriber = node_handle.subscribe("/camera/scan", 1, laser_callback);
  //odom_subscriber = node_handle.subscribe("odom", 1000, odometry_callback);
  
  listener = new tf::TransformListener(ros::Duration(10));

  ros::Rate loop_rate(10);

  int count = 0;
  /*while (ros::ok())
  {    
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
  }*/

  ros::spin();


  return 0;
}

