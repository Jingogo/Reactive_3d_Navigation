#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "dynamicvoronoi/dynamicvoronoi.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <sstream>

#include "reactive_3d_navigation/OccupancyMap.h"
#include "reactive_3d_navigation/map_helper.h"

// Size of the circular buffer
#define SIZE_MAP_RADIUS 50
// The resolution (how many steps in 1 meter)
// Area of one cell = (1 / RESOLUTION)^2 m^2
#define RESOLUTION 15

ros::Subscriber laser_subscriber;
ros::Subscriber goal_subscriber;
ros::Publisher vis_pub;
ros::Publisher vis_goal_pub;
tf::TransformListener *listener;

OccupancyMap occupancyMap(SIZE_MAP_RADIUS, RESOLUTION);

void visualize_goal(geometry_msgs::PoseStamped goal)
{
   
  visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = goal.header.frame_id;
  goal_marker.ns = "goal position";
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.pose.orientation.x = 0.0;
  goal_marker.pose.orientation.y = 0.0;
  goal_marker.pose.orientation.z = 0.0;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.scale.x = 0.1;
  goal_marker.scale.y = 0.1;
  goal_marker.scale.z = 0.1;
  goal_marker.color.a = 1.0; // Don't forget to set the alpha!
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 0.0;
  
  geometry_msgs::Point p;
  p.x=goal.pose.position.x;
  p.y=goal.pose.position.y;
  goal_marker.points.push_back(p);
  vis_goal_pub.publish(goal_marker); 
  ROS_INFO("visualize one goal");
  
}

void laser_callback(sensor_msgs::LaserScanConstPtr msg) {
  updateMap(msg, &occupancyMap, listener);
  visualizeMap(msg, occupancyMap, listener, &vis_pub);
}

void goal_callback(geometry_msgs::PoseStamped goal){
  visualize_goal(goal);
  ROS_INFO("subscribe one goal");
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "visualize_laser");

  ros::NodeHandle node_handle;

  vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
  vis_goal_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_goal", 0 );
  laser_subscriber = node_handle.subscribe("/camera/scan", 1, laser_callback);
  goal_subscriber = node_handle.subscribe("/move_base_simple/goal", 1, goal_callback);
  listener = new tf::TransformListener(ros::Duration(20));

  ros::Rate loop_rate(20);
  
  /*  int count = 0;
  while (ros::ok())
  {    
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
  }*/

  ros::spin();


  return 0;
}
