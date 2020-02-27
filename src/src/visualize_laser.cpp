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

// Size of the circular buffer
#define SIZE_X 50
#define SIZE_Y 50
// The resolution (how many steps in 1 meter)
// Area of one cell = (1 / RESOLUTION)^2 m^2
#define RESOLUTION 15

ros::Subscriber laser_subscriber;
ros::Subscriber odom_subscriber;
ros::Publisher vis_pub;
tf::TransformListener *listener;

// DynamicVoronoi dynVo;

// Circular buffer
bool occupied[SIZE_X][SIZE_Y] = { 0 };

// Offset position of the curcular buffer
int offsX = 0, offsY = 0;

int EUCMOD(int a, int b) {
  return ((a % b) + b) % b;
}

// Convert a coordinate value into an index in the buffer.
int toIndex(double x, int max) {
  return EUCMOD((int)(x * RESOLUTION + max/2.0 + 0.5), max);
}

double toCoord(int index, int max) {
  return (double)index / RESOLUTION - max/2.0;
}

// Mark a cell at 
void occupyCell(double x, double y) {
  ROS_INFO("Occupy %d, %d", toIndex(x, SIZE_X), toIndex(y, SIZE_Y));
  occupied[toIndex(x, SIZE_X)][toIndex(y, SIZE_Y)] = true;
}

geometry_msgs::Point transformPoint(const tf::TransformListener& listener, geometry_msgs::Point point,
    const char* from, const char* to) {
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = from;

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  laser_point.point.x = point.x;
  laser_point.point.y = point.y;
  laser_point.point.z = point.z;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint(to, laser_point, base_point);
    return base_point.point;
    
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", from, to, ex.what());
  }
  return geometry_msgs::Point();
}

void publishLaserMarker(sensor_msgs::LaserScanConstPtr msg) {
        visualization_msgs::MarkerArray vis_msg;
        // create behavior
        msg->angle_min; // starting angle
        msg->angle_max; // ending angle
        msg->angle_increment; // angular resolution
        msg->ranges; // sensor data list

        vis_msg.markers.push_back(visualization_msgs::Marker());
        visualization_msgs::Marker& marker = vis_msg.markers.back();
        marker.type = visualization_msgs::Marker::POINTS;
        marker.header.frame_id = msg->header.frame_id;
        marker.color.a = 1.0;
        marker.color.r = 0.2;
        marker.color.g = 1.0;
        marker.color.b = 0.2;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "xy laser";
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        std::vector<geometry_msgs::Point> points;          
        
        for (int index = 0; index < msg->ranges.size(); index++)
        {
                double range = msg->ranges[index];
                if (range < msg->range_max && range > msg->range_min)
                {
                        
                        double angle = msg->angle_min + msg->angle_increment * index;
                        double y = cos(angle) * range;
                        double x = sin(angle) * range;
                        
                        geometry_msgs::Point p;
                        p.x = x;
                        p.y = y;
                        
                        geometry_msgs::Point absP = transformPoint(*listener, p, "base_link", "odom");
                        
                        occupyCell(absP.x, absP.y);
                        
                        //points.push_back(absP);
                        
                }
        }
        
        
        for (int x = 0; x < SIZE_X; x++) {
          for (int y = 0; y < SIZE_Y; y++) {
            if (occupied[x][y]) {
              geometry_msgs::Point p;
              p.x = toCoord(x, SIZE_X);
              p.y = toCoord(y, SIZE_Y);
              geometry_msgs::Point absP = transformPoint(*listener, p, "odom", "base_link");
              points.push_back(p);
            }
          }
        }
    
        
        for (int i = 1; i < points.size(); i++) {
          marker.points.push_back(points.at(i));
        }
        
        vis_pub.publish(vis_msg);
}

void laser_callback(sensor_msgs::LaserScanConstPtr msg) {
  publishLaserMarker(msg);
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

  ros::init(argc, argv, "visualize_laser");

  ros::NodeHandle node_handle;

  vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
  laser_subscriber = node_handle.subscribe("/camera/scan", 1, laser_callback);
  odom_subscriber = node_handle.subscribe("odom", 1000, odometry_callback);
  
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

