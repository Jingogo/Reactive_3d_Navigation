#ifndef MAP_HELPER_H
#define MAP_HELPER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <sstream>

#include "reactive_3d_navigation/OccupancyMap.h"
#include "reactive_3d_navigation/tf_helper.h"

void updateMap(sensor_msgs::LaserScanConstPtr msg, OccupancyMap* occupancyMap, tf::TransformListener* listener) {
  geometry_msgs::Point zero;
  zero.x = 0.0;
  zero.y = 0.0;
  
  geometry_msgs::Point robotPos = transformPoint(*listener, zero, "base_link", "odom", msg->header.stamp);
  
  //ROS_INFO("Robot offset %f, %f", -robotPos.x, -robotPos.y);
  
  occupancyMap->updatePos(robotPos.x, robotPos.y);
  
  // create behavior
  msg->angle_min; // starting angle
  msg->angle_max; // ending angle
  msg->angle_increment; // angular resolution
  msg->ranges; // sensor data list
  
  std::vector<geometry_msgs::Point> points;
  
  for (int index = 0; index < msg->ranges.size(); index++)
  {
    double range = msg->ranges[index];
    if (range > msg->range_min)
    {
      double angle = msg->angle_min + msg->angle_increment * index
          - transformAngle(*listener, "odom", "base_link", msg->header.stamp);
      geometry_msgs::Point point;
      point.y = sin(angle) * range;
      point.x = cos(angle) * range;
      
      if (range < msg->range_max) {
        points.push_back(point);
      } else {
        // Even if it's max range, we still should free all position in that
        // direction.
        occupancyMap->freeInBetween(point.x, point.y);
      }
      
    }
  }
  
  // First free all position, then set obstacles.
  // This avoids that we free a position that was seen as an obstacle by another measurement.
  for (int i = 0; i < points.size(); i++) {
    occupancyMap->freeInBetween(points[i].x, points[i].y);
  }
  
  for (int i = 0; i < points.size(); i++) {
    occupancyMap->occupy(points[i].x, points[i].y);
  }
  
  
}

void visualizeMap(sensor_msgs::LaserScanConstPtr msg, const OccupancyMap& occupancyMap,
    tf::TransformListener* listener, ros::Publisher* vis_pub) {
  visualization_msgs::MarkerArray vis_msg;
  
  // create behavior
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

  const int RADIUS = occupancyMap.getRadius();

  for (int idx = -RADIUS; idx <= RADIUS; idx++) {
    for (int idy = -RADIUS; idy <= RADIUS; idy++) { 
      if (occupancyMap.isOccupiedIndex(idx, idy)) {
        //ROS_INFO("Occupied: %d, %d", idx, idy);  
        geometry_msgs::Point p;
        p.x = occupancyMap.indexToCoord(idx);
        p.y = occupancyMap.indexToCoord(idy);
        
        geometry_msgs::Point p_rot;
        double angle = -1 * transformAngle(*listener, "base_link", "odom", msg->header.stamp);
        p_rot.x = cos(angle) * p.x - sin(angle) * p.y;
        p_rot.y = cos(angle) * p.y + sin(angle) * p.x;
        points.push_back(p_rot);
      }
    }
  }
  
  for (int i = 0; i < points.size(); i++) {
    marker.points.push_back(points.at(i));
  }
  
  vis_pub->publish(vis_msg);
}

#endif // MAP_HELPER_H
