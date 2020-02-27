#ifndef DISTANCEMAP_HELPER_H
#define DISTANCEMAP_HELPER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
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
#include "reactive_3d_navigation/tf_helper.h"


void update_distance_map(bool **grid_map, const OccupancyMap& occupancyMap, Distance_Map* voronoi)
{
    const int RADIUS = occupancyMap.getRadius();
    
    const int map_size=2*RADIUS+1;
    
    if (!grid_map)
    {
      grid_map = new bool*[map_size];
      for (int x=0; x<map_size; x++) grid_map[x] = new bool[map_size];

      for (int idx = -RADIUS; idx <= RADIUS; idx++) {
      for (int idy = -RADIUS; idy <= RADIUS; idy++) { 
          int x= idx + RADIUS;
          int y= idy + RADIUS;
          grid_map[x][y]=occupancyMap.isOccupiedIndex(idx, idy);
        }
      }
      voronoi->initializeMap(map_size, map_size, grid_map);
      voronoi->update(); // update distance map and Voronoi diagram
      //if (doPrune) voronoi.prune();  // prune the Voronoi
      //voronoi->visualize("initial.ppm");
      //std::cerr << "Generated initial frame.\n";
    }
    
    else
    {
      for (int idx = -RADIUS; idx <= RADIUS; idx++) {
      for (int idy = -RADIUS; idy <= RADIUS; idy++) { 
          int x= idx + RADIUS;
          int y= idy + RADIUS;
          if (grid_map[x][y] != occupancyMap.isOccupiedIndex(idx, idy)){
            if (!grid_map[x][y]){
              //ROS_INFO("new Occupy %d, %d", x, y);
              voronoi->occupyCell(x,y);
            }
            else{
              //ROS_INFO("remove  %d, %d", x, y);
              voronoi->clearCell(x,y);         
            }
            grid_map[x][y]=occupancyMap.isOccupiedIndex(idx, idy);
          }
        }
      }
      
      /*for (const auto& pair : occupancyMap.popAddedPositions()) {
        const int x = pair.first, y = pair.second;
        if (!grid_map[x][y]){
          //ROS_INFO("new Occupy %d, %d", x, y);
          voronoi->occupyCell(x,y);
        }
      }
      
      for (const auto& pair : occupancyMap.popDeletedPositions()) {
        const int x = pair.first, y = pair.second;
        if (grid_map[x][y]){
          //ROS_INFO("new Occupy %d, %d", x, y);
          voronoi->clearCell(x,y);
        }
      }*/
      
      voronoi->update();  
      ROS_INFO("update distance map,done");
      
    }
    
}


void visualize_distance_map(sensor_msgs::LaserScanConstPtr msg, const OccupancyMap& occupancyMap,
    Distance_Map& voronoi, const tf::TransformListener& listener, ros::Publisher* vis_pub)
{
    const int RADIUS = occupancyMap.getRadius();
    
    const int map_size=2*RADIUS+1;
    
    visualization_msgs::MarkerArray vis_msg;
    
    // create behavior
    vis_msg.markers.push_back(visualization_msgs::Marker());
    visualization_msgs::Marker& marker = vis_msg.markers.back();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.header.frame_id = msg->header.frame_id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "distance map";
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    std::vector<geometry_msgs::Point> points; 
    std::vector<std_msgs::ColorRGBA> colors;
      
    for (int idx = -RADIUS; idx <= RADIUS; idx++) {
      for (int idy = -RADIUS; idy <= RADIUS; idy++) { 
          int x= idx+RADIUS;
          int y= idy+RADIUS;
          //ROS_INFO("position: (%d, %d), distance: %f", x, y,voronoi.getDistance(x,y));
           
          geometry_msgs::Point p;
          p.x = occupancyMap.indexToCoord(idx);
          p.y = occupancyMap.indexToCoord(idy);
          
          geometry_msgs::Point p_rot;
          double angle = -transformAngle(listener, "base_link", "odom", msg->header.stamp);
          p_rot.x = cos(angle) * p.x - sin(angle) * p.y;
          p_rot.y = cos(angle) * p.y + sin(angle) * p.x;
          
          //ROS_INFO("Visualize: %f, %f", p.x, p.y);
          points.push_back(p_rot);
          
          std_msgs::ColorRGBA c;

          if (occupancyMap.isOccupiedIndex(idx, idy)) {
            //ROS_INFO("new Occupied: %d, %d", x, y); 
            //ROS_INFO("Visualize: %f, %f", p.x, p.y);
            c.r = 0;
            c.g = 0;
            c.b = 0;
            c.a = 1;
          }
          else if (voronoi.getDistance(x,y)==INFINITY|voronoi.getDistance(x,y)==-INFINITY){
            c.r = 1;
            c.g = 1;
            c.b = 1;
            c.a = 1; 
          }
          else{
            double max_dis=sqrt(2*map_size*map_size)/5;
            c.r = 0.1;
            c.g = std::min(1.0,voronoi.getDistance(x,y)/max_dis);
            c.b = 0.1;
            c.a = 1; 
          }
                  
        colors.push_back(c);
      }
    }
    for (int i = 0; i < points.size(); i++) {
      marker.points.push_back(points.at(i));
      marker.colors.push_back(colors.at(i));
    }
    
    vis_pub->publish(vis_msg);

}

#endif  // DISTANCEMAP_HELPER_H
