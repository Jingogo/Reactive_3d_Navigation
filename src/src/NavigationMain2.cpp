#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "dynamicvoronoi/dynamicvoronoi.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <algorithm>
//#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <limits>
#include <angles/angles.h>

#include <sstream>

#include "reactive_3d_navigation/distance_map.h"
#include "reactive_3d_navigation/OccupancyMap.h"
#include "reactive_3d_navigation/map_helper.h"
#include "reactive_3d_navigation/distancemap_helper.h"
#include "reactive_3d_navigation/rasterization_helper.h"

// Size of the circular buffer
#define SIZE_MAP_RADIUS 40
// The resolution (how many steps in 1 meter)
// Area of one cell = (1 / RESOLUTION)^2 m^2
#define MAP_RESOLUTION 15

#define M_PI 3.14159265358979323846

ros::Subscriber laser_subscriber;
ros::Subscriber goal_subscriber;
ros::Publisher vis_pub;
ros::Publisher cmd_vel_pub;
tf::TransformListener *listener;

//bool DISPLAY_MAP=true;
//bool DISPLAY_OCCUPY=true;

Distance_Map voronoi;
//DynamicVoronoi voronoi;
bool ** grid_map=NULL;
int iter_count = 0;

OccupancyMap occupancyMap(SIZE_MAP_RADIUS, MAP_RESOLUTION);

struct Path {
  geometry_msgs::Point end_p;
  double start_orientation;
  double radius;
  double score;
  double minDistToObst;
  int steps;
};

bool hasGoal = false;
geometry_msgs::Point longTermGoal;
Path bestPath;


void rotate(double speed, ros::Publisher* cmd_vel_pub) {
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = 0;
    base_cmd.angular.z = speed;
    cmd_vel_pub->publish(base_cmd);
}

void driveForward(double speed, ros::Publisher* cmd_vel_pub) {
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = speed;
    base_cmd.angular.z = 0; 
    cmd_vel_pub->publish(base_cmd);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double dist(geometry_msgs::Point point1, geometry_msgs::Point point2) {
  return std::sqrt((point1.x - point2.x) * (point1.x - point2.x)
                 + (point1.y - point2.y) * (point1.y - point2.y));
}

template <typename T> geometry_msgs::Point rotatePoint(T x, T y, double angle) {
  geometry_msgs::Point p_rot;
  p_rot.x = cos(angle) * x - sin(angle) * y;
  p_rot.y = cos(angle) * y + sin(angle) * x;
  return p_rot;
}

double angleDiff(double firstAngle, double secondAngle) {
  double difference = secondAngle - firstAngle;
  while (difference < -M_PI) difference += 2 * M_PI;
  while (difference > M_PI) difference -= 2 * M_PI;
  return difference;
}

double getAngleToYAxis(double x1, double y1, double x2, double y2) {
  double w = x2 - x1;
  double h = y2 - y1;

  double angle = atan(h/w);
  if (w < 0 || h < 0)
      angle += M_PI;
  if (w > 0 && h < 0)
      angle -= M_PI;
  if (angle < 0)
      angle += 2 * M_PI;

  return angle;
}

Path getDirectPath(int endX, int endY, float minDist, double angleOdom, visualization_msgs::Marker* marker) {
  float minDistToObst = std::numeric_limits<float>::infinity();
  
  std::vector<intPoint> points = rasterLine(SIZE_MAP_RADIUS, SIZE_MAP_RADIUS, endX, endY);

  // Check all points on that line
  int i = 0;
  for (; i < points.size(); i++) {
    
    float distance = voronoi.getDistance(points[i].x, points[i].y) / MAP_RESOLUTION;
    
    if (distance < minDist) break;
    minDistToObst = std::min(distance, minDistToObst);
    
    if (marker != NULL) {
      geometry_msgs::Point p;
      p = rotatePoint((double)(points[i].x - SIZE_MAP_RADIUS) / MAP_RESOLUTION,
                      (double)(points[i].y - SIZE_MAP_RADIUS) / MAP_RESOLUTION, angleOdom);
      marker->points.push_back(p);
    }
  }

  // Create endpoint
  // Rotate point based on the angle between odom and base_link, because the distance map alsways
  // uses the same orientation.
  geometry_msgs::Point end_p = rotatePoint((double)(points[i-1].x - SIZE_MAP_RADIUS) / MAP_RESOLUTION,
                                           (double)(points[i-1].y - SIZE_MAP_RADIUS) / MAP_RESOLUTION, angleOdom);
  
  Path path;
  path.end_p = end_p;
  path.minDistToObst = minDistToObst;
  path.steps = i;
  
  return path;
}

std::pair<Path, Path> getCurvedPath(double radiusInMeter, double angle, float minDist,
    double angleOdom, visualization_msgs::Marker* marker) {
  float minDistToObst = std::numeric_limits<float>::infinity();
  
  int radiusInCells = (int)(radiusInMeter * MAP_RESOLUTION + 0.5);
  
  int m_x = SIZE_MAP_RADIUS;
  int m_y = SIZE_MAP_RADIUS - radiusInCells;
  
  std::vector<intPoint> points = rasterCircle(m_x, m_y, radiusInCells);
  
  int i = 0;
  for (; i < points.size(); i++) {
    geometry_msgs::Point p_rot = rotatePoint(points[i].x - SIZE_MAP_RADIUS,
                                             points[i].y - SIZE_MAP_RADIUS, angle);
    points[i] = intPoint((int)(p_rot.x + SIZE_MAP_RADIUS + 0.5), (int)(p_rot.y + SIZE_MAP_RADIUS + 0.5));
  }

  // Check all points on that line (clockwise)
  for (i = 0; i < points.size(); i++) {
    float distance = voronoi.getDistance(points[i].x, points[i].y) / MAP_RESOLUTION;
    if (distance < minDist) break;
    minDistToObst = std::min(distance, minDistToObst);
    if (marker != NULL) {
      geometry_msgs::Point p;
      p = rotatePoint((double)(points[i].x - SIZE_MAP_RADIUS) / MAP_RESOLUTION,
                      (double)(points[i].y - SIZE_MAP_RADIUS) / MAP_RESOLUTION, angleOdom);
      marker->points.push_back(p);
    }
  }
  
  // Create endpoint1
  // Rotate point based on the angle between odom and base_link, because the distance map alsways
  // uses the same orientation.
  geometry_msgs::Point end_p1 = rotatePoint((double)(points[i-1].x - SIZE_MAP_RADIUS) / MAP_RESOLUTION,
                                            (double)(points[i-1].y - SIZE_MAP_RADIUS) / MAP_RESOLUTION, angleOdom);
  
  Path path1;
  path1.end_p = end_p1;
  path1.minDistToObst = minDistToObst;
  path1.steps = i;
  path1.start_orientation = angle;
  
  
  // Check all points on that line (counter-clockwise)
  // We mirror all circle segments to get counter-clockwise orientation.
  minDistToObst = std::numeric_limits<float>::infinity();
  for (i = 0; i < points.size(); i++) {
    float distance = voronoi.getDistance(-points[i].x + 2 * SIZE_MAP_RADIUS, points[i].y) / MAP_RESOLUTION;
    if (distance < minDist) break;
    minDistToObst = std::min(distance, minDistToObst);
    if (marker != NULL) {
      geometry_msgs::Point p;
      p = rotatePoint((double)(-points[i].x + SIZE_MAP_RADIUS) / MAP_RESOLUTION,
                      (double)( points[i].y - SIZE_MAP_RADIUS) / MAP_RESOLUTION, angleOdom);
      marker->points.push_back(p);
    }
  }

  // Create endpoint2
  geometry_msgs::Point end_p2 = rotatePoint((double)(-points[i-1].x + SIZE_MAP_RADIUS) / MAP_RESOLUTION,
                                            (double)( points[i-1].y - SIZE_MAP_RADIUS) / MAP_RESOLUTION, angleOdom);
  
  Path path2;
  path2.end_p = end_p2;
  path2.minDistToObst = minDistToObst;
  path2.steps = i;
  path2.start_orientation = -angle + M_PI;  // Adjust angle according to mirror-transformation.
  
  return std::make_pair(path1, path2);
}


void findNewBestPath(sensor_msgs::LaserScanConstPtr msg, ros::Publisher* vis_pub) {

  const double maxPathLength = 2;
  const double radius = 1.5;
  const double minDistToObst = 0.35;
  const int minSteps = 3;

  visualization_msgs::MarkerArray vis_msg;
  
  // create markers (paths)
  vis_msg.markers.push_back(visualization_msgs::Marker());
  visualization_msgs::Marker& markersPaths = vis_msg.markers.back();
  markersPaths.type = visualization_msgs::Marker::POINTS;
  markersPaths.header.frame_id = msg->header.frame_id;
  markersPaths.color.a = 1.0; markersPaths.color.r = 0.2; markersPaths.color.g = 0.2; markersPaths.color.b = 1.0;
  markersPaths.action = visualization_msgs::Marker::ADD;
  markersPaths.ns = "paths";
  markersPaths.pose.orientation.w = 1;
  markersPaths.scale.x = 0.05; markersPaths.scale.y = 0.05; markersPaths.scale.z = 0.05;
  
  // create markers (goal)
  vis_msg.markers.push_back(visualization_msgs::Marker());
  visualization_msgs::Marker& markersGoal = vis_msg.markers.back();
  markersGoal.type = visualization_msgs::Marker::POINTS;
  markersGoal.header.frame_id = msg->header.frame_id;
  markersGoal.color.a = 1.0; markersGoal.color.r = 1.0; markersGoal.color.g = 0.2; markersGoal.color.b = 0.2;
  markersGoal.action = visualization_msgs::Marker::ADD;
  markersGoal.ns = "goal";
  markersGoal.pose.orientation.w = 1;
  markersGoal.scale.x = 0.05; markersGoal.scale.y = 0.05; markersGoal.scale.z = 0.05;
  
  // create markers (bestPath)
  vis_msg.markers.push_back(visualization_msgs::Marker());
  visualization_msgs::Marker& markersBestPath = vis_msg.markers.back();
  markersBestPath.type = visualization_msgs::Marker::POINTS;
  markersBestPath.header.frame_id = msg->header.frame_id;
  markersBestPath.color.a = 1.0; markersBestPath.color.r = 0.2; markersBestPath.color.g = 1.0; markersBestPath.color.b = 1.0;
  markersBestPath.action = visualization_msgs::Marker::ADD;
  markersBestPath.ns = "bestPath";
  markersBestPath.pose.orientation.w = 1;
  markersBestPath.scale.x = 0.05; markersBestPath.scale.y = 0.05; markersBestPath.scale.z = 0.05;

  std::vector<Path> paths;

  double angleOdom = transformAngle(*listener, "odom", "base_link", msg->header.stamp);
  
  geometry_msgs::Point goal_p = transformPoint(*listener, longTermGoal, "odom", "base_link");

  // Check every direction
  for (double angle = 0; angle < 2 * M_PI; angle += M_PI/24) {
    double y = sin(angle) * maxPathLength;
    double x = cos(angle) * maxPathLength;
    Path pathDirect1 = getDirectPath((int)(x * MAP_RESOLUTION + 0.5) + SIZE_MAP_RADIUS,
                                     (int)(y * MAP_RESOLUTION + 0.5) + SIZE_MAP_RADIUS, minDistToObst, angleOdom, &markersPaths);
    // Angle of that direction
    pathDirect1.start_orientation = angle;
    paths.push_back(pathDirect1);

    auto pathPairCurved = getCurvedPath(radius, angle, minDistToObst, angleOdom, &markersPaths);
    paths.push_back(pathPairCurved.first);
    paths.push_back(pathPairCurved.second);
  }


  Path goalPath = getDirectPath((int)(goal_p.x * MAP_RESOLUTION + 0.5) + SIZE_MAP_RADIUS,
                                (int)(goal_p.y * MAP_RESOLUTION + 0.5) + SIZE_MAP_RADIUS, minDistToObst, angleOdom, &markersPaths);


  goalPath.start_orientation = -atan2(goal_p.x, goal_p.y) + M_PI / 2 - angleOdom;
  paths.push_back(goalPath);

  bestPath.score = std::numeric_limits<double>::infinity();
  bool foundAnyPath = false;
  
  markersGoal.points.push_back(goal_p);

  // Find direction with lowest score, i.e. minimize distance to goal.
  for (int i = 0; i < paths.size(); i++) {
    paths[i].score = dist(goal_p, paths[i].end_p) - 0.1 * paths[i].minDistToObst;
    
    if (paths[i].score < bestPath.score && paths[i].steps > minSteps) {
      bestPath = paths[i];
      foundAnyPath = true;
    }
  }
  
  if (!foundAnyPath) {
    ROS_INFO("No feasable path found.");
    hasGoal = false;
  } else {
    markersBestPath.points.push_back(bestPath.end_p);
  }
  
  vis_pub->publish(vis_msg);
}

void laser_callback(sensor_msgs::LaserScanConstPtr msg) {
  updateMap(msg, &occupancyMap, listener);
  
  if (iter_count % 2 == 0) {
    update_distance_map(grid_map, occupancyMap, &voronoi);
    if (hasGoal) findNewBestPath(msg, &vis_pub);
  }
  
  
  double angle = transformAngle(*listener, "base_link", "odom");
  
  double diffAngle = angleDiff(angle, bestPath.start_orientation);
  geometry_msgs::Point zero;
  zero.x = 0.0;
  zero.y = 0.0;
  
  if (hasGoal) {
    //ROS_INFO("Angle goal: %f", bestPath.start_orientation);
    //ROS_INFO("My angle: %f", angle);
    //ROS_INFO("Angle diff: %f", diffAngle);
    
    geometry_msgs::Point goal_p = transformPoint(*listener, longTermGoal, "odom", "base_link");
    //ROS_INFO("Goal: %f, %f", goal_p.x, goal_p.y);


    if (std::abs(diffAngle) > 1) {
      rotate(sgn(diffAngle) * 2, &cmd_vel_pub);
    } else if (std::abs(diffAngle) > 0.2) {
      rotate(sgn(diffAngle) * 1, &cmd_vel_pub);
    } else if (dist(goal_p, zero) > 0.4) {
      double speed = bestPath.minDistToObst > 1 ? 0.5 : 0.25;
      driveForward(speed, &cmd_vel_pub);
    } else {
      hasGoal = false;
      ROS_INFO("Goal reached.");
    }
  }

  if (iter_count % 10 == 0) {
    visualizeMap(msg, occupancyMap, listener, &vis_pub);
  }

  if (iter_count % 20 == 0) {
    // Too expensive
    // visualize_distance_map(msg, occupancyMap, voronoi, *listener, &vis_pub);  
  }
  
  iter_count++;
}

void goal_callback(geometry_msgs::PoseStamped goal){
  ROS_INFO("recieve one goal");
  longTermGoal = transformPoint(*listener, goal.pose.position, "base_link", "odom");
  hasGoal = true;
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "get_distance_map");

  ros::NodeHandle node_handle;
  
  cmd_vel_pub = node_handle.advertise<geometry_msgs::Twist>( "/mobile_base/commands/velocity", 1 );  
  vis_pub = node_handle.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
  goal_subscriber = node_handle.subscribe("/move_base_simple/goal", 1, goal_callback);
  laser_subscriber = node_handle.subscribe("/camera/scan", 1, laser_callback);
  
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
