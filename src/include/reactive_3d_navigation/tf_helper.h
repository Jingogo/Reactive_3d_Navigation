#ifndef TF_HELPER_H
#define TF_HELPER_H

#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

bool getTransform(const tf::TransformListener& listener, const char* from, const char* to,
    tf::StampedTransform& transform, ros::Time time = ros::Time()) {
  try {    
    ros::Time now = ros::Time();  // ros::Time::now()
    listener.waitForTransform(to, from, now, ros::Duration(3.0));
    listener.lookupTransform(to, from, now, transform);
    return true;
  } catch(tf::TransformException& ex) {
    ROS_ERROR("Received an exception trying get a transform from \"%s\" to \"%s\": %s", from, to, ex.what());
  }
  return false;
}

geometry_msgs::Point transformPoint(const tf::TransformListener& listener,
    geometry_msgs::Point point, const char* from, const char* to, ros::Time time = ros::Time()) {
  tf::Point p;
  tf::pointMsgToTF(point, p);
  tf::StampedTransform transform;
  if (getTransform(listener, from, to, transform, time)) {
    p = transform * p;
    geometry_msgs::Point result;
    tf::pointTFToMsg(p, result);
    return result;
  }
  // Fail
  return geometry_msgs::Point();
}

geometry_msgs::Quaternion transformQuaternion(const tf::TransformListener& listener,
    geometry_msgs::Quaternion quaternion, const char* from, const char* to, ros::Time time = ros::Time()) {
  tf::Quaternion q;
  tf::quaternionMsgToTF(quaternion, q);
  tf::StampedTransform transform;
  if (getTransform(listener, from, to, transform, time)) {
    q = transform * q;
    geometry_msgs::Quaternion result;
    tf::quaternionTFToMsg(q, result);
    return result;
  }
  // Fail
  return geometry_msgs::Quaternion();
}

float transformAngle(const tf::TransformListener& listener,
    const char* from, const char* to, ros::Time time = ros::Time()) {
  geometry_msgs::Quaternion q_zero;
  q_zero.w = 1;
  return tf::getYaw(transformQuaternion(listener, q_zero, from, to, time));
}

#endif // TF_HELPER_H
