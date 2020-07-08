#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Wrench.h"
#include <iostream>
#include <eigen-3.3.7/Eigen/Core>
#include <ros/time.h>

using namespace Eigen;
Matrix <double, 12, 1> x;
Matrix <double, 12, 1> x_sens;
Matrix <double, 3, 1> y;
Matrix <double, 3, 1> y_sens;
geometry_msgs::Point y1_msg;
nav_msgs::Odometry x1_msg;
ros::Publisher sensor_pub;
ros::Publisher sensor_pub1;
ros::Timer my_sensor_frequency;

void y_sensor(const geometry_msgs::Point::ConstPtr& y_msg) {
y_sens[0]=y_msg->x;
y_sens[1]=y_msg->y;
y_sens[2]=y_msg->z;
y=y_sens;
y1_msg.x=y[0];
y1_msg.y=y[1];
y1_msg.z=y[2];
}

void x_sensor(const nav_msgs::Odometry::ConstPtr& x_msg) {
  x_sens[0]=x_msg->pose.pose.position.x;
  x_sens[1]=x_msg->pose.pose.position.y;
  x_sens[2]=x_msg->pose.pose.position.z;
  x_sens[3]=x_msg->twist.twist.linear.x;
  x_sens[4]=x_msg->twist.twist.linear.y;
  x_sens[5]=x_msg->twist.twist.linear.z;
  x_sens[6]=x_msg->pose.pose.orientation.x;
  x_sens[7]=x_msg->pose.pose.orientation.y;
  x_sens[8]=x_msg->pose.pose.orientation.z;
  x_sens[9]=x_msg->twist.twist.angular.x;
  x_sens[10]=x_msg->twist.twist.angular.y;
  x_sens[11]=x_msg->twist.twist.angular.z;
  x=x_sens;
  x1_msg.pose.pose.position.x=x[0];
  x1_msg.pose.pose.position.y=x[1];
  x1_msg.pose.pose.position.z=x[2];
  x1_msg.twist.twist.linear.x=x[3];
  x1_msg.twist.twist.linear.y=x[4];
  x1_msg.twist.twist.linear.z=x[5];
  x1_msg.pose.pose.orientation.x=x[6];
  x1_msg.pose.pose.orientation.y=x[7];
  x1_msg.pose.pose.orientation.z=x[8];
  x1_msg.twist.twist.angular.x=x[9];
  x1_msg.twist.twist.angular.y=x[10];
  x1_msg.twist.twist.angular.z=x[11];
}

// void publisherThread1()
// {
//     ros::Rate rate(250);
    
//     while (ros::ok()) {
//         sensor_pub.publish(x1_msg);
//     sensor_pub1.publish(y1_msg);
//         rate.sleep();
//     }
// }
// void callmyfunction1(const ros::WallTimerEvent& event) {
//   sensor_pub1.publish(y1_msg);
//   sensor_pub.publish(x1_msg);
// }


int main(int argc, char **argv){
  ros::init(argc, argv,"sensor_reader");
  ros::NodeHandle node_obj;
  ros::Subscriber sensor_sub1=node_obj.subscribe("/y_out",1,y_sensor);
  ros::Subscriber sensor_sub=node_obj.subscribe("/x",1,x_sensor);
  sensor_pub1= node_obj.advertise<geometry_msgs::Point>("/y_out", 1);
  sensor_pub= node_obj.advertise<nav_msgs::Odometry>("/x", 1);
  //ros::WallTimer timer = node_obj.createWallTimer(ros::WallDuration(0.004), callmyfunction1);
  ros::Rate loop_rate(25);
  while(ros::ok()) {
    sensor_pub1.publish(y1_msg);
   sensor_pub.publish(x1_msg);
    loop_rate.sleep();
    ros::spinOnce();
  }
  // std::thread worker(publisherThread1);
  // ros::spin();
  // worker.join();
  return 0;
}

