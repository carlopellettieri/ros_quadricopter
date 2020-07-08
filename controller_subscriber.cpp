#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Wrench.h"
#include <iostream>
#include <eigen-3.3.7/Eigen/Core>
#include <ros/time.h>
#include <thread>

using namespace Eigen;
Matrix <double, 12, 1> x;
Matrix <double, 12, 1> x_sens;
Matrix <double, 3, 1> y;
Matrix <double, 3, 1> y_sens;
Matrix <double, 3, 1> y_ast;
Matrix <double, 3, 1> y_tilde_integrate;
Matrix <double, 4, 15> K_lqre;
Matrix <double, 3, 1> y_tilde;
Matrix <double, 3, 1> y_tilde_out;
Matrix <double, 4, 1> u;
Matrix <double, 3, 1> sigma_star;
ros::Publisher ctrl_pub;
ros::WallTime previous;
ros::WallTime now;
geometry_msgs::Wrench msg2;
ros::Timer my_desired_frequency;
ros::WallDuration error;

// void callmyfunction(const ros::WallTimerEvent& event) {
//   error = event.current_expected- event.last_expected;
// }


void y_out_controller_callback(const geometry_msgs::Point::ConstPtr& msg1) {
y[0]=msg1->x;
y[1]=msg1->y;
y[2]=msg1->z;
}

void x_controller_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  x[0]=msg->pose.pose.position.x;
  x[1]=msg->pose.pose.position.y;
  x[2]=msg->pose.pose.position.z;
  x[3]=msg->twist.twist.linear.x;
  x[4]=msg->twist.twist.linear.y;
  x[5]=msg->twist.twist.linear.z;
  x[6]=msg->pose.pose.orientation.x;
  x[7]=msg->pose.pose.orientation.y;
  x[8]=msg->pose.pose.orientation.z;
  x[9]=msg->twist.twist.angular.x;
  x[10]=msg->twist.twist.angular.y;
  x[11]=msg->twist.twist.angular.z;
y_tilde = y - y_ast;
now=ros::WallTime::now();
double dt = (now-previous).toSec();
previous=ros::WallTime::now();
//double dt = 0.004;
y_tilde_out += y_tilde * dt;
VectorXd x_tilde(x.size() + y_tilde_out.size());
x_tilde << x, y_tilde_out;
u = K_lqre * x_tilde;
    msg2.force.z=u[0];
    msg2.torque.x=u[1];
    msg2.torque.y=u[2];
    msg2.torque.z=u[3];
   ROS_INFO("%f", dt);

}

// void publisherThread()
// {
//     ros::Rate rate(250);
    
//     while (ros::ok()) {
//         ctrl_pub.publish(msg2);
//         rate.sleep();
//     }
// }



int main(int argc, char **argv){
K_lqre <<
    -0.0000,   -0.0000,    0.4863,   -0.0000,   -0.0000,    0.6973,   -0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    0.0000,   -0.0000,   -0.0000,    0.1410,
   -0.0000,   -0.6731,    0.0000,   -0.0000,   -0.4276,    0.0000,   -1.4428,    0.0000,   -0.0000,   -0.2564,    0.0000,    0.0000,   -0.0000,   -0.3080,    0.0000,
    0.6731,    0.0000,   -0.0000,    0.4276,    0.0000,   -0.0000,    0.0000,   -1.4428,    0.0000,    0.0000,   -0.2564,   -0.0000,    0.3080,    0.0000,   -0.0000,
    0.0000,    0.0000,   -0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,   -0.0000,   -0.4411,    0.0000,   -0.0000,   -0.1817,    0.0000,    0.0000,   -0.0000;
y_ast<< 15,
  -10,
  20;
  sigma_star <<   -21.8363,
   10.9182,
  -34.2352;
  y_tilde_out=sigma_star;
  ros::init(argc, argv,"controller_subscriber");
  ros::Time::init();
  ros::NodeHandle node_obj;
  // ros::WallTimer timer = node_obj.createWallTimer(ros::WallDuration(0.004), callmyfunction);
  // dt=error.toSec();
  ros::Subscriber ctrl_sub1=node_obj.subscribe("/y_out",1,y_out_controller_callback);
  ros::Subscriber ctrl_sub=node_obj.subscribe("/x",1,x_controller_callback);
  ctrl_pub= node_obj.advertise<geometry_msgs::Wrench>("/u", 1);
  // ros::Rate loop_rate(150);
  while(ros::ok()) {
    
    ctrl_pub.publish(msg2);
    // loop_rate.sleep();
    ros::spinOnce();
  }
  //std::thread worker(publisherThread);
  //ros::spin();
  //worker.join();
  return 0;
}


