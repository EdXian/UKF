#include <iostream>
#include "ukf.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>


geometry_msgs::PoseStamped mocap_pose;
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  mocap_pose = *msg;
}
sensor_msgs::Imu imu_data;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu_data = *msg;//test
}

int main(int argc, char **argv){
  ukf ukf1;

  ros::init(argc, argv, "ukf");
  ros::NodeHandle nh;
  ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody2/pose", 10, mocap_cb);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/drone2/mavros/imu/data", 10, imu_cb);



  ros::Rate loop_rate(30);
  while(ros::ok()){


//    ukf1.predict();
//    ukf1.correct(0.5);

    std::cout <<"ok"<<std::endl;
    ros::spinOnce();
    loop_rate.sleep();
  }

return 0;
}
