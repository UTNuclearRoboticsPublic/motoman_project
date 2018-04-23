#include <iostream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


void kinectCallback(const sensor_msgs::PointCloud2 &input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(input, cloud);

  pcl::io::savePCDFileASCII ("gazebo.pcd", cloud);
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "kinect_pcd");

  ROS_INFO("Started PCL write node");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/merged_cloud2", 10, kinectCallback);
  ros::spin();

  return 0;

}
