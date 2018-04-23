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

  //点群をKinect座標系からWorld座標系に変換
  //変換されたデータはoutputに格納される．
  sensor_msgs::PointCloud2 output;
  std::string frame_id_;
  tf::TransformListener tf_;
  try {
    pcl_ros::transformPointCloud(frame_id_, *input, output, tf_);
  } catch (tf::ExtrapolationException e) {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  //sensor_msgs::PointCloud2 -> pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud_converted;
  pcl::fromROSMsg(output, cloud_converted);
  pcl::PointCloud<pcl::PointXYZ> cloud_converted_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));

  
  // ROS_INFO("I heard: [%s]", *****);     //*****は関数名
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "kinect_pcd");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/kinect_first/hd/points", 10, kinectCallback);
  ros::spin();

  ros::Publisher pub
  return 0;

}
