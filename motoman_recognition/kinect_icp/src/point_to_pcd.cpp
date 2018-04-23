#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

class PointCloudToPCD
{
  protected:
    ros::NodeHandle nh_;

  private:
    std::string prefix_;

  public:
  string cloud_topic_;

  ros::Subscriber sub_;

  //callback
  void cloud_cb(const sensor_msgs::PointCloud2constPtr& cloud)
  {
    if ((cloud->width * cloud->hight) ==0)
      return;

    ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
              (int)cloud->width * cloud->height,
              cloud->header.frame_id.c_str (),
              pcl::getFieldsList (*cloud).c_str ());

    std::stringstream ss;
    ss << prefix_ << cloud->header.stamp << ".pcd";
    ROS_INFO ("Data saved to %s", ss.str ().c_str ());

    pcl::io::savePCDFileBinary (ss.str (), *cloud, Eigen::Vector4f::Zero (),
                          Eigen::Quaternionf::Identity (), false);
  }

  PointCloudToPCD ()
  {
    // Check if a prefix parameter is defined for output file names.
    ros::NodeHandle priv_nh("~");
    if (priv_nh.getParam ("prefix", prefix_))
    {
      ROS_INFO_STREAM ("PCD file prefix is: " << prefix_);
    }
    else if (nh_.getParam ("prefix", prefix_))
    {
      ROS_WARN_STREAM ("Non-private PCD prefix parameter is DEPRECATED: "
                       << prefix_);
    }

    cloud_topic_ = "input";

    sub_ = nh_.subscribe (cloud_topic_, 1,  &PointCloudToPCD::cloud_cb, this);
    ROS_INFO ("Listening for incoming data on topic %s",
              nh_.resolveName (cloud_topic_).c_str ());
  }
};

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pointcloud_to_pcd", ros::init_options::AnonymousName);

  PointCloudToPCD b;
  ros::spin();

  return (0);
}
