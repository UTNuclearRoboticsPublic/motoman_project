#include "../include/kinect_icp.hpp"

using namespace pcl;

// EuclideanCluster::EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle n)
//     : nh_(nh),
//       rate_(n.param("loop_rate", 10)),
//       frame_id_(n.param<std::string>("clustering_frame_id", "world"))
// {
//   source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/merged_cloud"), 1, &EuclideanCluster::EuclideanCallback, this);
//   fileterd_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("filtered_pc_topic_name", "/filtered_pointcloud"), 1);
//   euclidean_cluster_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(n.param<std::string>("box_name", "/clustering_result"), 1);

//   // クラスタリングのパラメータを初期化
//   n.param<double>("clusterTolerance", clusterTolerance_, 0.02);
//   n.param<int>("minSize", minSize_, 100);
//   n.param<int>("maxSize", maxSize_, 25000);
//   // clopboxを当てはめるエリアを定義
//   n.param<float>("crop_x_min", crop_min_.x, 0.15);
//   n.param<float>("crop_x_max", crop_max_.x, 1.5);
//   n.param<float>("crop_y_min", crop_min_.y, -1.5);
//   n.param<float>("crop_y_max", crop_max_.y, 1.5);
//   n.param<float>("crop_z_min", crop_min_.z, 0.01);
//   n.param<float>("crop_z_max", crop_max_.z, 0.5);

//   n.param<float>("clustering_min_height", min_height_, 0.2);
// }

void KinectICP::Kinect_cb(const sensor_msgs::PointCloud2::ConstPtr &source_pc) {

  // 点群をKinect座標系からWorld座標系に変換
  // 変換されたデータはtrans_pcに格納される．
  sensor_msgs::PointCloud2 trans_pc;
  try {
    pcl_ros::transformPointCloud(frame_id_, *source_pc, trans_pc, tf_);
  } catch (tf::ExtrapolationException e) {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }
  
  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ> pcl_source;
  pcl::fromROSMsg(*source_pc, pcl_source);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<pcl::PointXYZ>(pcl_source));

  // 点群の中からnanを消す
  // std::vector<int> dummy;
  // pcl::removeNaNFromPointCloud(*pcl_source_ptr, *pcl_source_ptr, dummy);

  pcl::io::savePCDFileBinary ("kinect_pcd.pcd", *pcl_source_ptr);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Kinect ICP");
  ros::NodeHandle n;
  ros::Subscriber source_pc_sub = n.subscribe("kinect_first/qhd/points", 1000);
  ros::spin();
  return 0;
}
  
  // ****** 以下 interactive_icp.cpp より ******

  // Visualization
//   pcl::visualization::PCLVisualizer viewer ("ICP sia5");
  
//   // Create two verticaly separated viewports
//   int v1 (0);
//   int v2 (1);
//   viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//   viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

//   // The color we will be using
//   float bckgr_gray_level = 0.0;  // Black
//   float txt_gray_lvl = 1.0 - bckgr_gray_level;


//   // Original point cloud is white
//   //pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
//   //                                                                           (int) 255 * txt_gray_lvl);
//   //viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
//   //viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);


//   // Transformed point cloud is green  ←　kinectから得られた点群データとする．
//   pcl::visualization::PointCloudColorHandlerCustom<PointT> pcl_source_ptr_color_h (pcl_source_ptr, 20, 180, 20);
//   viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

  
//   // ICP aligned point cloud is red
//   //pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
//   //viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

//   // Adding text descriptions in each viewport
//   //viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
//   //viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

//   //std::stringstream ss;
//   //ss << iterations;
//   //std::string iterations_cnt = "ICP iterations = " + ss.str ();
//   //viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

//   // Set background color
//   viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
//   viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

//   // Set camera position and orientation
//   viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//   viewer.setSize (1280, 1024);  // Visualiser window size

//   // Register keyboard callback :
//   //viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

  
//   // Display the visualiser
//   while (!viewer.wasStopped ())
//   {
//     viewer.spinOnce ();

//     // The user pressed "space" :
//     if (next_iteration)
//     {
//       // The Iterative Closest Point algorithm
//       time.tic ();
//       icp.align (*cloud_icp);
//       std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

//       if (icp.hasConverged ())
//       {
//         printf ("\033[11A");  // Go up 11 lines in terminal output.
//         printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
//         std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
//         transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
//         print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

//         ss.str ("");
//         ss << iterations;
//         std::string iterations_cnt = "ICP iterations = " + ss.str ();
//         viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
//         viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
//       }
//       else
//       {
//         PCL_ERROR ("\nICP has not converged.\n");
//         return (-1);
//       }
//     }
//     next_iteration = false;
//   }
//   return (0);
// }
