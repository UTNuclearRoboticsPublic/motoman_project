#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_type.h>
#include <pcl/filters/passthrough.h>


int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ);

  //Fill in the cloud data
  pcl::PCDReader reader;

  //保存したファイルの読み込み
  reader.read<pcl::pointXYZ> ("******.pcd", *cloud);     //******には.pcdのデータファイルを.

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");   //フィルタリングを行う次元の設定．
  pass.setFilterLimits(***, ***);   //範囲指定．
  //pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("motoman_only.pcd", *colud_filtered, false);

  return(0);
}
