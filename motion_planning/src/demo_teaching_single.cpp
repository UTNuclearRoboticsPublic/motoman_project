#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <vector>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <dhand/Servo_move.h>
#include <std_msgs/String.h>

// #define OFSET 0.265 //腕の長さ
#define OFSET 0.2 //腕の長さ

static const double pi = 3.141592653589793;


//3つの値から中央値を求める
double median(std::vector<double> &x)
{
  for(int i = 0; i < 3 - 1; i++)
  {
    int j = i;
    for (int k = i; k < 3; k++)
    {
      if (x[k] < x[j]) j = k;
    }
    if (i < j)
    {
      double v = x[i];
      x[i] = x[j]; x[j] = v;
    }
  }
  return x[1];
}

//回転を表すquatanianを求める
std::vector<double> quat(char a, double deg)
{
  double rad = deg*pi/180;
  if(a == 'x')
  {
    return {cos(rad/2), sin(rad/2), 0, 0};
  }
  else if(a == 'y') 
  {
    return {cos(rad/2), 0, sin(rad/2), 0};
  }
  else if(a == 'z') 
  {
    return {cos(rad/2), 0, 0, sin(rad/2)};
  }
}

//quatanianの積を求める
std::vector<double> mult_quat(std::vector<double> &q, std::vector<double> &r)
{
  double w, x, y, z;
  w = q[0]*r[0] - (q[1]*r[1] + q[2]*r[2] + q[3]*r[3]);
  x = q[0]*r[1] + r[0]*q[1] + q[2]*r[3] - q[3]*r[2];
  y = q[0]*r[2] + r[0]*q[2] + q[3]*r[1] - q[1]*r[3];
  z = q[0]*r[3] + r[0]*q[3] + q[1]*r[2] - q[2]*r[1];
  return {w, x, y, z};
}

//途中の角度を求める
double mid(std::vector<double> &deg, double a, double b)
{
  for(int i = 0; i < deg.size(); i++)
  {
    if(abs(deg[i] - a) <= abs(5))
    {
      deg[i] = 0;
    }
    else if (abs(deg[i] -b) <= abs(5))
    {
      deg[i] = 0;
    }
    else
    {
      deg[i] = deg[i];
    }
  }

  std::vector<double> way_deg = {};
  for(int i = 0; i < deg.size(); i++)
  {
    if(deg[i] != 0)
    {
      way_deg.push_back(deg[i]);
    }
  }

  double sum = 0;
  for (int i = 0; i < way_deg.size(); i++)
  {
    sum += way_deg[i]; 
  }

  if(way_deg.size() != 0)
  {
    return sum / way_deg.size();
  }
  else
  {
    return b - 1;
  }
} 


int main(int argc, char **argv)
{
  // Initialize ROS, create the node handle and an async spinner
  ros::init(argc, argv, "move_group_plan_target");
  ros::NodeHandle nh;

  ros::AsyncSpinner spin(1);
  spin.start();

  // Get the arm planning group
  moveit::planning_interface::MoveGroup plan_group("arm");

  // Create a published for the arm plan visualization
  ros::Publisher display_pub = nh.advertise
    <moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);


  ros::Publisher pub = nh.advertise<dhand::Servo_move>("/dhand_grasp", 1);
  dhand::Servo_move grasp_msg;
  grasp_msg.position = 0.0;
  grasp_msg.speed = 20.0;
  grasp_msg.acceleration = 2.0;
  grasp_msg.current_limit = 0.5;
  ROS_INFO("%f", grasp_msg.position);
  pub.publish(grasp_msg);


  // Read a csv file
  std::ifstream ifs("/home/motoman/workspace/ROS/motoman_ws/src/motoman_project/motion_planning/src/input_files/input_2.csv");
  if(!ifs)
    {
      std::cerr << "ファイルオープンに失敗" << std::endl;
      std::exit(1);
    }

  std::string str;
  std::vector<std::vector<double>> co_pt(100, std::vector<double>(4));  //pt = [i][j]
  int i = 0;  //読み込んだ情報の数
  int j = 0;  //ワールド座標からの対象物のx,y座標と対象物の高さとz軸の回転を表す角度 計4つ

  while(std::getline(ifs, str))
    {
      std::string token;
      std::istringstream stream(str);

      while(getline(stream, token, ','))  //1行づつ読み込んでvectorに代入
        {
          co_pt[i][j] = stod(token);
          j++;
        }
      j = 0;
      i++;
    }

    
  //中央値を求めるために要素が3個のvectorを作る
  std::vector<double> x_1 = {co_pt[0][0], co_pt[1][0], co_pt[2][0]};
  std::vector<double> y_1 = {co_pt[0][1], co_pt[1][1], co_pt[2][1]};
  std::vector<double> z_1 = {co_pt[0][2], co_pt[1][2], co_pt[2][2]};
  std::vector<double> d_1 = {co_pt[0][3], co_pt[1][3], co_pt[2][3]};
    
  std::vector<double> x_2 = {co_pt[i-4][0], co_pt[i-3][0], co_pt[i-2][0]};
  std::vector<double> y_2 = {co_pt[i-4][1], co_pt[i-3][1], co_pt[i-2][1]};
  std::vector<double> z_2 = {co_pt[i-4][2], co_pt[i-3][2], co_pt[i-2][2]};
  std::vector<double> d_2 = {co_pt[i-4][3], co_pt[i-3][3], co_pt[i-2][3]};

  //角度だけのvectorを作成する
  std::vector<double> deg = {};
  for(int k = 0; k < i-1; k++)
    {
      deg.push_back(co_pt[k][3]);
    }

  double mid_deg = mid(deg, median(d_1), median(d_2));
   


  //first point
  geometry_msgs::Pose goal; 

  std::vector<double> Q_1 = quat('y', 90);
  std::vector<double> R_1 = quat('x', median(d_1));
  std::vector<double> date_1 = mult_quat(Q_1, R_1);

  goal.orientation.w = date_1[0];
  goal.orientation.x = date_1[1];
  goal.orientation.y = date_1[2];
  goal.orientation.z = date_1[3];
  goal.position.x = median(x_1);
  goal.position.y = median(y_1);
  goal.position.z = median(z_1) + OFSET;

  plan_group.setGoalTolerance(0.2);

  plan_group.setPoseTarget(goal);

  moveit::planning_interface::MoveGroup::Plan goal_plan;
  if (plan_group.plan(goal_plan))
    {
      plan_group.move();
    }

  grasp_msg.position = 7.0;
  grasp_msg.speed = 20.0;
  grasp_msg.acceleration = 2.0;
  grasp_msg.current_limit = 0.5;
  ROS_INFO("%f", grasp_msg.position);
  pub.publish(grasp_msg);


  //second point
  std::vector<double> Q_2 = quat('y', 90);
  std::vector<double> R_2 = {};
  std::vector<double> date_2 = {};

  if (median(d_1) > mid_deg && mid_deg > median(d_2))
    {
      R_2 = quat('x', median(d_2));
    }
  else if(median(d_1) < mid_deg && mid_deg < median(d_2))
    {
      R_2 = quat('x', median(d_2));
    }
  else
    {
      R_2 = quat('x', -180 + median(d_2));
    }

  date_2 = mult_quat(Q_2, R_2);

  goal.orientation.w = date_2[0];
  goal.orientation.x = date_2[1];
  goal.orientation.y = date_2[2];
  goal.orientation.z = date_2[3];
  goal.position.x = median(x_2);
  goal.position.y = median(y_2);
  goal.position.z = median(z_2) + OFSET;

  plan_group.setGoalTolerance(0.2);

  plan_group.setPoseTarget(goal);

  moveit::planning_interface::MoveGroup::Plan goal_plan2;
  if (plan_group.plan(goal_plan2))
    {    
      plan_group.move();
    }

  grasp_msg.position = 0;
  pub.publish(grasp_msg);



  sleep(7.0);



  ros::shutdown();

  return 0;
}
