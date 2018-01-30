#include <ros/ros.h>
#include <math.h>
#include <vector>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <dhand/Servo_move.h>
#include <std_msgs/String.h>

static const double pi = 3.141592653589793;

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

std::vector<double> mult_quat(std::vector<double> &q, std::vector<double> &r)
{
  double w, x, y, z;
  w = q[0]*r[0] - (q[1]*r[1] + q[2]*r[2] + q[3]*r[3]);
  x = q[0]*r[1] + r[0]*q[1] + q[2]*r[3] - q[3]*r[2];
  y = q[0]*r[2] + r[0]*q[2] + q[3]*r[1] - q[1]*r[3];
  z = q[0]*r[3] + r[0]*q[3] + q[1]*r[2] - q[2]*r[1];
  return {w, x, y, z};
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "move_group_plan_target");
  ros::NodeHandle nh;

  ros::AsyncSpinner spin(1);
  spin.start();

  moveit::planning_interface::MoveGroup plan_group("arm");

  ros::Publisher display_pub = nh.advertise
    <moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  std::vector<double> Q = quat('y', 0);
  std::vector<double> R = quat('x', 90);
  std::vector<double> date = mult_quat(Q, R);

  geometry_msgs::Pose goal;
  goal.orientation.w = date[0];
  goal.orientation.x = date[1];
  goal.orientation.y = date[2];
  goal.orientation.z = date[3];
  goal.position.x = 0.4;
  goal.position.y = 0;
  goal.position.z = 0.65;

  // plan_group.setGoalTolerance(0.2);
  
  plan_group.setPoseTarget(goal);

  moveit::planning_interface::MoveGroup::Plan goal_plan;
  if (plan_group.plan(goal_plan))
  {
    plan_group.move();
  }

  // ros::Publisher pub {nh.advertise<Type>("/servo", 1)};
  // Type data {42};
  // pub.publish(data);

  ros::Publisher pub = nh.advertise<dhand::Servo_move>("/dhand_grasp", 1);
  dhand::Servo_move grasp_msg;
  grasp_msg.position = 3;
  pub.publish(grasp_msg);

  // plan_group.plan(goal_plan);

  std::cout << date[0] << std::endl;
  std::cout << date[1] << std::endl;
  std::cout << date[2] << std::endl;
  std::cout << date[3] << std::endl;

  ros::shutdown();

  return 0;
}
