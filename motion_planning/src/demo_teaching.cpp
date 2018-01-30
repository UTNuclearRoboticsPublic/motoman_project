#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdlib>
#include <math.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>

#define OFSET 0.5


//中央値を求める
double median(double *x)
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




    // Read a csv file
    // std::ifstream ifs("/home/nakao/motoman_ws/src/motion_planning/src/input_files/input_2.csv");
    std::ifstream ifs("./src/motion_planning/src/input_files/input_2.csv");
    if(!ifs)
    {
      std::cerr << "ファイルオープンに失敗" << std::endl;
      std::exit(1);
    }

    std::string str;
    double co_pt[100][7] = {};  //pt = [i][j]
    int i = 0;  //移動する地点の数×2
    int j = 0;  //x,y,z座標とorientation x,y,z,w

    while(std::getline(ifs, str))
    {
      std::string token;
      std::istringstream stream(str);

      while(getline(stream, token, ','))
      {
	co_pt[i][j] = stod(token);
	j++;
      }
      j = 0;
      i++;
    }
    
    //中央値を求めるために要素が3個の配列を作る
    double a_orix_1[3] = {co_pt[0][2], co_pt[2][2], co_pt[4][2]};  //z軸の回転をx軸に変えている
    double a_x_1[3] = {co_pt[0][4], co_pt[2][4], co_pt[4][4]};
    double a_y_1[3] = {co_pt[0][5], co_pt[2][5], co_pt[4][5]};
    double a_z_1[3] = {co_pt[0][6], co_pt[2][6], co_pt[4][6]};
    
    double a_orix_2[3] = {co_pt[i-7][2], co_pt[i-5][2], co_pt[i-3][2]};
    double a_x_2[3] = {co_pt[i-7][4], co_pt[i-5][4], co_pt[i-3][4]};
    double a_y_2[3] = {co_pt[i-7][5], co_pt[i-5][5], co_pt[i-3][5]};
    double a_z_2[3] = {co_pt[i-7][6], co_pt[i-5][6], co_pt[i-3][6]};
    
    double b_orix_1[3] = {co_pt[1][2], co_pt[3][2], co_pt[5][2]};
    double b_x_1[3] = {co_pt[1][4], co_pt[3][4], co_pt[5][4]};
    double b_y_1[3] = {co_pt[1][5], co_pt[3][5], co_pt[5][5]};
    double b_z_1[3] = {co_pt[1][6], co_pt[3][6], co_pt[5][6]};
    
    double b_orix_2[3] = {co_pt[i-6][2], co_pt[i-4][2], co_pt[i-2][2]};
    double b_x_2[3] = {co_pt[i-6][4], co_pt[i-4][4], co_pt[i-2][4]};
    double b_y_2[3] = {co_pt[i-6][5], co_pt[i-4][5], co_pt[i-2][5]};
    double b_z_2[3] = {co_pt[i-6][6], co_pt[i-4][6], co_pt[i-2][6]};
    



    //first point
    // Set a goal message as a pose of the end effector
    geometry_msgs::Pose goal;
    goal.orientation.x = median(a_orix_1);
    goal.orientation.y = sqrt(0.5);
    goal.orientation.z = 0;
    goal.orientation.w = sqrt(0.5 - pow(median(a_orix_1), 2));
    goal.position.x = median(a_x_1);
    goal.position.y = median(a_y_1);
    goal.position.z = median(a_z_1) + OFSET;

    // Set the tolerance to consider the goal achieved
    plan_group.setGoalTolerance(0.2);

    // Set the target pose, which is the goal we already defined
    plan_group.setPoseTarget(goal);

    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroup::Plan goal_plan;
    if (plan_group.plan(goal_plan))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan.start_state_;
        display_msg.trajectory.push_back(goal_plan.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        plan_group.move();
    }

    sleep(7.0);



    //second point
    goal.orientation.x = median(a_orix_2);
    goal.orientation.y = sqrt(0.5);
    goal.orientation.z = 0;
    goal.orientation.w = sqrt(0.5 - pow(median(a_orix_2), 2));
    goal.position.x = median(a_x_2);
    goal.position.y = median(a_y_2);
    goal.position.z = median(a_z_2) + OFSET;

    // Set the tolerance to consider the goal achieved
    plan_group.setGoalTolerance(0.2);

    // Set the target pose, which is the goal we already defined
    plan_group.setPoseTarget(goal);

    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroup::Plan goal_plan2;
    if (plan_group.plan(goal_plan2))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan2.start_state_;
        display_msg.trajectory.push_back(goal_plan2.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        plan_group.move();
    }

    sleep(7.0);



    //third point
    goal.orientation.x = median(b_orix_1);
    goal.orientation.y = sqrt(0.5);
    goal.orientation.z = 0;
    goal.orientation.w = sqrt(0.5 - pow(median(b_orix_1), 2));
    goal.position.x = median(b_x_1);
    goal.position.y = median(b_y_1);
    goal.position.z = median(b_z_1) + OFSET;

    // Set the tolerance to consider the goal achieved
    plan_group.setGoalTolerance(0.2);

    // Set the target pose, which is the goal we already defined
    plan_group.setPoseTarget(goal);

    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroup::Plan goal_plan3;
    if (plan_group.plan(goal_plan3))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan3.start_state_;
        display_msg.trajectory.push_back(goal_plan3.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        plan_group.move();
    } 
    
    sleep(7.0);


    
    //forth point
    goal.orientation.x = median(b_orix_2);
    goal.orientation.y = sqrt(0.5);
    goal.orientation.z = 0;
    goal.orientation.w = sqrt(0.5 - pow(median(b_orix_2), 2));
    goal.position.x = median(b_x_2);
    goal.position.y = median(b_y_2);
    goal.position.z = median(b_z_2) + OFSET;

    // Set the tolerance to consider the goal achieved
    plan_group.setGoalTolerance(0.2);

    // Set the target pose, which is the goal we already defined
    plan_group.setPoseTarget(goal);

    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroup::Plan goal_plan4;
    if (plan_group.plan(goal_plan4))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan4.start_state_;
        display_msg.trajectory.push_back(goal_plan4.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        plan_group.move();
    } 


    ros::shutdown();

    return 0;
}
