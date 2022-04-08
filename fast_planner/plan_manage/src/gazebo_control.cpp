//#include <plan_manage/gazebo_control.h>

#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <Eigen/Core>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/tf.h>

ros::Publisher trajectory_pub, position_pub;
geometry_msgs::PointStamped current_position;

  float linear_smoothing_navigation_step = 2;
  bool flag_gps_initialized_OK = false;
  bool flag_take_off_OK = false;
  int flag_tasks_OK = 0;
  Eigen::Vector3d home;


void UpdateUavPosition(const geometry_msgs::PointStamped& msg)
{
  if(!flag_gps_initialized_OK)
  {
    flag_gps_initialized_OK = true;
    home[0] = msg.point.x;
    home[1] = msg.point.y;
    home[2] = msg.point.z;
  }
  current_position = msg;
}

double getDistanceToTarget(const Eigen::Vector3d& target)
{
  double temp = 0;
  temp += pow((target[0] - current_position.point.x), 2);
  temp += pow((target[1] - current_position.point.y), 2);
  temp += pow((target[2] - current_position.point.z), 2);
  return temp;
}

bool reachTargetPosition(const Eigen::Vector3d& target, float max_error)
{
  double temp = getDistanceToTarget(target);
  if(temp<max_error)
    return true;
  return false;
}

bool linearSmoothingNavigationTask(const Eigen::Vector3d& target)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    if (reachTargetPosition(target,0.2))
        return true;

    double dist = getDistanceToTarget(target);
    Eigen::Vector3d next_step;

    if(dist<linear_smoothing_navigation_step)
    {
        next_step = target;
    }
    else
    {
        next_step[0] = current_position.point.x+(target[0]-current_position.point.x)/dist*linear_smoothing_navigation_step;
        next_step[1] = current_position.point.y+(target[1]-current_position.point.y)/dist*linear_smoothing_navigation_step;
        next_step[2] = current_position.point.z+(target[2]-current_position.point.z)/dist*linear_smoothing_navigation_step;
    }

    double desired_yaw = 0.0; 

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_step, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    return false;
}

bool takeOffTask(float height)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    static Eigen::Vector3d desired_position(current_position.point.x, current_position.point.y, height);
    double desired_yaw = 0.0;

    if (reachTargetPosition(desired_position,0.2))
        return true;

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    return false;
}

void gohome()
{
    static Eigen::Vector3d temp(home[0], home[1], current_position.point.z);
    static bool flag_temp = false;

    if (!flag_temp)
    {
        flag_temp = linearSmoothingNavigationTask(temp);
    }
    else
    {
        linearSmoothingNavigationTask(home);
    }
}

void cmd_callback(const quadrotor_msgs::PositionCommand& pos_cmd)
{
  // trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  // trajectory_msg.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped cmd;
  cmd.header.stamp = ros::Time::now();
  double desired_yaw = pos_cmd.yaw;
  static Eigen::Vector3d desired_position(pos_cmd.position.x, pos_cmd.position.y, pos_cmd.position.z);
  if(reachTargetPosition(desired_position, 0.2))
    return;
  // mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
  // trajectory_pub.publish(trajectory_msg);
  cmd.header.frame_id = "world";
  cmd.pose.position = pos_cmd.position;
  cmd.pose.orientation = tf::createQuaternionMsgFromYaw(pos_cmd.yaw);
  position_pub.publish(cmd);
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "Gazegbo_Cotroller");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  std::string uav_name ="";
  ros::param::get("~mav_name", uav_name);

  ros::Subscriber position_sub = nh.subscribe(std::string("/"+uav_name+"/odometry_sensor1/position").c_str(), 10, &UpdateUavPosition);
  ros::Subscriber cmd_sub      = nh.subscribe("/planning/pos_cmd", 10, &cmd_callback);
  //trajectory_pub = nh.advertise<trajectory_msgs:MultiDOFJointTrajectory>("/gazebo_control/traj", 10); 
  trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/ardrone/command/trajectory", 10); 
  position_pub   = nh.advertise<geometry_msgs::PoseStamped>("/ardrone/command/pose", 10);
  
  // ros::Duration(5.0).sleep();
  // std_srvs::Empty srv;
  // bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  
  // int i=0;
  // while (i <= 10 && !unpaused) {
  //       ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
  //       std::this_thread::sleep_for(std::chrono::seconds(1));
  //       unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  //       ++i;
  // }

  // if (!unpaused) {
  //   ROS_FATAL("Could not wake up Gazebo");
  //   return -1;
  // } else {
  //   ROS_INFO("Unpaused the Gazebo simulation");
  // }

  // std::vector<Eigen::Vector3d> path;
  // path.push_back(Eigen::Vector3d(5.f, 5.f, 5.f));
  // path.push_back(Eigen::Vector3d(-5.f,5.f,5.f));
  // path.push_back(Eigen::Vector3d(-5.f,-5.f,5.f));
  // path.push_back(Eigen::Vector3d(5.f,-5.f,5.f));
  // path.push_back(Eigen::Vector3d(5.f,5.f,5.f));
  // std::cout<<path.size()<<std::endl;
  // ros::Rate loop_rate(10);  
  // while (ros::ok())
  // {
  //   if(flag_gps_initialized_OK && !flag_take_off_OK)
  //   {
  //     flag_take_off_OK = takeOffTask(3);
  //   }
  //   else if(flag_take_off_OK && flag_tasks_OK < path.size())
  //   {
  //     if(flag_tasks_OK<path.size())
  //     {
  //       bool temp = linearSmoothingNavigationTask(path[flag_tasks_OK]);
	// if(temp)
	//   flag_tasks_OK++;
  //     }
  //   }
  //   else if (flag_tasks_OK >= path.size())
  //   {
  //     if(flag_tasks_OK<path.size())
  //     {
  //       bool temp = linearSmoothingNavigationTask(path[flag_tasks_OK]);
  //       if(temp)
	//   flag_tasks_OK++;
  //     }
  //   }
  //   else if(flag_tasks_OK >= path.size())
  //   {
  //     gohome();
  //   }
  // }
    ros::spinOnce();
    // loop_rate.sleep();

}

