#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <random>
#include <ctime>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_test");
    ros::NodeHandle nh("~");

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    int count = 0;

    std::default_random_engine e(time(0));
    
    std::uniform_real_distribution<double> x_random(-20.0, 20.0);
    std::uniform_real_distribution<double> y_random(-10.0, 10.0);
    double z = 5;
    ros::spinOnce();
    while(count<50)
    {
        double x = x_random(e);
        double y = y_random(e);

        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "world";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = z;
        
        goal.pose.orientation.x = 0;
        goal.pose.orientation.y = 0;
        goal.pose.orientation.z = 0;
        goal.pose.orientation.w = 1;
        goal_pub.publish(goal);
        std::cout<<"终点为：("<<x<<","<<y<<")\n";
        ros::Duration(5.0).sleep();
        count++;
    }
    return 0;
}