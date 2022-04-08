#ifndef VORONOI_H_
#define VORONOI_H_

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <nav_msgs/Path.h>
#include <traj_utils/planning_visualization.h>
#include <dynamic_voronoi/dynamicvoronoi.h>
#include <nav_msgs/OccupancyGrid.h>
#include <plan_manage/Bspline.h>

using std::vector;

namespace fast_planner {
  
class VoronoiReplan {

  private:
    enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
    enum TARGET_TYPE { MANUAL_TARGET=1, PRESET_TARGET=2, REFENCE_PATH=3};

    FastPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;

    /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // target state
  int current_wp_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_, map_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_; 

    DynamicVoronoi voronoiDiagram;
    bool** binMap;
    nav_msgs::OccupancyGrid gridMap;
    double originX, originY;//表示地图的起始位置
    int height, width;//代表地图的尺寸
    
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();

    void execFSMCallback(const ros::TimerEvent& e);
    void waypointCallback(const nav_msgs::Path::ConstPtr& msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void checkCollisionCallback(const ros::TimerEvent& e);
    bool callVoronoiReplan();
  public:
    void init(ros::NodeHandle& nh);
    VoronoiReplan(){}
    ~VoronoiReplan() {
    }

     
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif
