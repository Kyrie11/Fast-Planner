#ifndef _VORONOI_BUBBLE_H
#define _VORONOI_BUBBLE_H

#include <Eigen/Eigen>
#include <iostream>
#include <queue>
#include <list>
#include <ros/ros.h>

#include "dynamic_voronoi/dynamicvoronoi.h"
#include "plan_env/edt_environment.h"

namespace fast_planner {

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

typedef VoronoiNode* VoronoiNodePtr;

class NodeComparator1 {
public: 
  bool operator()(VoronoiNode* node1, VoronoiNode* node2) {
    return node1->f_score + node1->g_score > node2->f_score + node2->g_score;
  }
};



class VoronoiBubble {
  private:

    typedef enum {voronoiKeep = -4, freeQueued = -3, voronoiRetry = -2, voronoiPrune = -1, free = 0, occupied = 1} State;

    std::vector<VoronoiNode*> path_node_pool_;
    std::vector<VoronoiNode> step_nodes;
    int use_node_num_, iter_num_;
    
    std::priority_queue<VoronoiNode*, std::vector<VoronoiNode*>, NodeComparator1> open_set_; 
    //std::list<VoronoiNode*> open_set_; 
        /* ---------- record data ---------- */
    Eigen::Vector3d start_vel_, end_vel_, start_acc_;
    Eigen::Matrix<double, 6, 6> phi_;  // state transit matrix
    // shared_ptr<SDFMap> sdf_map;
    EDTEnvironment::Ptr edt_environment_;
    bool is_shot_succ_ = false;
    Eigen::MatrixXd coef_shot_;
    double t_shot_;
    bool has_path_ = false;
    double dis_thresh;

    ros::Publisher voronoi_nodes_pub_;
    DynamicVoronoi diagram;
    /* ---------- parameter ---------- */
    /* search */
    double max_tau_, init_max_tau_;
    double max_vel_, max_acc_;
    double w_time_, horizon_, lambda_heu_;
    int allocate_num_, check_num_;
    double tie_breaker_;
    bool optimistic_;
    double time_duration_;
    /* map */
    double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
    Eigen::Vector3d origin_, map_size_3d_;
    double time_origin_;
    double cellResolution;
    ros::NodeHandle node;

    void retrievePath(VoronoiNode* end_node);
    bool iSstateTransitable(Eigen::Matrix<double, 6, 1> state0, Eigen::Matrix<double, 6, 1> state1, Eigen::Vector3d um, double tau); 
    void stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um, double tau);
    void getDynamicInfo(Eigen::Matrix<double, 6, 1> state0, Eigen::Matrix<double, 6, 1>& state1, double tau, Eigen::Vector3d& input);
    double movementCost(Eigen::Vector3d state1, Eigen::Vector3d state2);
  public:
    VoronoiBubble(){};
    VoronoiBubble(ros::NodeHandle& nh);
    ~VoronoiBubble();
    vector<double> quartic(double a, double b, double c, double d, double e);
    vector<double> cubic(double a, double b, double c, double d);
    VoronoiNode* getClosestVoro(Eigen::Vector3d point, Eigen::Vector3d end_pt);
    double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time);
    double calcCost(VoronoiNode* cur_node, VoronoiNode* pro_node, VoronoiNode goal);
    int search2(Eigen::Vector3d start_pt_, Eigen::Vector3d start_v_, Eigen::Vector3d starrt_acc, Eigen::Vector3d end_pt_, Eigen::Vector3d end_v_, DynamicVoronoi voronoiDiagram, bool init);
    int search(Eigen::Vector3d start_pt_, Eigen::Vector3d start_v_, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt_, Eigen::Vector3d end_v_, DynamicVoronoi voronoiDiagram);//维诺图寻路算法
    void step();
 //   void setParam(ros::NodeHandle& nh);

    void setEnvironment(const EDTEnvironment::Ptr& env);
    void displayVoronoiNodes(vector<Eigen::Vector3d> list);    
    std::vector<Eigen::Vector3d> getVoronoiTraj(double delta_t=0.1);
    void init();
    bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);
    void reset();
    void getSamples(double& ts, vector<Eigen::Vector3d>& point_set, vector<Eigen::Vector3d>& start_end_derivatives);
    bool reachHorizion(Eigen::Vector3d s, DynamicVoronoi& voronoiDiagram); 
    Eigen::Vector3d getTmpEnd(Eigen::Vector3d start_pt_, Eigen::Vector3d end_pt_, DynamicVoronoi& voronoiDiagram);
    enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };
    typedef shared_ptr<VoronoiBubble> Ptr;

    std::vector<VoronoiNode*> path_nodes_;
    VoronoiNode* terminal;
};
}
#endif
