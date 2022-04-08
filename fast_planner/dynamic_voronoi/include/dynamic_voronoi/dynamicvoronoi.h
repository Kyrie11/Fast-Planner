#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/any.hpp>
#include <list>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include "bucketedqueue.h"
#include <visualization_msgs/Marker.h>

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

namespace fast_planner {

class VoronoiNode {
 public:
  int x, y;
  float dist;
  char voronoi;
  char queueing;
  int obstX, obstY;
  bool needsRaise;
  bool isAdded;
  bool isMarked;
  int sqdist;
  char node_state;
  Eigen::Matrix<double, 6, 1> state;//前面三个为位置，后面三个为速度
  Eigen::Vector3d input;
  double g_score, f_score;
  double duration;
  double time;
  double distanceCost;
  //double isParent;

  VoronoiNode* parent;
  
  /*----------------------*/
  VoronoiNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~VoronoiNode(){};

  float movementCost(const VoronoiNode pred) const {return sqrt((state[0]-pred.state[0])*(state[0]-pred.state[0]) + (state[1]-pred.state[1])*(state[1]-pred.state[1]));}

  void updateG(VoronoiNode node) { g_score += movementCost(node);}
  void updateH(const VoronoiNode goal) { f_score= movementCost(goal);}
  double getC() { return g_score + f_score /*+   1.5*distanceCost*/;}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class DynamicVoronoi {
  private:
    //boost::mutex mutex; 
  public: 
    DynamicVoronoi(ros::NodeHandle& nh);	
    DynamicVoronoi();
    ~DynamicVoronoi();
 
    void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap = true);
    void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);
    
    void occupyCell(int x, int y);
    void clearCell(int x, int y);
    void exchangeObstacles(const std::vector<Eigen::Vector3d> &newObsctacles);
    
    void update(bool updateRealDist = true);
    void prune();

    float getDistance(int x, int y) const;
    bool isVoronoi(int x, int y) const;
    bool isOccupied(int x, int y) const;
    void visualize(const char* filename = "result.ppm");
    void brushFireMark(int x, int y);
    void visualizeMark(double resolution);
   
    unsigned int getSizeX() const {return sizeX;}
    unsigned int getSizeY() const {return sizeY;}

    bool isOnGrid(const int x, const int y) const {
      return x>=0 && x<sizeX && y>=0 && y<sizeY;
    }

    ros::Publisher marker_pub_;
    ros::NodeHandle n;
  public: 
    struct dataCell {
      dataCell() : x(0), y(0) {}
      dataCell(int _x, int _y) : x(_x), y(_y) {}
      float dist;
      char voronoi;
      char queueing;
      int obstX;
      int obstY;
      bool needsRaise;
      bool isAdded;
      int sqdist;
      bool isMarked;//在"Bubble"寻路方法中使用的标志位,brushfire标志位
      int x, y;
      std::queue<dataCell> adjs;//存储相邻的维诺点
    };
    
    typedef enum {voronoiKeep = -4, freeQueued = -3, voronoiRetry = -2, voronoiPrune = -1, free = 0, occupied = 1} State;
    typedef enum {fwNotQueued = 1, fwQueued = 2, fwProcessed = 3, bwQueued = 4, bwProcessed = 1} QueueingState;
    typedef enum {invalidObstData = SHRT_MAX / 2} ObstDataState;
    typedef enum {pruned, keep, retry} markerMatchResult;
    typedef enum {added = true, notAdded = false} addedState;

    void setObstacle(int x, int y);
    void removeObstacle(int x, int y);
    inline void checkVoro(int x, int y, int nx, int ny, VoronoiNode& c, VoronoiNode& nc);
    void recheckVoro();
    void commitAndColorize(bool updateRealDist = true);
    inline void reviveVoroNeighbors(int& x, int& y);
  
    inline bool isOccupied(int& x, int& y, VoronoiNode& c);
    inline markerMatchResult markerMatch(int x, int y);
  
  BucketPrioQueue open;
  std::queue<Eigen::Vector3d> pruneQueue;
  
  std::vector<Eigen::Vector3d> removeList;
  std::vector<Eigen::Vector3d> addList;
  std::vector<Eigen::Vector3d> lastObstacles;
  std::queue<VoronoiNode*> unsortedQueue;
  std::vector<VoronoiNode> freeNodes;
  int sizeY, sizeX;
  double originX, originY;
  //dataCell** data;
  VoronoiNode** data;
  bool** gridMap;
  
  int padding;
  double doubleThreshold;
  
  double sqrt2;
  
};
}

#endif
