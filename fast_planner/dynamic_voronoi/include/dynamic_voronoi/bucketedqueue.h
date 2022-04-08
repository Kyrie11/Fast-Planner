#ifndef _PRIORITYQUEUE2_H_
#define _PRIORITYQUEUE2_H_

#define MAXDIST 1000
#define RESERVE 64

#include <Eigen/Eigen>
#include <vector>
#include <set>
#include <queue>
#include <assert.h>

namespace fast_planner {

class BucketPrioQueue {
 public:
  BucketPrioQueue();
  bool empty() const;
  void push(int prio, Eigen::Vector3d t);
  Eigen::Vector3d pop();

 private:
  static void initSqrIndices();
  static std::vector<int> sqrIndices;
  static int numBuckets;
  int count;
  int nextBucket;

  std::vector<std::queue<Eigen::Vector3d>> buckets;
};
}

#endif
