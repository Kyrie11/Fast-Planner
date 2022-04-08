#include "dynamic_voronoi/bucketedqueue.h"

#include "limits.h"
#include <stdio.h>
#include <stdlib.h>

using namespace fast_planner;

std::vector<int> BucketPrioQueue::sqrIndices;
int BucketPrioQueue::numBuckets;


BucketPrioQueue::BucketPrioQueue() {
  if (sqrIndices.size()==0) initSqrIndices();
  nextBucket = INT_MAX;
  buckets = std::vector<std::queue<Eigen::Vector3d>>(numBuckets);
  count = 0;
}

bool BucketPrioQueue::empty() const{
  return (count==0);
}

void BucketPrioQueue::push(int prio, Eigen::Vector3d t) {
  if(prio>=(int)sqrIndices.size()) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }

  int id = sqrIndices[prio];
  if (id<0) {
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  buckets[id].push(t);
  //    printf("pushing %d with prio %d into %d. Next: %d\n", t.x, prio, id, nextBucket);
  if (id<nextBucket) nextBucket = id;
  //    printf("push new next is %d\n", nextBucket);
  count++;
  
}

Eigen::Vector3d BucketPrioQueue::pop() {
  int i;
  assert(count>0);
  for(i=nextBucket; i<(int)buckets.size(); i++) {
    if(!buckets[i].empty()) break;
  }
  assert(i<(int)buckets.size());
  nextBucket = i;
  count--;
  Eigen::Vector3d p = buckets[i].front();
  buckets[i].pop();
  return p;
}

void BucketPrioQueue::initSqrIndices() {
  //    std::cout << "BUCKETQUEUE Starting to build the index array...\n";
  //  std::set<int> sqrNumbers;

  sqrIndices = std::vector<int>(2*MAXDIST*MAXDIST+1, -1);

  int count=0;
  for (int x=0; x<=MAXDIST; x++) {
    for (int y=0; y<=x; y++) {
      int sqr = x*x+y*y;
      sqrIndices[sqr] = count++;
    }
  }
  numBuckets = count;
  //    std::cout << "BUCKETQUEUE Done with building the index arrays.\n";
}
