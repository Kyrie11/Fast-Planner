#include <dynamic_voronoi/dynamicvoronoi.h>
#include <math.h>
#include <iostream>

namespace fast_planner {
  
DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
  originX = originY = 0;
  addList = std::vector<Eigen::Vector3d>();
}

DynamicVoronoi::DynamicVoronoi(ros::NodeHandle& nh) {
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/planning_vis/brushfireMarker", 10);
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
  originX = originY = 0;
}


DynamicVoronoi::~DynamicVoronoi() {
  /*if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  if (gridMap) {
    for (int x=0; x<sizeX; x++) delete[] gridMap[x];
    delete[] gridMap;
  }*/
}

void DynamicVoronoi::visualizeMark(double resolution)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id = 101;
  marker_pub_.publish(mk);
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = 50;
  mk.color.g = 50;
  mk.color.b = 50;
  mk.color.a = 1;
  
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;

  for (int i=0; i<sizeX; i++) 
    for (int j=0; j<sizeY; j++) {
      if (data[i][j].voronoi == free || data[i][j].voronoi == voronoiKeep || data[i][j].isMarked)
      {
        pt.x = i*resolution + originX;
	pt.y = j*resolution + originY;
        pt.z = 0;
	mk.points.push_back(pt);	
      }
    }
  marker_pub_.publish(mk);
  ros::Duration(0.001).sleep();
}
void DynamicVoronoi::visualize(const char *filename) {
  // write pgm files

  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  for(int y = sizeY-1; y >=0; y--){      
    for(int x = 0; x<sizeX; x++){	
      unsigned char c = 0;
      if (isVoronoi(x,y) || data[x][y].isMarked) {
        fputc( 255, F );
        fputc( 0, F );
        fputc( 0, F );
	freeNodes.push_back(data[x][y]);
      } else if (data[x][y].sqdist==0) {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
      } else {
        float f = 80+(data[x][y].dist*5);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );
        fputc( c, F );
        fputc( c, F );
      }
    }
  }
  fclose(F);
}

bool DynamicVoronoi::isVoronoi(int x, int y) const {
  VoronoiNode v = data[x][y];
  return (v.voronoi==free || v.voronoi==voronoiKeep);
}



void DynamicVoronoi::removeObstacle(int x, int y) {
  VoronoiNode v = data[x][y];
  if(isOccupied(x,y,v) == false) return;
  removeList.push_back(Eigen::Vector3d(x,y,0));
  v.obstX = invalidObstData;
  v.obstY = invalidObstData;
  v.queueing = bwQueued;
  data[x][y] = v;
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  sizeX = _sizeX;
  sizeY = _sizeY;
  /*if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }*/

  data = new VoronoiNode*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new VoronoiNode[sizeY];
  if (initGridMap) {
    if (gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x];
      delete[] gridMap;
    }
    gridMap = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY];
  }

  VoronoiNode c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.isAdded = notAdded;
  c.isMarked = false;
  c.needsRaise = false;
  c.node_state = IN_OPEN_SET;
  c.parent = NULL; 
  //c.distanceCost = 0;
  //c.isParent = false;
  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) {
      c.x = x;
      c.y = y;
      data[x][y] = c;
    }
 
  if (initGridMap) {
    for (int x=0; x<sizeX; x++)
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0;
  }
}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);
  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y]) {
        VoronoiNode v = data[x][y];
	v.node_state = IN_OPEN_SET;
   	if (!isOccupied(x,y,v)) {
	  bool isSurrounded = true; 
	  for (int dx=-1; dx<=1; dx++) {
	    int nx = x+dx;
 	    if (nx<=0 || nx>=sizeX-1) continue;
	    for (int dy=-1; dy<=1; dy++) {
	      if (dx==0 && dy==0) continue;
	      int ny = y+dy;
	      if (ny<=0 || ny>=sizeY-1) continue;

	      if (!gridMap[nx][ny]) {
		isSurrounded = false;
		break;
	      }
	    }
	  }
	  if (isSurrounded) {
	    v.obstX = x;
	    v.obstY = y;
	    v.sqdist = 0;
	    v.dist=0;
	    v.voronoi=occupied;
	    v.queueing = fwProcessed;
	    data[x][y] = v;
	  } else setObstacle(x,y);
	}
      }
    }
  } 
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1;
  setObstacle(x,y);
}

void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0;
  removeObstacle(x,y);
}

void DynamicVoronoi::setObstacle(int x, int y) {
  //boost::unique_lock<boost::mutex>(mutex);
  VoronoiNode v = data[x][y];
  if(isOccupied(x,y,v)) return;
  addList.push_back(Eigen::Vector3d(x,y,0));
  v.obstX = x;
  v.obstY = y;
  data[x][y] = v;
}

void DynamicVoronoi::exchangeObstacles(const std::vector<Eigen::Vector3d>& points) {
  for (unsigned int i=0; i<lastObstacles.size(); i++) {
    int x = lastObstacles[i][0];
    int y = lastObstacles[i][1];

    bool v = gridMap[x][y];
    if (v) continue;
    removeObstacle(x,y);
  }
  
  lastObstacles.clear();
  lastObstacles.reserve(points.size());

  for (unsigned int i=0; i<points.size(); i++) {
    int x = points[i][0];
    int y = points[i][1];
    bool v = gridMap[x][y];
    if (v) continue;
    setObstacle(x, y);
    lastObstacles.push_back(points[i]);
  }
}

void DynamicVoronoi::update(bool updateRealDist) {
  commitAndColorize(updateRealDist);

  while(!open.empty()) {
    Eigen::Vector3d p = open.pop();
    int x = p[0];
    int y = p[1];
    VoronoiNode v = data[x][y];
    
    if (v.queueing == fwProcessed) continue;
    
    if (v.needsRaise) {
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
   	  if (dx==0 && dy==0) continue;
	  int ny = y+dy;
 	  if (ny<=0 || ny>=sizeY-1) continue;
	  VoronoiNode nc = data[nx][ny];
	  if (nc.obstX!=invalidObstData && !nc.needsRaise) {
	    if (!isOccupied(nc.obstX, nc.obstY, data[nc.obstX][nc.obstY])) {
	      open.push(nc.sqdist, Eigen::Vector3d(nx,ny,0));
	      nc.queueing = fwQueued;
	      nc.needsRaise = true;
	      nc.obstX = invalidObstData;
	      nc.obstY = invalidObstData;
	      if (updateRealDist) nc.dist = INFINITY;
	      nc.sqdist = INT_MAX;
	      data[nx][ny] = nc;
	    } else {
	      if (nc.queueing != fwQueued) {
  	        open.push(nc.sqdist, Eigen::Vector3d(nx,ny,0));
		nc.queueing = fwQueued;
		data[nx][ny] = nc;
	      }
	    }
          }
        }
      }
      v.needsRaise = false;
      v.queueing = bwProcessed;
      data[x][y] = v;
    }
    else if (v.obstX != invalidObstData && isOccupied(v.obstX, v.obstY, data[v.obstX][v.obstY])) {
      v.queueing = fwProcessed;
      v.voronoi = occupied;

      for (int dx=-1; dx<=1; dx++) {
	int nx = x+dx;
 	if (nx<=0 || nx>=sizeX-1) continue;
	for (int dy=-1; dy<=1; dy++) {
	  if (dx==0 && dy==0) continue;
	  int ny = y+dy;
	  if (ny<=0 || ny>=sizeY-1) continue;
	  VoronoiNode nc = data[nx][ny];
	  if (!nc.needsRaise) {
	    int distx = nx-v.obstX;
	    int disty = ny-v.obstY;
	    int newSqDistance = distx*distx + disty*disty;
	    bool overwrite = (newSqDistance < nc.sqdist);
	    if (!overwrite && newSqDistance==nc.sqdist) {
 	      if (nc.obstX == invalidObstData || isOccupied(nc.obstX, nc.obstY, data[nc.obstX][nc.obstY])==false) overwrite = true;
	    }
	    if (overwrite) {
	      open.push(newSqDistance, Eigen::Vector3d(nx,ny,0));	
	      nc.queueing = fwQueued;
	      if (updateRealDist) {
 	        nc.dist = sqrt((double) newSqDistance);
	      }
	      nc.sqdist = newSqDistance;
	      nc.obstX = v.obstX; 
	      nc.obstY = v.obstY;
	    } else {
		checkVoro(x,y,nx,ny,v,nc);
	    }
	    data[nx][ny] = nc;
	  }
        }
      }
    }
    data[x][y] = v;
  }
}   

float DynamicVoronoi::getDistance(int x, int y) const {
  if ((x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist;
  else return -INFINITY;
}

void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  for (unsigned int i=0; i<addList.size(); i++) {
    Eigen::Vector3d p = addList[i];
    int x = p[0];
    int y = p[1];
    VoronoiNode c = data[x][y];
   
    if (c.queueing != fwQueued) {
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;	
      c.obstY = y;	
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, Eigen::Vector3d(x,y,0));
    }
  }

  for (unsigned int i=0; i<removeList.size(); i++) {
    Eigen::Vector3d p = removeList[i];
    int x = p[0];
    int y = p[1];
    VoronoiNode c = data[x][y];
    
    if (isOccupied(x,y,c) == true) continue;
    open.push(0, Eigen::Vector3d(x,y,0));
    if (updateRealDist) c.dist = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}


void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, VoronoiNode& c, VoronoiNode& nc) {
  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) {
    if (abs(c.obstX-nc.obstX) > 1 || abs(c.obstY-nc.obstY) > 1) {
      int dxy_x = x-nc.obstX;
      int dxy_y = y-nc.obstY;
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist<0) return;
      
      int dnxy_x = nx - c.obstX;	
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist <0) return;

      if(stability_xy <= stability_nxy && c.sqdist>2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          reviveVoroNeighbors(x,y);
          pruneQueue.push(Eigen::Vector3d(x,y,0));
        }
      }
  
      if(stability_nxy <= stability_xy && nc.sqdist>2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx,ny);
          pruneQueue.push(Eigen::Vector3d(nx,ny,0));
        }
      }
    }
  }
}

void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  for (int dx=-1; dx<=1; dx++) {
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) continue;
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) continue;
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) continue;
      VoronoiNode nc = data[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        data[nx][ny] = nc;
        pruneQueue.push(Eigen::Vector3d(nx,ny,0));
      }
    }
  }
}
	
bool DynamicVoronoi::isOccupied(int x, int y) const {
  VoronoiNode c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, VoronoiNode &c) { 
  return (c.obstX==x && c.obstY==y);
}

void DynamicVoronoi::prune() {
  // filler
  while(!pruneQueue.empty()) {
    Eigen::Vector3d p = pruneQueue.front();
    pruneQueue.pop();
    int x = p[0];
    int y = p[1];

    if (data[x][y].voronoi==occupied) continue;
    if (data[x][y].voronoi==freeQueued) continue;

    data[x][y].voronoi = freeQueued;
    open.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    VoronoiNode tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    VoronoiNode r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) { 
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, Eigen::Vector3d(x+1,y,0));
        data[x+1][y] = r;
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, Eigen::Vector3d(x-1,y,0));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, Eigen::Vector3d(x,y+1,0));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, Eigen::Vector3d(x,y-1,0));
        data[x][y-1] = b;
      }
    } 
  }


  while(!open.empty()) {
    Eigen::Vector3d p = open.pop();
    VoronoiNode c = data[int(p[0])][int(p[1])];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(int(p[0]),int(p[1]));
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[int(p[0])][int(p[1])] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        Eigen::Vector3d p = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[int(p[0])][int(p[1])].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}

DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        VoronoiNode nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;
  


  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}


void DynamicVoronoi::brushFireMark(int x, int y) {
  //boost::unique_lock<boost::mutex>(mutex);
  std::queue<VoronoiNode*> brushFireNodes = std::queue<VoronoiNode*>();
  VoronoiNode *s = &data[x][y];
  brushFireNodes.push(s);
  while (!brushFireNodes.empty()) {
    s = brushFireNodes.front();
    brushFireNodes.pop();
    //x = s->x;
    //y = s->y;
    s->isMarked = true;
    //data[x][y].isMarked = true;
    for(int dx=-1; dx<=1; dx++) {
      int nx = x+dx;
      if (nx<0 || nx>=sizeX) continue;
      for (int dy=-1; dy<=1; dy++) {
        if (dx==0 && dy==0) continue;
        int ny = y+dy;
        if (ny<0 || ny>=sizeY) continue;
	//if (isOccupied(nx, ny)) continue;
        if (data[nx][ny].voronoi == occupied) continue;
        if (nx == x && ny == y) continue;
        if (!isVoronoi(nx, ny) && !data[nx][ny].isMarked) 
        {
          brushFireNodes.push(&data[nx][ny]);
        }
      }
    }
  }
}

}


