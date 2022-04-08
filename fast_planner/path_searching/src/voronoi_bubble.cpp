#include <path_searching/voronoi_bubble.h>
#define MAX 100000
using namespace std;
using namespace Eigen;

namespace fast_planner
{
VoronoiBubble::~VoronoiBubble()
{
  for (int i=0; i<allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

void VoronoiBubble::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new VoronoiNode;
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;
  std::cout<<"voronoibubble init 成功\n";
}

void VoronoiBubble::reset()
{
  path_nodes_.clear();
  std::priority_queue<VoronoiNode*, std::vector<VoronoiNode*>, NodeComparator1> empty_queue;
  open_set_.swap(empty_queue);
  step_nodes.clear();
  for (int i=0; i<use_node_num_; i++)
  {
    VoronoiNode* node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_  = false;
}

void VoronoiBubble::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

void VoronoiBubble::displayVoronoiNodes(vector<Eigen::Vector3d> list) {
    
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = 0;
  voronoi_nodes_pub_.publish(mk);
  
  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = 50;
  mk.color.g = 50;
  mk.color.b = 50;
  mk.color.a = 100;

  mk.scale.x = cellResolution;
  mk.scale.y = cellResolution;
  mk.scale.z = cellResolution;
  
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }

  voronoi_nodes_pub_.publish(mk);
  ros::Duration(0.001).sleep();
}

bool VoronoiBubble::reachHorizion(Eigen::Vector3d s, DynamicVoronoi& voronoiDiagram) {
  //std::cout<<"检查终点范围,originX="<<voronoiDiagram.originX<<",originY="<<voronoiDiagram.originY<<",sizeX="<<voronoiDiagram.sizeX<<",sizeY="<<voronoiDiagram.sizeY<<std::endl;
  //std::cout<<"终点为：x="<<s[0]<<",y="<<s[1]<<std::endl;
  bool reach_horizion = s[0] < voronoiDiagram.originX || s[0] > voronoiDiagram.originX+voronoiDiagram.sizeX
                           || s[1] < voronoiDiagram.originY || s[1] > voronoiDiagram.originY+voronoiDiagram.sizeY;
  return reach_horizion;
}

vector<double> VoronoiBubble::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

vector<double> VoronoiBubble::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

Eigen::Vector3d VoronoiBubble::getTmpEnd(Eigen::Vector3d start_pt_, Eigen::Vector3d end_pt_, DynamicVoronoi& voronoiDiagram) {
  std::cout<<"计算临时终点\n";
  Eigen::Vector3d tmp_end_ = end_pt_;
  bool reach_horizion = reachHorizion(end_pt_, voronoiDiagram);
  //if (reach_horizion) std::cout<<"再次检查超出终点\n";
  double xDiff = end_pt_[0] - start_pt_[0];
  double yDiff = end_pt_[1] - start_pt_[1];
  double proportion = yDiff / xDiff;
  double tmpX, tmpY;
  tmpX = end_pt_[0];
  tmpY = end_pt_[1];
  while(reach_horizion) {
    //std::cout<<"tmpX:"<<tmpX<<",tmpY:"<<tmpY<<std::endl;  
    tmpX -= xDiff/abs(xDiff);
    tmp_end_[0] = int(tmpX);
    tmpY -= proportion;
    tmp_end_[1] = int(tmpY);
    //if(voronoiDiagram.isOccupied(int(tmp_end_[0]) , int(tmp_end_[1]))) continue;
    reach_horizion = reachHorizion(tmp_end_, voronoiDiagram);
  }
  return tmp_end_;
}

int VoronoiBubble::search2(Eigen::Vector3d start_pt_, Eigen::Vector3d start_v_, Eigen::Vector3d start_a_,  Eigen::Vector3d end_pt_, Eigen::Vector3d end_v_, DynamicVoronoi diagram, bool init)
{
  bool init_search = init;
  std::cout<<"**************search2**********\n";
  int startVoronoiX = ceil(start_pt_[0] - diagram.originX);
  int startVoronoiY = ceil(start_pt_[1] - diagram.originY);
  int endVoronoiX   = ceil(end_pt_[0]   - diagram.originX);
  int endVoronoiY   = ceil(end_pt_[1]   - diagram.originY);
  std::cout<<"sizeX="<<diagram.sizeX<<",sizeY="<<diagram.sizeY<<std::endl;
  std::cout<<"originX="<<diagram.originX<<",originY="<<diagram.originY<<std::endl;
  std::cout<<"起点为("<<start_pt_[0]<<","<<start_pt_[1]<<")\n";
  std::cout<<"终点为("<<end_pt_[0]<<","<<end_pt_[1]<<")\n";
  /*if (diagram.isOccupied(endVoronoiX, endVoronoiY))
  {
     std::cout<<"终点为障碍\n";
     return NO_PATH;
  }*/
  VoronoiNode* start = &diagram.data[startVoronoiX][startVoronoiY];
  VoronoiNode* end = &diagram.data[endVoronoiX][endVoronoiY];
  start->parent = NULL;
  start->node_state = IN_OPEN_SET;
  start->state.head(3) = start_pt_;
  start->state.tail(3) = start_v_;
  end->state.head(3)   = end_pt_;
  end->state.tail(3)   = end_v_;
  start->node_state = IN_OPEN_SET;
  end->node_state   = IN_OPEN_SET;
  start->g_score = 0;
  start->updateH(*end);
  //open_set_.push(start);
  //open_set_.push_back(start);
  VoronoiNode* s;
  int iterations = 0;
  while (!open_set_.empty())
  {
    iterations ++;
    std::cout<<"iterations"<<iterations<<" ";
    if (iterations > 1000)
    {
      std::cout<<"迭代到头了\n";  
      return NO_PATH;
    }
    //s = open_set_.top();
    //s = open_set_.front();
    if (fabs((s->state.head(2) - end_pt_.head(2)).norm()) <= 0.5)
    {
       std::cout<<"find path\n";
       start->parent = NULL;
       retrievePath(s);
       std::cout<<"retrieve 成功\n";
       return REACH_END;
    }
    //open_set_.pop();
    //open_set_.pop_front();
    if (!init_search)
      s->node_state = IN_CLOSE_SET;
    double res = 1/10.0, time_res = 1/20.0, time_res_init = 1/20.0;  
    Eigen::Matrix<double, 6, 1> cur_state = s->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    Eigen::Vector3d um;
    double pro_t;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    if (init_search)
    {
      inputs.push_back(start_a_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_+1e-3; tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
    }
    else
    {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;
            inputs.push_back(um);
          }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }
   double COST = INT_MAX;
   VoronoiNode* next_node;
   for (int i=0; i<inputs.size(); ++i)
     for (int j=0; j<durations.size(); ++j)
     {
       um = inputs[i];
       double tau = durations[j];
       stateTransit(cur_state, pro_state, um, tau);
       //pro_t = s->time + tau;
       Eigen::Vector3d pro_pos = pro_state.head(3);
       Eigen::Vector3d pro_v   = pro_state.tail(3);
       int voronoiX = ceil(pro_pos(0) - diagram.originX);
       int voronoiY = ceil(pro_pos(1) - diagram.originY);
       if (voronoiX <0 || voronoiX >= diagram.sizeX || voronoiY < 0 || voronoiY >= diagram.sizeY) continue;
       if (diagram.isOccupied(voronoiX, voronoiY) && !init_search) continue;
       if (diagram.data[voronoiX][voronoiY].node_state == IN_CLOSE_SET) continue;
       if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_) continue;
       VoronoiNode* pro_node = &diagram.data[voronoiX][voronoiY];
       pro_node->duration = tau;
       pro_node->input = inputs[i];
       pro_node->state = pro_state;
       double cost = calcCost(s, pro_node, *end);
       if (cost < COST) 
       {
         next_node = pro_node;
	 COST = cost;
       }
     }
  init_search = false;
  //std::cout<<next_node->state.transpose()<<std::endl;
  next_node->parent = s;
  //open_set_.push(next_node);
 // open_set_.push_back(next_node);
  }
  return NO_PATH;
}

double VoronoiBubble::calcCost(VoronoiNode* cur_node, VoronoiNode* pro_node, VoronoiNode goal)
{
  pro_node->g_score = cur_node->g_score;
  pro_node->updateG(*cur_node);
  double time_to_goal; 
  pro_node->updateH(goal);
  //pro_node->f_score = lambda_heu_ * abs(estimateHeuristic(pro_node->state, goal.state, time_to_goal));
  pro_node->parent = NULL;
  return pro_node->getC();
}

double VoronoiBubble::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

double VoronoiBubble::movementCost(Eigen::Vector3d state1, Eigen::Vector3d state2)
{
  return sqrt((state1[0]-state2[0])*(state1[0]-state2[0])+(state1[1]-state2[1])*(state1[1]-state2[1]));
} 

VoronoiNode* VoronoiBubble::getClosestVoro(Eigen::Vector3d point, Eigen::Vector3d end)
{
  int x,y;
  double COST = MAX;
  double DIS  = MAX;
  for(int i=0; i<diagram.freeNodes.size(); i++)
  {
    VoronoiNode node = diagram.freeNodes[i];
    if(diagram.data[node.x][node.y].node_state == IN_CLOSE_SET) continue; 
    double stateX = node.x*cellResolution + diagram.originX;
    double stateY = node.y*cellResolution + diagram.originY;
    double tempDis = sqrt(pow(stateX-point[0], 2)+pow(stateY-point[1], 2));
    //double cost    = sqrt(pow(stateX-end[0], 2)+pow(stateY-end[1], 2));
    if(tempDis<DIS) {DIS = tempDis; x = node.x; y = node.y;}
  }
  return &diagram.data[x][y];
}

int VoronoiBubble::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a, Eigen::Vector3d end_pt, Eigen::Vector3d end_v, DynamicVoronoi voronoiDiagram)
{
  std::list<VoronoiNode*> open_set;
  int sizeX = voronoiDiagram.sizeX;
  int sizeY = voronoiDiagram.sizeY;
  double originX = voronoiDiagram.originX;
  double originY = voronoiDiagram.originY;
  std::cout<<"起点:"<<start_pt.transpose().head(2)<<",终点:"<<end_pt.transpose().head(2)<<std::endl;
  std::cout<<"sizeX="<<sizeX<<",sizeY="<<sizeY<<std::endl;
  std::cout<<"originX="<<originX<<",originY="<<originY<<std::endl;
  std::cout<<"xMax="<<(originX+sizeX*cellResolution)<<std::endl;
  open_set_.empty();
  start_vel_ = start_v;
  start_acc_ = start_a;
  Eigen::VectorXd end_state(6);
  end_state.head(3) = end_pt;
  end_state[2] = 0;
  end_state.tail(3) = end_v;
  
  //VoronoiNode *end = &voronoiDiagram.data[endVoronoiX][endVoronoiY];
  //VoronoiNode* end = new VoronoiNode();
  VoronoiNode* end = new VoronoiNode();
  end->state.head(3) = end_pt;
  end->state.tail(3) = end_v;

  double time_to_goal;
  int startVoronoiX = ceil((start_pt[0]-voronoiDiagram.originX)/cellResolution);
  int startVoronoiY = ceil((start_pt[1]-voronoiDiagram.originY)/cellResolution);
  
  int endVoronoiX = ceil((end_pt[0]-voronoiDiagram.originX)/cellResolution);
  int endVoronoiY = ceil((end_pt[1]-voronoiDiagram.originY)/cellResolution);

  std::cout<<"起点维诺图位置：("<<startVoronoiX<<","<<startVoronoiY<<")\n";
  std::cout<<"终点维诺图位置：("<<endVoronoiX<<","<<endVoronoiY<<")\n";
  voronoiDiagram.setObstacle(startVoronoiX, startVoronoiY);
  voronoiDiagram.setObstacle(endVoronoiX, endVoronoiY);
  voronoiDiagram.update();
  voronoiDiagram.brushFireMark(startVoronoiX, startVoronoiY);
  voronoiDiagram.brushFireMark(endVoronoiX, endVoronoiY);
  voronoiDiagram.visualize();
  diagram = voronoiDiagram;
  std::cout<<"set obstacle & update successfully\n";
  
  VoronoiNode* start = new VoronoiNode();
  start->state.head(3) = start_pt;
  start->state.tail(3) = start_v;
  start->input = start_a;
  start->g_score = 0.0;
  start->f_score = movementCost((*start).state.head(3), end_pt);
  start->x = startVoronoiX;
  start->y = startVoronoiY;
  start->parent = NULL;


  VoronoiNode* cur_node  = getClosestVoro(start_pt, end_pt);
  std::cout<<"最近的起点位置("<<cur_node->x<<","<<cur_node->y<<")\n";
  if(cur_node->voronoi == free || cur_node->voronoi == voronoiKeep) std::cout<<"起点可行\n"; 
  else std::cout<<"起点不可行\n";
  cur_node->parent = start;
  cur_node->state.head(3) = start->state.head(3) + Eigen::Vector3d((cur_node->x-startVoronoiX)*cellResolution, (cur_node->y-startVoronoiY)*cellResolution, 0);
  cur_node->state.tail(3) = start_v;
  cur_node->g_score = movementCost((*cur_node).state.head(3), start_pt);;
  cur_node->f_score = movementCost((*cur_node).state.head(3), end_pt);
  getDynamicInfo((*start).state, cur_node->state,0.5, cur_node->input);
  open_set_.push(cur_node);
 
  VoronoiNode* last_node = getClosestVoro(end_pt, end_pt);
  
  std::cout<<"最近的终点位置("<<last_node->x<<","<<last_node->y<<")\n";
  if(voronoiDiagram.isVoronoi(last_node->x, last_node->y)) std::cout<<"终点可行\n";
  else std::cout<<"终点不可行\n";
  last_node->node_state = IN_OPEN_SET;
  const double tolerance = 1;
  while (!open_set_.empty()) {
    Eigen::Vector3d um;
    cur_node = open_set_.top();
    //cur_node = open_set.front();
    std::cout<<"现在的位置为:("<<cur_node->x<<","<<cur_node->y<<")  "<<cur_node->state.transpose()<<std::endl;
    bool reach_horizon = cur_node->x >= voronoiDiagram.sizeX || cur_node->x <0 || cur_node->y >= voronoiDiagram.sizeY || cur_node->y <0;
    //bool near_end = fabs((cur_node->state.head(2) - end_pt.head(2)).norm()) < 0.2 || cur_node->x==endVoronoiX && cur_node->y==endVoronoiY;
    bool near_end = fabs(cur_node->x - last_node->x)<=2 && (cur_node->y - last_node->y)<=2;
    if (reach_horizon || near_end)
    {
      end->parent = cur_node;
      end->state.head(3) = cur_node->state.head(3)+Eigen::Vector3d((end->x-cur_node->x)*cellResolution, (end->y-cur_node->y)*cellResolution, 0);
      retrievePath(cur_node);
      step();
    }
    if (reach_horizon)
    {
      if (is_shot_succ_)	
      {
        std::cout<<"reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        std::cout<<"reach horizon" << std::endl;
	return REACH_HORIZON;
      }
    }
  
    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else //if (cur_node->parent != NULL)
      {
	std::cout << "near end" << std::endl;
	return NEAR_END;
      }
      /*else 
      {
        std::cout << "no path" << std::endl;
        return NO_PATH;
      }*/
    }
    open_set_.pop();
    //open_set_.pop();
    voronoiDiagram.data[cur_node->x][cur_node->y].node_state = IN_CLOSE_SET;
    //cur_node->node_state = IN_CLOSE_SET;

    int x = cur_node->x;
    int y = cur_node->y;
    int nx, ny;
    VoronoiNode* n;
    double cost = MAX;
    int number=0;
    int step =3;
    while(number==0)
    {
      for (int dy=-step; dy<=step; dy+=1) {
      int ny = y+dy;
      if(ny<0 || ny>=sizeY) continue;
      for (int dx=-step; dx<=step; dx+=1) {
        if (dx==0 && dy==0) continue;
        int nx = x+dx;
        if (nx<0 || ny>=sizeX) continue;
        VoronoiNode* next = &voronoiDiagram.data[nx][ny];
        //if (voronoiDiagram.data[nx][ny].node_state == IN_CLOSE_SET) continue;
        if(voronoiDiagram.data[nx][ny].node_state == IN_CLOSE_SET) continue;
        //if (voronoiDiagram.isVoronoi(nx, ny) || voronoiDiagram.data[nx][ny].isMarked) 
        if(next->isMarked || next->voronoi != occupied || next->voronoi == voronoiKeep || next->voronoi == free || nx==last_node->x && ny==last_node->y)
 	{
          //VoronoiNode* next = new VoronoiNode();
          next->parent = NULL;
          next->x = nx;
          next->y = ny;
          next->state(0) = cur_node->state(0)+dx*cellResolution;
          next->state(1) = cur_node->state(1)+dy*cellResolution;
	  next->state(2) = cur_node->state(2);
    	  next->g_score = cur_node->g_score+movementCost((*next).state.head(3), (*cur_node).state.head(3));
	  next->f_score = movementCost((*next).state.head(3), end_pt);
          //std::cout<<"   子节点:"<<next->state.transpose().head(2)<<",维诺图坐标为("<<nx<<","<<ny<<"), 最优的代价为:"<<cost<<",目前的代价为:"<<next->getC()<<std::endl;
	  Eigen::Vector3d state = next->state.head(3);
	  double distance = edt_environment_->evaluateCoarseEDT(state, -1.0);
	  double distanceCost = 1.0/distance;
  	  number++;
 	  if (2*next->f_score+next->g_score < cost)
          {
            cost=2*next->f_score+next->g_score;
	    n = next;
          }
        }//else{std::cout<<"    ("<<nx<<","<<ny<<")被占据\n";}
      }

      }
     step++;
    }   
    //if(number == 0){std::cout<<"*******开始找寻最近的维诺点*******\n"; n = getClosestVoro(cur_node->state.head(3),end_pt);}
    //std::cout<<"得到的下一个点是("<<n->x<<","<<n->y<<")\n"; 
    std::cout<<"找到点：("<<n->state(0)<<","<<n->state(1)<<") " ;
    double res = 1/20.0, time_res = 1/10.0, time_res_init = 1/20.0;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;   
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    n->parent = cur_node;
    open_set_.push(n);
  }
  return NO_PATH;
}

void VoronoiBubble::retrievePath(VoronoiNode* end_node) 
{
  VoronoiNode* cur_node = end_node;
  std::cout<<"    retrievePath:"<<cur_node->state.transpose()<<std::endl;
  path_nodes_.push_back(cur_node);
  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    std::cout<<"    retrievePath————位置和速度："<<cur_node->state.transpose()<<"，加速度："<<cur_node->input.transpose()<<std::endl;
    path_nodes_.push_back(cur_node);
  }
  reverse(path_nodes_.begin(), path_nodes_.end());
}

void VoronoiBubble::step()
{
  step_nodes = std::vector<VoronoiNode>();
  std::cout<<"进入跳点过程，初始一共"<<path_nodes_.size()<<"个点\n";
  double T_sum = 0.0;
  int curIndex=0, nextIndex=1;
  VoronoiNode* curNode = path_nodes_[curIndex];
  VoronoiNode* nextNode = path_nodes_[nextIndex];
  Eigen::Vector3d curState  = curNode->state.head(3);
  Eigen::Vector3d nextState = nextNode->state.head(3);
  step_nodes.push_back(*curNode);
  //std::cout<<"    现在的位置:("<<curNode->x<<","<<curNode->y<<")\n";
  while(nextIndex < path_nodes_.size()) 
  {
    std::cout<<"    现在的位置:("<<curNode->x<<","<<curNode->y<<")\n";
    while(edt_environment_->lineCollision(curState, nextState)>dis_thresh && nextIndex+1<path_nodes_.size())
    {
      nextIndex++;
      nextNode = path_nodes_[nextIndex];
      nextState = nextNode->state.head(3);
    }
    //nextIndex--;
    //nextNode = path_nodes_[nextIndex];
    //nextState = nextNode->state.head(3);
    double duration = (nextIndex - curIndex) * time_duration_;
    nextNode->duration = duration;
    getDynamicInfo((*curNode).state, nextNode->state, duration, nextNode->input);
    step_nodes.push_back(*nextNode);
    curIndex=nextIndex;
    curNode  = path_nodes_[curIndex];
    curState = curNode->state.head(3);
    nextIndex++;
  }
}
 

std::vector<Eigen::Vector3d> VoronoiBubble::getVoronoiTraj(double delta_t)
{
  vector<Eigen::Vector3d> state_list ;//= vector<Eigen::Vector3d>();
  std::cout<<"进入getVoronoiTraj, 初始点数为"<<step_nodes.size()<<std::endl;
  VoronoiNode node;
  Matrix<double, 6, 1> x0, xt;
  int iterations=0;
  for(int i=0;i<step_nodes.size()-1;i++)
  {
    node = step_nodes[i];
    Vector3d ut = step_nodes[i+1].input;
    //std::cout<<"加速度为:"<<ut.transpose()<<std::endl;
    double duration = node.duration;
    x0 = node.state;
    //std::cout<<x0.transpose()<<std::endl;
    for(double t=0; t<=duration; t+=delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
  }
  return state_list;
}

bool VoronoiBubble::iSstateTransitable(Eigen::Matrix<double, 6, 1> state0, Eigen::Matrix<double, 6, 1> state1, Eigen::Vector3d um, double tau) 
{
  const double tolerance = 1/resolution_;
  double result = false;
  double xVel = state0[3];
  double yVel = state0[4];
  double x = state0[0] + xVel*tau + 1/2*um[0]*pow(tau, 2);
  double y = state0[1] + yVel*tau + 1/2*um[1]*pow(tau, 2);
  if (abs(x-state1[0]) <=0.5 && abs(y-state1[1]) <= 0.5)
  {
    result = true;
    /*double tauX = -xVel + sqrt(pow(xVel,2) - 2*um[0]*(state0[0]-state1[0]));
    double tauY = -yVel + sqrt(pow(yVel,2) - 2*um[1]*(state0[1]-state1[1]));
    tau = tauX > tauY ? tauX : tauY;*/
  }
  return result;
}

void VoronoiBubble::getDynamicInfo(Eigen::Matrix<double, 6, 1> state0, Eigen::Matrix<double, 6, 1>& state1, double tau, Eigen::Vector3d& input)
{
  double xDiff = state1[0] - state0[0];
  double yDiff = state1[1] - state0[1];
  double vx0 = state0[3];
  double vy0 = state0[4];
  double ax = (xDiff - vx0*tau)/pow(tau, 2) * 2;
  double ay = (yDiff - vy0*tau)/pow(tau, 2) * 2;
  //double ax = 2*(state1[0] - state0[0]*tau)/pow(tau,2);
  //double ay = 2*(state1[1] - state0[1]*tau)/pow(tau,2);
  state1[3] = vx0 + ax*tau;
  state1[4] = vy0 + ay*tau;
  input[0] = ax;
  input[1] = ay;
  //std::cout<<"    速度为:"<<state1[3]<<","<<state1[4]<<",加速度为:"<<input[0]<<","<<input[1]<<std::endl;
}

void VoronoiBubble::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um, double tau)
{
  for(int i=0; i<3; i++)
    phi_(i, i+3) = tau;
  um[2] = 0;
  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5*pow(tau, 2) * um;
  integral.tail(3) = tau * um;
  state1 = phi_ * state0 + integral;
}
  /*
  int xDiff = curNode.state[0] - nextNode.state[0]; 
  int yDiff = curNode.state[1] - nextNode.state[1]; 
  int zDiff = curNode.state[2] - nextNode.state[2]; 
  double xV = curNode.state[3];
  double yV = curNode.state[4];
  double zV = curNode.state(5);
  double tauX = (-xV+sqrt(pow(xV,2)-2*max_acc_*xDiff)) / max_acc_;
  double tauY = (-yV+sqrt(pow(yV,2)-2*max_acc_*yDiff)) / max_acc_;
  double tau;
  if (tauX > tauY) 
  {
    tau = tauX;
    double ay = 2*(yDiff - yV*tau) / pow(tau, 2);
    nextNode.input[0] = max_acc_;
    nextNode.input[1] = ay;
    nextNode.state[3] = curNode.state[3] + tau*max_acc_;
    nextNode.state[4] = curNode.state[4] + tau*ay;
  } else
  {
    tau = tauY;
    double ax = 2*(xDiff - xV*tau) / pow(tau, 2);
    nextNode.input[0] = ax;
    nextNode.input[1] = max_acc_;
    nextNode.state[3] = curNode.state[3] + tau*ax;
    nextNode.state[4] = curNode.state[4] + tau*max_acc_;
  }*/

VoronoiBubble::VoronoiBubble(ros::NodeHandle& nh)
{
  node = nh;
  nh.param("search/max_tau", max_tau_, -1.0);
  nh.param("search/init_max_tau", init_max_tau_, -1.0);
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/w_time", w_time_, -1.0);
  nh.param("search/horizon", horizon_, -1.0);
  nh.param("search/resolution_astar", resolution_, -1.0);
  nh.param("search/time_resolution", time_resolution_, -1.0);
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/allocate_num", allocate_num_, -1);
  nh.param("search/check_num", check_num_, -1);
  nh.param("search/optimistic", optimistic_, true);
  nh.param("search/duration", time_duration_, 0.1);
  nh.param("search/cellResolution", cellResolution, 0.2);
  nh.param("search/dis_thresh", dis_thresh, 1.0);
  tie_breaker_ = 1.0 + 1.0 / 10000;
  voronoi_nodes_pub_ = node.advertise<visualization_msgs::Marker>("/vis/voronoi_nodes", 20);
  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);
  max_vel_ += vel_margin;
} 



bool VoronoiBubble::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3,3);
  const Vector3d v1 = state2.segment(3,3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3,4);
  end_vel_ = v1;
  
  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;
  
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;
  
  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;
  
  Eigen::MatrixXd Tm(4,4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;
  
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
        coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
    {
      return false;
    }

    // if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
    //   return false;
    // }
    if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == 1)
    {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

void VoronoiBubble::getSamples(double& ts, vector<Eigen::Vector3d>& point_set, 
			       vector<Eigen::Vector3d>& start_end_derivatives)
{
  std::cout<<"进入getSamples\n";
  VoronoiNode node;
  for (int i=0; i<step_nodes.size()-1; i++)
  {
    node =  step_nodes[i];
    Eigen::Matrix<double, 6, 1> x0 = node.state;
    Eigen::Matrix<double, 6, 1> xt;
    Vector3d ut = step_nodes[i+1].input;
    std::cout<<"    加速度为:"<<ut.head(2).transpose()<<std::endl;
    for(double t=1e-5; t<=step_nodes[i+1].duration; t+=ts)
    {
      stateTransit(x0, xt, ut, t);
      std::cout<<"  "<<xt.transpose().head(3)<<std::endl;
      point_set.push_back(xt.head(3));
    } 
  } 
  node = step_nodes[step_nodes.size()-1]; 
//Eigen::Vector3d start_acc = node->input;*/
  Eigen::Vector3d start_acc = step_nodes[0].input;
  Eigen::Vector3d end_acc = step_nodes[step_nodes.size()-1].input;
  Eigen::Vector3d end_vel = step_nodes[step_nodes.size()-1].state.tail(3);

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
} 

}
