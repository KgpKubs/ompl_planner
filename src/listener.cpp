#include "ros/ros.h"
#include "std_msgs/String.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
#include <iostream>
#include "MotionPlanner.h"
#include <bits/stdc++.h>
#include <time.h>
using namespace std;
// using namespace cv;
int count_=0;
bool replanCondition;
// ########################### Debug
// time_t t;
struct timeval t;
long long int currT;
// ##########################


std::vector<point> path_points;
std::vector<pos> v;
krssg_ssl_msgs::planner_path points;
krssg_ssl_msgs::point_2d point_;
ros::Publisher pub;
gui_msg gui_msgs={0.5, 1000, 100, 3};

void Callback_gui(const krssg_ssl_msgs::point_SF::ConstPtr& msg)
{
  // cout<<" in Callback_gui , planner_selector = "<<msg->max_iteration<<endl;
  gui_msgs.stepSize = msg->step_size;
  gui_msgs.maxIteration = msg->max_iteration;
  gui_msgs.biasParam = msg->bias_param;
  gui_msgs.planner_selector = msg->max_iteration;;
}

bool shouldReplan(){
  int botid = 0;
  float startDeviationThres = 100;
  float endDeviationThres = 100;
  replanCondition = 0;
  cout<<path_points.size()<<endl;
  if (path_points.size())
  {
    float distance = sqrt(pow(path_points[0].x - v[botid].x,2) + pow(path_points[0].y - v[botid].y,2));
    if (distance > startDeviationThres)
    {
      replanCondition = 1;
    }
  }
  else{
    replanCondition = 1;
  }
  cout<<replanCondition<<endl;
  return replanCondition;
  // TODO---->Complete Function
}
void Callback(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
  gettimeofday(&t,NULL);
  currT = (long long)(t.tv_sec)*1000 + (long long)(t.tv_usec)/1000;
  pos p;
  count_++;

  v.clear();
  for(int i=0;i<msg->homePos.size();i++){
    p.x=fabs(msg->homePos[i].x+4000)*13.0/160.0;
    p.y=fabs(msg->homePos[i].y+3000)*3.0/40.0;
  	v.push_back(p);
  }

  for(int i=0;i<msg->awayPos.size();i++){
  	p.x=fabs(msg->awayPos[i].x+4000)*13.0/160.0;
  	p.y=fabs(msg->awayPos[i].y+3000)*3.0/40.0;
  	v.push_back(p);
  }

  if (shouldReplan() == 1)
  {
  Planning planning(v,v.size(), gui_msgs);
  planning.planSimple();


  // For Testing
  float endX = 2000;
  float endY = 1000;

  float scaledEndX = (endX + 4000)*13.0/160;
  float scaledEndY = (endY + 3000)*3.0/40;
  planning.plan(v[0].x, v[0].y, scaledEndX, scaledEndY);
  path_points = planning.recordSolution();
  points.point_array.clear();
   for(int i=0;i<path_points.size();i++)
    {
      point_.x=path_points[i].x;
      point_.y=path_points[i].y;
      points.point_array.push_back(point_);
    }   
  // int shouldReplan = 1;
  
    pub.publish(points); 
  }
  gettimeofday(&t,NULL);
  currT = (long long)(t.tv_sec)*1000 + (long long)(t.tv_usec)/1000;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/belief_state", 1000, Callback);
  ros::Subscriber sub1 = n.subscribe("/gui_params", 1000, Callback_gui);
  pub = n.advertise<krssg_ssl_msgs::planner_path>("/path_planner_ompl", 1000);
  ros::spin();
  return 0;
}
