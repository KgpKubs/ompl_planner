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

void Callback(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
  // time(&t);
  gettimeofday(&t,NULL);
  currT = (long long)(t.tv_sec)*1000 + (long long)(t.tv_usec)/1000;
  pos p;
  count_++;

  v.clear();
  for(int i=0;i<msg->homePos.size();i++){
  	// p.x=fabs(msg->homePos[i].x+3300)/8.0;
    // p.y=fabs(msg->homePos[i].y+2200)/8.0;
    p.x=fabs(msg->homePos[i].x+4000)*13.0/160.0;
    p.y=fabs(msg->homePos[i].y+3000)*3.0/40.0;
  	//cout<<p.x<<" "<<p.y<<endl;
  	v.push_back(p);
  }

  for(int i=0;i<msg->awayPos.size();i++){
  	p.x=fabs(msg->awayPos[i].x+4000)*13.0/160.0;
  	p.y=fabs(msg->awayPos[i].y+3000)*3.0/40.0;
  	v.push_back(p);
  }

  Planning planning(v,v.size(), gui_msgs);
  // planning.planWithSimpleSetup();
  // planning.drawPath();
  // planning.output();

  planning.planSimple();


  // For Testing
  float endX = 2000;
  float endY = 1000;

  float scaledEndX = (endX + 4000)*13.0/160;
  float scaledEndY = (endY + 3000)*3.0/40;




  planning.plan(v[0].x, v[0].y, scaledEndX, scaledEndY);
  // planning.plan(v[0].x, v[0].y, 600, 500);
  path_points = planning.recordSolution();
  // planning.drw();
  points.point_array.clear();
   for(int i=0;i<path_points.size();i++)
    {
      point_.x=path_points[i].x;
      point_.y=path_points[i].y;
      // point_.x=path_points[i].x*600/800;
      // point_.y=path_points[i].y*400/500;
      points.point_array.push_back(point_);
    }   

  pub.publish(points); 
  gettimeofday(&t,NULL);
  currT = (long long)(t.tv_sec)*1000 + (long long)(t.tv_usec)/1000;
  // time(&t);

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
