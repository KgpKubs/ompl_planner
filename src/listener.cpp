/**
 * @file listener.cpp
 * @brief Communication Module for planner.
 * 
 * Node Name :- /listener
 * Suscribed to :- /beliefstate, /gui_params
 * Publish to :- /path_planner_ompl
 */

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
int count_=0;
bool replanCondition;
struct timeval t;
long long int currT;

std::vector<krssg_ssl_msgs::point_2d> path_points;
std::vector<krssg_ssl_msgs::point_2d> v;
krssg_ssl_msgs::planner_path points;
krssg_ssl_msgs::point_2d point_, initial_p, final_p;
ros::Publisher pub;
krssg_ssl_msgs::point_SF gui_msgs;

std::vector<krssg_ssl_msgs::point_2d> awayVel;
std::vector<krssg_ssl_msgs::point_2d> homeVel;

/**
 * @brief      Callback for gui msgs
 *
 * @param[in]  msg   msg from /gui_params
 * 
 * Step Size
 * Max_Iteration
 * Bias_Param
 */
void Callback_gui(const krssg_ssl_msgs::point_SF::ConstPtr& msg)
{
  // TODO
  // Link Gui with planner completely
  gui_msgs.step_size = msg->step_size;
  gui_msgs.max_iteration = msg->max_iteration;
  gui_msgs.bias_param = msg->bias_param;
}


/**
 * @brief      Replan Path
 *
 * @param[in]  awayBotSize  Number of opponent Bots
 *
 * @return     True if replanning is needed, False otherwise
 */
bool shouldReplan(int awayBotSize){
  int botid = 0;
  float startDeviationThres = 100;
  float endDeviationThres = 100;
  float velThresh = 50;
  float angleThres = 0.1;
  replanCondition = 0;
  if (path_points.size())
  {
    float distance = sqrt(pow(path_points[0].x - v[botid].x,2) + pow(path_points[0].y - v[botid].y,2));
    float velMagnitude = sqrt(pow(homeVel[botid].x,2) + pow(homeVel[botid].y,2));
    if (distance > startDeviationThres && velMagnitude < velThresh)
    {
      replanCondition = 1;
    }
  }
  else{
    replanCondition = 1;
  }
  for (int i = 0; i < awayBotSize; ++i)
  {
    float alignAngle = atan2(v[botid].y - v[awayBotSize + i].y,v[botid].x - v[awayBotSize + i].x);
    float awayMoveAngle = atan2(awayVel[i].y,awayVel[i].x);
    float angleDiff = abs(alignAngle - awayMoveAngle);
    if (angleDiff < angleThres)
    {
      replanCondition = 1;
    }
  }

  return replanCondition;
}

/**
 * @brief      BeliefState Callback
 *
 * @param[in]  msg   msg from /belief_state
 * 
 * Subscribe to BeliefState, Get start and end points and plan 
 * path accordingly
 *  
 * @see Planning
 * @see Planning::planSimple()
 * @see Planning::plan()
 * @see Planning::recordSolution()
 */
void Callback(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
  krssg_ssl_msgs::point_2d p;
  count_++;
  krssg_ssl_msgs::point_2d vel;
  v.clear();
  for(int i=0;i<msg->homePos.size();i++){
    p.x = msg->homePos[i].x*BS_TO_OMPL;
    p.y = msg->homePos[i].y*BS_TO_OMPL;
  	v.push_back(p);
    vel.x = msg->homeVel[i].x;
    vel.y = msg->homeVel[i].y;
    homeVel.push_back(vel);
  }

  for(int i=0;i<msg->awayPos.size();i++){
    p.x = msg->awayPos[i].x*BS_TO_OMPL;
    p.y = msg->awayPos[i].y*BS_TO_OMPL;
  	v.push_back(p);
    vel.x = msg->awayVel[i].x;
    vel.y = msg->awayVel[i].y;
    awayVel.push_back(vel);
  }

  float endX = 2000;
  float endY = 1000;

    if (points.point_array.size() == 0)
    {
      cout<<"Replanning "<<endl;
      Planning planning(v,v.size(), gui_msgs);
      planning.planSimple();
      planning.plan(v[0].x,v[0].y,endX*BS_TO_OMPL, endY*BS_TO_OMPL);
      path_points = planning.recordSolution();
      for(int i=0;i<path_points.size();i++)
      {
       point_.x=path_points[i].x*OMPL_TO_BS;
       point_.y=path_points[i].y*OMPL_TO_BS;
       points.point_array.push_back(point_);
      }
      cout<<"Publishing"<<endl;
    }
       
    pub.publish(points); 
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
