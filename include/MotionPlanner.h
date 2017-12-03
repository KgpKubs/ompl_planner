#ifndef _MOTION_PLANNER_H_
#define _MOTION_PLANNER_H_

#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/planner_path.h"
#include "krssg_ssl_msgs/point_2d.h"
#include "krssg_ssl_msgs/point_SF.h"
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <boost/bind.hpp>
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/PlannerData.h>
#include <cmath>
#include <iostream>
#include <ostream>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

#define BS_TO_OMPL 0.1
#define OMPL_TO_BS 10
#define HALF_FIELD_MAXX 4000
#define HALF_FIELD_MAXY 3000
#define HALF_FIELD_MAXX_OMPL HALF_FIELD_MAXX*BS_TO_OMPL
#define HALF_FIELD_MAXY_OMPL HALF_FIELD_MAXY*BS_TO_OMPL
#define radius 150*BS_TO_OMPL

template<typename T>
boost::shared_ptr<T> make_shared_ptr(std::shared_ptr<T>& ptr)
{
    return boost::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

template<typename T>
std::shared_ptr<T> make_shared_ptr(boost::shared_ptr<T>& ptr)
{
    return std::shared_ptr<T>(ptr.get(), [ptr](T*) mutable {ptr.reset();});
}

class Planning{
  public:
    Planning(std::vector<krssg_ssl_msgs::point_2d> &v,int n, krssg_ssl_msgs::point_SF gui_msgs);
    void init(std::vector<krssg_ssl_msgs::point_2d> &v,int n, krssg_ssl_msgs::point_SF gui_msgs);
  
    bool isStateValid(const ob::State *state) const;
    void planWithSimpleSetup();
    void drawPath();
    void output();
    void planSimple();
    bool plan(int start_row, int start_col, int goal_row, int goal_col);
    vector<krssg_ssl_msgs::point_2d> recordSolution();
    bool isStateValid1(const ob::State *state);
    void drw();

  private:
    double* xc;
    double* yc;
    double* r;
    // Number of obstacles in space.
    int numObstacles;
    // Start position in space
    double xStart;
    double yStart;
    // Goal position in space
    double xGoal;
    double yGoal;
    // Max. distance toward each sampled position we
    // should grow our tree
    double stepSize;
    // Boundaries of the space
    double xLeft;
    double xRight;
    double yTop;
    double yBottom;
    int selector;


    int maxWidth_;
    int maxHeight_;
    og::SimpleSetupPtr ss_;

};
#endif
