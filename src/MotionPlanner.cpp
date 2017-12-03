#include "MotionPlanner.h"

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

#include <boost/make_shared.hpp>

#define radius 10
int windowSize = 50;  //even for simplifyPoint Function
using namespace cv;
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;


Mat img(800,800,CV_8UC3,Scalar(0,255,0));

std::vector<point> vect,vect1;
std::vector<point> publishingPoint;
Planning::Planning(vector<pos> &v,int n, gui_msg gui_msgs){
  init(v,n,gui_msgs);
  //CreateCircle();
}

void simplifyPoint(std::vector<point>& v){
  std::vector<point> newPoints;
  for (int i = 0; i < windowSize/2; ++i)
  {
    newPoints.push_back(v[i]);
  }
  for (int i = windowSize/2; i < v.size() - windowSize/2; ++i)
  {
    float X = 0;
    float Y = 0;
    for (int j = i - windowSize/2; j <= i + windowSize/2; ++j)
    {
      X += v[j].x;
      Y += v[j].y;
    }
    point p;
    p.x = X/(windowSize + 1);
    p.y = Y/(windowSize + 1);
    newPoints.push_back(p);
  }
  float deltaX = (newPoints[windowSize/2].x - newPoints[0].x)*2/windowSize;
  float deltaY = (newPoints[windowSize/2].y - newPoints[0].y)*2/windowSize;
  for (int i = 0; i < windowSize/2; ++i)
  {
    newPoints[i].x = newPoints[0].x + i*deltaX;
    newPoints[i].y = newPoints[0].y + i*deltaY;
  }
  deltaY = (v[v.size() - 1].y - newPoints[v.size() - windowSize/2 - 1].y)*2/windowSize;
  deltaX = (v[v.size() - 1].x - newPoints[v.size() - windowSize/2 - 1].x)*2/windowSize;
  for (int i = 0; i < windowSize/2; ++i)
  {
    point p;
    p.x = newPoints[v.size() - windowSize/2 - 1].x + (i+1)*deltaX;
    p.y = newPoints[v.size() - windowSize/2 - 1].y + (i+1)*deltaY; 
    newPoints.push_back(p);
  }
  v = newPoints;
}

void simplifyWindow(vector<point> & v){
  std::vector<point> newPoints;
  cout<<v.size()<<endl;
  for (int i = 0; i <= v.size(); i += windowSize)
  {
    int lastPtr = i + windowSize - 1;
    if (lastPtr > v.size() - 1)
    {
      lastPtr = v.size() - 1;
    }
    int currentWindowSize = lastPtr - i + 1;
    float deltaX = (v[lastPtr].x - v[i].x)/currentWindowSize;
    float deltaY = (v[lastPtr].y - v[i].y)/currentWindowSize;
    for (int j = i; j <= lastPtr; j++)
    {
      point p;
      p.x = v[i].x + (j - i + 1)*deltaX;
      p.y = v[i].y + (j - i + 1)*deltaY;
      newPoints.push_back(p);
    }
    cout<<"---";
  }
  v = newPoints;
  cout<<v.size()<<endl;
}

void Planning::init(vector<pos> &v,int n, gui_msg gui_msgs)
{

  xStart=v[0].x;
  yStart=v[0].y;
  // Goal position in space
  xGoal=800.0;
  yGoal=500.0;
  // Max. distance toward each sampled position we
  // should grow our tree
  stepSize=gui_msgs.stepSize;
  // Boundaries of the space
  xLeft=0.0;
  xRight=800.0;
  yTop=800.0;
  yBottom=0.0;
  selector=gui_msgs.planner_selector;

  n--;
  numObstacles = n;
  xc = new double[numObstacles];
  yc = new double[numObstacles];

  for(int i=0;i<n;i++){
    xc[i] = v[i+1].x;
    yc[i] = v[i+1].y;
  }

  // for(int i = 0; i < numObstacles; ++i){
  //   printf("Circle %d : [x,y]--[%5.2lf, %5.2lf] radius--[%5.2lf]\n", i+1, xc[i], yc[i],30.0);
  // }

  // printf("\nStart [%5.2lf, %5.2lf]\n", xStart, yStart);
  // printf("End [%5.2lf, %5.2lf]\n\n", xGoal, yGoal);

}


void Planning::CreateCircle(){
  for(int i=0;i<numObstacles;i++){
    circle(img,Point(xc[i],yc[i]),10.0,Scalar(250,128,114),CV_FILLED,8,0);
  }
}


bool Planning::isStateValid1(const ob::State *state){
  const ob::SE2StateSpace::StateType *state_2d= state->as<ob::SE2StateSpace::StateType>();
  const double &x(state_2d->getX()), &y(state_2d->getY());

  for (int i = 0; i < numObstacles; ++i){
    if (sqrt(pow((xc[i]-x),2)+pow((yc[i]-y),2))<=10.0){
      return false;
    }
  }

  return true;
}


bool Planning::plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col){
  if (!ss_)
    return false;
  ob::ScopedState<> start(ss_->getStateSpace());
  start[0] = start_row;
  start[1] = start_col;
  ob::ScopedState<> goal(ss_->getStateSpace());
  goal[0] = goal_row;
  goal[1] = goal_col;
  ss_->setStartAndGoalStates(start, goal);
  // generate a few solutions; all will be added to the goal;
  for (int i = 0 ; i < 1 ; ++i){
    // if (ss_->getPlanner())
    //   ss_->getPlanner()->clear();
        if(selector == 1){
        ob::PlannerPtr planner(new og::PRM(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 2){
        ob::PlannerPtr planner(new og::RRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 3){
        ob::PlannerPtr planner(new og::RRTConnect(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 4){
        ob::PlannerPtr planner(new og::RRTstar(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 5){
        ob::PlannerPtr planner(new og::LBTRRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 6){
        ob::PlannerPtr planner(new og::LazyRRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 7){
        ob::PlannerPtr planner(new og::TRRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 8){
        ob::PlannerPtr planner(new og::pRRT(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }else if(selector == 9){
        ob::PlannerPtr planner(new og::EST(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }
      else{
        ob::PlannerPtr planner(new og::RRTConnect(ss_->getSpaceInformation()));
        ss_->setPlanner(planner);
      }
    ss_->solve();
  }
  const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
  // OMPL_INFORM("HEY! SHUBHAM --> Found %d solutions", (int)ns);
  if (ss_->haveSolutionPath()){
    ss_->simplifySolution();
    og::PathGeometric &p = ss_->getSolutionPath();
    ss_->getPathSimplifier()->simplifyMax(p);
    ss_->getPathSimplifier()->smoothBSpline(p);
    return true;
  }
  return false;
}

vector<point> Planning::recordSolution(){
  vect1.clear();
  if (!ss_ || !ss_->haveSolutionPath())
    {
      vect1.clear();
      return vect1;
    }
  og::PathGeometric &p = ss_->getSolutionPath();
  p.interpolate();
  for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
  {
    const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
    const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
    //cout<<w<<"            "<<h<<endl;
    point s;
    s.x=w;
    s.y=h;
    vect1.push_back(s);
    // cout<<s.x<<" "<<s.y<<" ";
  }
  // cout<<"\n\n\n\n\n\n";
  // for (int i = 0; i < vect1.size(); ++i)
  // {
  //   cout<<"("<<vect1[i].x<<","<<vect1[i].y<<"),";
  // }

  simplifyPoint(vect1);
  // cout<<"\n\n\n\n\n\n";
  // for (int i = 0; i < vect1.size(); ++i)
  // {
  //   cout<<"("<<vect1[i].x<<","<<vect1[i].y<<"),";
  // }

  cout<<endl;
  // cout<<" size of vect1 = "<<vect1.size()<<endl;
  return vect1;
}

void Planning::drw(){
  Mat img1(800,800,CV_8UC3,Scalar(0,255,0));
  circle(img1,Point(xStart,yStart),5,Scalar(250,0,0),CV_FILLED,8,0);
  circle(img1,Point(xGoal,yGoal),5,Scalar(250,0,0),CV_FILLED,8,0);
  for(int i=0;i<numObstacles;i++)
    circle(img1,Point(xc[i],yc[i]),10.0,Scalar(250,128,114),CV_FILLED,8,0);

  for(int i=0;i<vect1.size()-1;i++)
    line(img1,Point(vect1[i].x,vect1[i].y),Point(vect1[i+1].x,vect1[i+1].y),Scalar(32,0,170),2,8,0);

  namedWindow( "OMPL1", WINDOW_AUTOSIZE );
  imshow("OMPL1",img1);
  waitKey(1);
  //img1.release();
  img1=Scalar(0,0,0);

}

bool Planning::isStateValid(const ob::State *state) const {
  const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
  const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);
  for (int i = 0; i < numObstacles; ++i){
    if (sqrt(pow((xc[i]-w),2)+pow((yc[i]-h),2))<=10.0){
      return false;
    }
  }
  return true;
}



void Planning::planSimple(){
  //if throwing an error related to shared pointer conflict b/w boost and std, uncomment the follwing line and comment the 2nd one.
  // boost::shared_ptr<ob::RealVectorStateSpace> space(boost::make_shared<ob::RealVectorStateSpace>());
  auto space(std::make_shared<ob::RealVectorStateSpace>());

  space->addDimension(0.0, 800.0);
  space->addDimension(0.0, 800.0);
  maxWidth_ = 800-1;
  maxHeight_ = 800-1;

  //do the same here too.
  // ss_ = boost::make_shared<og::SimpleSetup>(space);
  ss_ = std::make_shared<og::SimpleSetup>(space);

  ss_->setStateValidityChecker([this](const ob::State *state) {return isStateValid(state);});
  space->setup();
  ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
}

void Planning::planWithSimpleSetup()
{
  // Construct the state space where we are planning
  ob::StateSpacePtr space(new ob::SE2StateSpace());

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0,xLeft);
  bounds.setHigh(0,xRight);
  bounds.setLow(1,yBottom);
  bounds.setHigh(1,yTop);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  // Instantiate SimpleSetup
  og::SimpleSetup ss(space);

  // Setup the StateValidityChecker
  ss.setStateValidityChecker(boost::bind(&Planning::isStateValid1, this, _1));

  // Setup Start and Goal
  ob::ScopedState<ob::SE2StateSpace> start(space);
  start->setXY(xStart,yStart);
  //cout << "start: "; start.print(cout);

  ob::ScopedState<ob::SE2StateSpace> goal(space);
  goal->setXY(xGoal,yGoal);
  //cout << "goal: "; goal.print(cout);

  ss.setStartAndGoalStates(start, goal);

  if(selector == 1){
    ob::PlannerPtr planner(new og::PRM(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 2){
    ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 3){
    ob::PlannerPtr planner(new og::RRTConnect(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 4){
    ob::PlannerPtr planner(new og::RRTstar(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 5){
    ob::PlannerPtr planner(new og::LBTRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 6){
    ob::PlannerPtr planner(new og::LazyRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 7){
    ob::PlannerPtr planner(new og::TRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 8){
    ob::PlannerPtr planner(new og::pRRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }else if(selector == 9){
    ob::PlannerPtr planner(new og::EST(ss.getSpaceInformation()));
    ss.setPlanner(planner);
  }

  // cout << "----------------" << endl;

  // Execute the planning algorithm
  ob::PlannerStatus solved = ss.solve(5);

  // If we have a solution,
  if (solved)
  {
    ss.getSolutionPath();

    // Simplify the solution
    ss.simplifySolution();
    // cout << "----------------" << endl;
    // cout << "Found solution:" << endl;

    //Print the solution path to a file
    std::ofstream ofs("path.dat");
    ss.getSolutionPath().printAsMatrix(ofs); 

    og::PathGeometric &p = ss.getSolutionPath();
    p.interpolate();

  }
  else
    cout << "No solution found" << endl;
}

void Planning::drawPath(){

  double temp;
  point p;
  vector<point> vec;
  std::ifstream input("path.dat");
  if (!input) {
    std::cerr << "error file" << std::endl;
    std::exit(1);
  }

  while(!input.eof()){
    input>>p.x>>p.y>>temp;
    vec.push_back(p);
  }

  for(int i=0;i<vec.size()-1;i++){
    line(img,Point(vec[i].x,vec[i].y),Point(vec[i+1].x,vec[i+1].y),Scalar(32,178,170),2,8,0);
  } 

}

void Planning::output(){
  circle(img,Point(xStart,yStart),5,Scalar(250,0,0),CV_FILLED,8,0);
  circle(img,Point(xGoal,yGoal),5,Scalar(250,0,0),CV_FILLED,8,0);
  namedWindow( "OMPL", WINDOW_AUTOSIZE );
  imshow("OMPL",img);
  waitKey(1);
  img=Scalar(0,255,0);
}
