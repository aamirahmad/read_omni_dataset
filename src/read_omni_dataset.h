#include <cstdio>
#include <iostream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <read_omni_dataset/BallData.h>
#include <read_omni_dataset/LRMLandmarksData.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "std_msgs/String.h"

// #define M_PI        3.141592653589793238462643383280    /* pi */


// Most of these values will be read through the parameter server, but a default value is given

std::size_t NUM_ROBOTS = 5; // total number of robots in the team including self
const std::size_t NUM_SENSORS_PER_ROBOs = 3;// SENSORS include odometry, each feature sensor like a ball detector, each landmark-set detector and so on. In this case for example the number of sensors are 3, 1-odometry, 1-orange ball, 1-landmarkset. Usually this must co-incide with the number of topics to which each robot is publishing its sensed information.

const std::size_t NUM_TARGETS = 1; // Number of targets being tracked. In omni dataset, only one target exists for now: the orange ball. This may be improved in future by adding the blue ball which can be seen the raw footage of the dataset experiment

//Below are empirically obtained coefficients in the covariance expression. See (add publications here)


//coefficients for landmark observation covariance
std::size_t K1 = 2.0;
std::size_t K2 = 0.5;

//coefficients for target observation covariance
std::size_t K3 = 0.2;
std::size_t K4 = 0.5;
std::size_t K5 = 0.5;
//float K3 = 0.2, K4 = 0.5, K5 = 0.5; 

std::size_t ROB_HT = 0.81; //(In this dataset) fixed height of the robots above ground in meter
std::size_t MY_ID = 1; // Use this flag to set the ID of the robot expected to run a certain decentralized algorithm. Robot with MY_ID will be trated as the self robot running the algorithm while the rest will be considered teammates. Note that in the dataset there are 4 robots with IDs 1,3,4 and 5. Robot with ID=2 is not present.

//Initial 2D positons of the robot as obtained from the overhead ground truth system. The order is OMNI1 OMNI2 OMNI3 OMNI4 and OMNI5. Notice OMNI2 is initialized as 0,0 because the robot is absent from the dataset.
//This initialization will work only if the read_omni_dataset node startes befoe the rosbag of the dataset. Obviously, the initialization below is for the initial positions at the begginning of the dataset. Otherwise, the initialization makes the odometry-only trajecory frame transformed to the origin.
const double initArray[10] = {5.086676,-2.648978,0.0,0.0,1.688772,-2.095153,3.26839,-3.574936,4.058235,-0.127530};


using namespace ros;
using namespace std;

template <typename T>
void readParamDouble(ros::NodeHandle *nh, const std::string name, std::size_t variable){

  double tmp;
  ostringstream oss;
  if(nh->getParam(name, tmp))
  {
    variable = (T)tmp;
    oss << "Received parameter " << name << "=" << variable;
    ROS_INFO("%s", oss.str().c_str());
  }
  else
    ROS_ERROR("Failed to receive parameter %s", name.c_str());
}

class SelfRobot
{
  NodeHandle *nh;
  //One subscriber per sensor in the robot
  Subscriber sOdom_;
  Subscriber sBall_;
  Subscriber sLandmark_;
  
  Eigen::Isometry2d initPose; // x y theta;
  Eigen::Isometry2d prevPose;  
  
  public:
    SelfRobot(NodeHandle *nh, int robotNumber, Eigen::Isometry2d _initPose): initPose(_initPose), curPose(_initPose)
    {
    
      sOdom_ = nh->subscribe<nav_msgs::Odometry>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/odometry", 10, boost::bind(&SelfRobot::selfOdometryCallback,this, _1,robotNumber+1));
      
      sBall_ = nh->subscribe<read_omni_dataset::BallData>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/orangeball3Dposition", 10, boost::bind(&SelfRobot::selfTargetDataCallback,this, _1,robotNumber+1));
      
      sLandmark_ = nh->subscribe<read_omni_dataset::LRMLandmarksData>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/landmarkspositions", 10, boost::bind(&SelfRobot::selfLandmarkDataCallback,this, _1,robotNumber+1));
      
      ROS_INFO(" constructing SelfRobot <<object>> and called sensor subscribers for this robot %d",robotNumber+1);
      
    }

    /// Use this method to implement perception algorithms
    void selfOdometryCallback(const nav_msgs::Odometry::ConstPtr&, int);
    
    /// Use this method to implement perception algorithms
    void selfTargetDataCallback(const read_omni_dataset::BallData::ConstPtr&, int);
     
    /// Use this method to implement perception algorithms
    void selfLandmarkDataCallback(const read_omni_dataset::LRMLandmarksData::ConstPtr&, int);

    Eigen::Isometry2d curPose;
    Time curTime;
    Time prevTime;
    
};


class TeammateRobot
{
  NodeHandle *nh;
  //One subscriber per sensor in the robot
  Subscriber sOdom_;
  Subscriber sBall_;
  Subscriber sLandmark_;

  Eigen::Isometry2d initPose; // x y theta;
  Eigen::Isometry2d prevPose;
  
  public:
    TeammateRobot(NodeHandle *nh, int robotNumber, Eigen::Isometry2d _initPose): initPose(_initPose), curPose(_initPose)
    {
    
      sOdom_ = nh->subscribe<nav_msgs::Odometry>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/odometry", 10, boost::bind(&TeammateRobot::teammateOdometryCallback,this, _1,robotNumber+1));
      
      sBall_ = nh->subscribe<read_omni_dataset::BallData>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/orangeball3Dposition", 10, boost::bind(&TeammateRobot::teammateTargetDataCallback,this, _1,robotNumber+1));
      
      sLandmark_ = nh->subscribe<read_omni_dataset::LRMLandmarksData>("/omni"+boost::lexical_cast<string>(robotNumber+1)+"/landmarkspositions", 10, boost::bind(&TeammateRobot::teammateLandmarkDataCallback,this, _1,robotNumber+1));
      
      ROS_INFO(" constructing TeammateRobot object and called sensor subscribers for robot %d",robotNumber+1);
      
    }

    /// Use this method to implement perception algorithms
    void teammateOdometryCallback(const nav_msgs::Odometry::ConstPtr&, int);
    
    /// Use this method to implement perception algorithms
    void teammateTargetDataCallback(const read_omni_dataset::BallData::ConstPtr&, int);
    
    /// Use this method to implement perception algorithms
    void teammateLandmarkDataCallback(const read_omni_dataset::LRMLandmarksData::ConstPtr&, int);
  

    Eigen::Isometry2d curPose;
    Time curTime;
    Time prevTime;
    
};



class ReadRobotMessages
{
  NodeHandle nh_;
  Rate loop_rate_;
  
  SelfRobot* robot_;
  vector<TeammateRobot*> teammateRobots_;

  public:
    ReadRobotMessages(): loop_rate_(30), nh_("~")
    {
      readParamDouble<int>(&nh_, "NUM_ROBOTS", NUM_ROBOTS);
      readParamDouble<double>(&nh_, "ROB_HT", ROB_HT);
      readParamDouble<int>(&nh_, "MY_ID", MY_ID);
      readParamDouble<double>(&nh_, "LANDMARK_COV/K1", K1);
      readParamDouble<double>(&nh_, "LANDMARK_COV/K2", K2);
      readParamDouble<double>(&nh_, "LANDMARK_COV/K3", K3);
      readParamDouble<double>(&nh_, "LANDMARK_COV/K4", K4);
      readParamDouble<double>(&nh_, "LANDMARK_COV/K5", K5);

      Eigen::Isometry2d initialRobotPose;
      
      teammateRobots_.reserve(NUM_ROBOTS);
	
      for(int i=0;i<NUM_ROBOTS;i++)
      {
	initialRobotPose = Eigen::Rotation2Dd(-M_PI).toRotationMatrix();
	initialRobotPose.translation() = Eigen::Vector2d(initArray[2*i+0],initArray[2*i+1]); 
	
	if(i+1 == MY_ID)
	{
      robot_ = new SelfRobot(&nh_,i, initialRobotPose);
	}
	else
	{	  
      TeammateRobot *tempRobot = new TeammateRobot(&nh_,i, initialRobotPose);
	  teammateRobots_.push_back(tempRobot);
	}	
      }
      
      
    }
    
    void initializeFixedLandmarks();
    
    
    
};






































