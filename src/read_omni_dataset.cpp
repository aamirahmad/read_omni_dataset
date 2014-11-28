
#include "read_omni_dataset.h"


inline void addLandmark(int vertexId, double x, double y)
{
  ROS_INFO("A fixed landmark with ID %d at position x=%f, y=%f must be created in your application, e.g., fixed vertices in a graph if your application is graph-based",vertexId,x,y);
}


void ReadRobotMessages::initializeFixedLandmarks()
{
    addLandmark(0,  6.0,  4.5);
    addLandmark(1,  6.0, -4.5);
    addLandmark(2,  0.0, -4.5);
    addLandmark(3,  0.0,  4.5);
    addLandmark(4,  0.0, -1.5);
    addLandmark(5,  0.0,  1.5);
    addLandmark(6,  3.0, -4.5);
    addLandmark(7,  3.0,  4.5);
    addLandmark(8,  3.75, -2.25);
    addLandmark(9,  3.75,  2.25);   
}



////////////////////////// METHOD DEFINITIONS OF THE SELFROBOT CLASS //////////////////////

void SelfRobot::selfOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry, int RobotNumber)
{
  
  uint seq = odometry->header.seq;
  prevTime = curTime;
  curTime = odometry->header.stamp;
  
  ROS_WARN(" got odometry from self-robot (ID=%d) at time %d\n",RobotNumber, odometry->header.stamp.sec);  

  //Below is an example how to extract the odometry from the message and use it to propogate the robot state by simply concatenating successive odometry readings-
  double odomVector[3] = {odometry->pose.pose.position.x, odometry->pose.pose.position.y, tf::getYaw(odometry->pose.pose.orientation)};
  
  Eigen::Isometry2d odom; 
  odom = Eigen::Rotation2Dd(odomVector[2]).toRotationMatrix();
  odom.translation() = Eigen::Vector2d(odomVector[0], odomVector[1]);  

  if(seq == 0)
    curPose = initPose;
  else
  {
    prevPose = curPose;
    curPose = prevPose*odom;
  }
   
  Eigen::Vector2d t;
  t = curPose.translation();
  Eigen::Matrix<double,2,2> r = curPose.linear();
  double angle = acos(r(0,0));
  
  ROS_INFO("Odometry propogated self robot state is x=%f, y=%f, theta=%f",t(0),t(1),angle);

}


void SelfRobot::selfTargetDataCallback(const read_omni_dataset::BallData::ConstPtr& ballData, int RobotNumber)
{
  ROS_WARN("Got ball data from self robot %d",RobotNumber);  
  Time curObservationTime = ballData->header.stamp;
  
  if(ballData->found)
  {    
    ///Below is the procedure to calculate the observation covariance associate with the ball measurement made by the robots. Caution: Make sure the validity of the calculations below by crosschecking the obvious things, e.g., covariance cannot be negative or very close to 0
    
    Eigen::Vector2d tempBallObsVec = Eigen::Vector2d(ballData->x,ballData->y);
    
    double d = tempBallObsVec.norm(),
	   phi = atan2(ballData->y,ballData->x);
    
    double covDD = (double)(1/ballData->mismatchFactor)*(K3*d + K4*(d*d));
    double covPhiPhi = K5*(1/(d+1));
    
    double covXX = pow(cos(phi),2) * covDD 
				+ pow(sin(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
    double covYY = pow(sin(phi),2) * covDD 
				+ pow(cos(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
    ROS_INFO("Ball found in the image, refer to the method to see how covariances are calculated");	
    
  }
  else
  {
    ROS_INFO("Ball not found in the image");
  }
  
}


void SelfRobot::selfLandmarkDataCallback(const read_omni_dataset::LRMLandmarksData::ConstPtr& landmarkData, int RobotNumber)
{
  ROS_WARN(" got landmark data from self robot (ID=%d)",RobotNumber);  
  
  uint seq = landmarkData->header.seq;

  // There are a total of 10 distinct, known and static landmarks in this dataset.
  for(int i=0;i<10; i++)
  {
    if(landmarkData->found[i])
    {
    
      ///Below is the procedure to calculate the observation covariance associate with the ball measurement made by the robots. Caution: Make sure the validity of the calculations below by crosschecking the obvious things, e.g., covariance cannot be negative or very close to 0     
      
      Eigen::Vector2d tempLandmarkObsVec = Eigen::Vector2d(landmarkData->x[i],landmarkData->y[i]);

      double d = tempLandmarkObsVec.norm(),
	     phi = atan2(landmarkData->y[i],landmarkData->x[i]);
      
      double covDD = (K1*fabs(1.0-(landmarkData->AreaLandMarkActualinPixels[i]/landmarkData->AreaLandMarkExpectedinPixels[i])))*(d*d);
      double covPhiPhi = K2*(1/(d+1));
      
      double covXX = pow(cos(phi),2) * covDD 
				  + pow(sin(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
      double covYY = pow(sin(phi),2) * covDD 
				  + pow(cos(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
      
      ROS_INFO("Landmark %d found in the image, refer to the method to see how covariances are calculated",i);  
    }
  }
  
 }

 

////////////////////////// METHOD DEFINITIONS OF THE TEAMMATEROBOT CLASS //////////////////////

void TeammateRobot::teammateOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry, int RobotNumber)
{

  uint seq = odometry->header.seq;
  prevTime = curTime;
  curTime = odometry->header.stamp;
  
  ROS_WARN(" got odometry from teammate-robot (ID=%d) at time %d\n",RobotNumber, odometry->header.stamp.sec);  

  //Below is an example how to extract the odometry from the message and use it to propogate the robot state by simply concatenating successive odometry readings-
  double odomVector[3] = {odometry->pose.pose.position.x, odometry->pose.pose.position.y, tf::getYaw(odometry->pose.pose.orientation)};
  
  Eigen::Isometry2d odom; 
  odom = Eigen::Rotation2Dd(odomVector[2]).toRotationMatrix();
  odom.translation() = Eigen::Vector2d(odomVector[0], odomVector[1]);  

  if(seq == 0)
    curPose = initPose;
  else
  {
    prevPose = curPose;
    curPose = prevPose*odom;
  }
   
  Eigen::Vector2d t;
  t = curPose.translation();
  Eigen::Matrix<double,2,2> r = curPose.linear();
  double angle = acos(r(0,0));
  
  ROS_INFO("Odometry propogated state f robot OMNI%d is x=%f, y=%f, theta=%f",RobotNumber, t(0),t(1),angle);

}


void TeammateRobot::teammateTargetDataCallback(const read_omni_dataset::BallData::ConstPtr& ballData, int RobotNumber)
{
  ROS_WARN("Got ball data from teammate robot %d",RobotNumber);  
  Time curObservationTime = ballData->header.stamp;
  
  if(ballData->found)
  {    
    ///Below is the procedure to calculate the observation covariance associate with the ball measurement made by the robots.
    
    Eigen::Vector2d tempBallObsVec = Eigen::Vector2d(ballData->x,ballData->y);
    
    double d = tempBallObsVec.norm(),
	   phi = atan2(ballData->y,ballData->x);
    
    double covDD = (double)(1/ballData->mismatchFactor)*(K3*d + K4*(d*d));
    double covPhiPhi = K5*(1/(d+1));
    
    double covXX = pow(cos(phi),2) * covDD 
				+ pow(sin(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
    double covYY = pow(sin(phi),2) * covDD 
				+ pow(cos(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
    ROS_INFO("Ball found in the image, refer to the method to see how covariances are calculated");	
    
  }
  else
  {
    ROS_INFO("Ball not found in the image");
  }
}


void TeammateRobot::teammateLandmarkDataCallback(const read_omni_dataset::LRMLandmarksData::ConstPtr& landmarkData, int RobotNumber)
{
  ROS_WARN(" got landmark data from teammate robot (ID=%d)",RobotNumber);  
  
  uint seq = landmarkData->header.seq;

  // There are a total of 10 distinct, known and static landmarks in this dataset.
  for(int i=0;i<10; i++)
  {
    if(landmarkData->found[i])
    {
    
      ///Below is the procedure to calculate the observation covariance associate with the ball measurement made by the robots. Caution: Make sure the validity of the calculations below by crosschecking the obvious things, e.g., covariance cannot be negative or very close to 0     
      
      Eigen::Vector2d tempLandmarkObsVec = Eigen::Vector2d(landmarkData->x[i],landmarkData->y[i]);

      double d = tempLandmarkObsVec.norm(),
	     phi = atan2(landmarkData->y[i],landmarkData->x[i]);
      
      double covDD = (K1*fabs(1.0-(landmarkData->AreaLandMarkActualinPixels[i]/landmarkData->AreaLandMarkExpectedinPixels[i])))*(d*d);
      double covPhiPhi = K2*(1/(d+1));
      
      double covXX = pow(cos(phi),2) * covDD 
				  + pow(sin(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
      double covYY = pow(sin(phi),2) * covDD 
				  + pow(cos(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
      
      ROS_INFO("Landmark %d found in the image, refer to the method to see how covariances are calculated",i);  
    }
  }
 }



int main (int argc, char* argv[])
{
  ros::init(argc, argv, "read_omni_dataset");
 
  if(MY_ID==2)
  {
    ROS_INFO("OMNI2 not present in dataset. Please try with another Robot ID for self robot");
    return 0;
  }
  ReadRobotMessages node;
  node.initializeFixedLandmarks();

  spin();
  return 0;
}




























