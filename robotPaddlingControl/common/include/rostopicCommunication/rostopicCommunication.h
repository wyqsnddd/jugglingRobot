#ifndef ROSTOPICCOMMUNICATION_H
#define ROSTOPICCOMMUNICATION_H

#include <ros/ros.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <vector>

enum{
  DOF = 7
};

class rostopicCommunication{

 private:
  ros::Publisher leftJointVelocityPublisher_;
  ros::Publisher rightJointVelocityPublisher_;

  brics_actuator::JointVelocities leftMsg_;
  brics_actuator::JointVelocities rightMsg_;
    
  ros::Subscriber leftJointPositionSubscriber_;
  ros::Subscriber rightJointPositionSubscriber_;

  /* ros::Time leftJointVelocityStamp_; */
  /* ros::Time leftJointPositionStamp_; */
  /* ros::Time rightJointVelocityStamp_; */
  /* ros::Time rightJointPositionStamp_; */

  std::vector<std::string> jointNames_;
  
  /* std::vector<double> leftJointVelocities_; */
  /* std::vector<double> rightJointVelocities_; */
  std::vector<double> leftJointPositions_;
  std::vector<double> rightJointPositions_;

  bool leftReceivedJointPosition_;
  bool rightReceivedJointPosition_;

  void readParameters_();
  
  void leftTopicCallback_(const brics_actuator::JointPositions::ConstPtr& msg);
  void rightTopicCallback_(const brics_actuator::JointPositions::ConstPtr& msg);
  struct publish_thread_data{
    brics_actuator::JointVelocities * jointVelMsgPtr;
    ros::Publisher * publisherPtr;
  };
  static void * publishArmVelocity(void * data);

  ros::NodeHandle n_;

 public:
  rostopicCommunication();
    
  ~rostopicCommunication(){
  }

  std::vector<double>  readLeftArmJoints(){
    return leftJointPositions_;
  }
  std::vector<double>  readRightArmJoints(){
    return rightJointPositions_;
  }
  void publishDualarmJointVelocities(std::vector<double> & leftJointVelocities,
				     std::vector<double> & rightJointVelocities
				     );
};
# endif
