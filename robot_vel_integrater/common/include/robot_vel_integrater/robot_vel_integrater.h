#ifndef ROBOT_VEL_INTEGRATER_H
#define ROBOT_VEL_INTEGRATER_H

#include <stdio.h>
#include <stdlib.h>
#include <exception>
#include <ros/ros.h>


#include <brics_actuator/JointVelocities.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

enum{
  Joints = 7
};

using namespace std;

class robot_vel_integrater{

 private:
  // declaration of topics to publish (For Multi_obj_control )
  bool control_is_initialized_;
  
  ros::Publisher topicpub_joint_l_;
  ros::Publisher topicpub_joint_r_;
  
  pr2_controllers_msgs::JointTrajectoryControllerState controller_state_l_msg_;
  pr2_controllers_msgs::JointTrajectoryControllerState controller_state_r_msg_;

  // declaration of topics to publish (For Visualization )
  ros::Publisher topicpub_joint_states_;
  sensor_msgs::JointState joint_states_msg_;  

  // declaration of topics to subscribe, callback is called for new messages arriving
  ros::Subscriber topicsub_joint_v_l_;
  ros::Subscriber topicsub_joint_v_r_;
  

  ros::Time m_Joint_v_stamp_l_;
  ros::Time m_Joint_v_stamp_r_;
  ros::Time m_Joint_states_stamp_;
  
  std::vector<std::string> m_JointNames;
  std::vector<double> Joint_v_l_;
  std::vector<double> Joint_v_r_;
  std::vector<double> Joint_v_;

  std::vector<double> Joint_v_buffer_l_;
  std::vector<double> Joint_v_buffer_r_;

  std::vector<double> Joint_pos_l_;
  std::vector<double> Joint_pos_r_;
  std::vector<double> Joint_pos_;

  std::vector<double> Joint_pos_l_last_;
  std::vector<double> Joint_pos_r_last_;
  std::vector<double> Joint_pos_last_;

  std::vector<double> Dual_dof_initial_;
  
  bool m_received_js_v_l_;
  bool m_received_js_v_r_;
  
  void getROSParameters();

  void topicCallback_joint_v_l_(const brics_actuator::JointVelocities::ConstPtr& msg);
  void topicCallback_joint_v_r_(const brics_actuator::JointVelocities::ConstPtr& msg);

  struct publish_pr2_thread_data{
    pr2_controllers_msgs::JointTrajectoryControllerState  * joint_vel_msg_ptr;
    ros::Publisher * publisher_ptr;
  };
  struct publish_sensor_thread_data{
    sensor_msgs::JointState  * joint_vel_msg_ptr;
    ros::Publisher * publisher_ptr;
  };

  static void * publish_arm_velocity(void * data);
  static void * publish_robot_velocity(void * data);
  
 public:
  ros::NodeHandle n_s;
  bool params_OK;
  unsigned int m_DOF; 
  double Sampling_peroid_;
  ros::Time last_publish_time_;

  void publish_joints();    
  bool one_step_forward();
  
  robot_vel_integrater(){
    n_s = ros::NodeHandle("~");
    
    params_OK = true;
    m_received_js_v_l_ = false;
    m_received_js_v_r_ = false;
    control_is_initialized_ = false;
    getROSParameters();
    
    // Publish the joint velocities to left and right arms 
    topicpub_joint_l_ = n_s.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/left_arm_controller/state", 1);
    topicpub_joint_r_ = n_s.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/right_arm_controller/state", 1);

    // publish the joint_states to the "robot state pulisher"
    topicpub_joint_states_ = n_s.advertise<sensor_msgs::JointState> ("/joint_states", 1);    
    // Subscribe the joint velocities to left and right arms 
    topicsub_joint_v_l_ = n_s.subscribe("/left_arm_controller/command_vel", 10, &robot_vel_integrater::topicCallback_joint_v_l_, this);         
    topicsub_joint_v_r_ = n_s.subscribe("/right_arm_controller/command_vel", 10, &robot_vel_integrater::topicCallback_joint_v_r_, this);         
  }
  ~robot_vel_integrater(){  

  }

};

#endif
