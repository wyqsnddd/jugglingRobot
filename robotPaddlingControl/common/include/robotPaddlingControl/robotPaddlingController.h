#ifndef ROBOTPADDLINGCONTROLLER_H
#define ROBOTPADDLINGCONTROLLER_H

/**
 * @file robotPaddlingController.h
 * This is the header of the robot paddling controller class, which implements a constrained quadratic optimization method. 
 * @author Yuquan 
 */


#include <signal.h>
#include <exception>
# include <sstream>
# include <iostream>
# include <fstream>
# include <realtime_tools/realtime_publisher.h> 

# include <gurobi_c++.h> 


#include <ros/ros.h>
#include <brics_actuator/JointVelocities.h>
/* #include <pr2_controllers_msgs/JointTrajectoryControllerState.h> */
#include <sensor_msgs/JointState.h>

# include <exception>
# include <std_msgs/Float64.h>
# include <std_msgs/Float32MultiArray.h>


# include <boost/assert.hpp>
# include <boost/bind.hpp>
# include <boost/multi_array.hpp>
# include <boost/scoped_ptr.hpp>
# include <boost/shared_ptr.hpp>

#include <Eigen/Geometry>

# include <handJacobian/handJacobian.h>
# include <graspMatrix/graspMatrix.h>
# include <adjointMap/adjointMap.h>

# include <singleArmTranslationConstraint/singleArmTranslationConstraint.h>
# include <singleArmQuaternionConstraint/singleArmQuaternionConstraint.h>

# include <rostopicCommunication/rostopicCommunication.h>

/* enum{ */
/*   DOF = 7 */
/* }; */


class robotPaddlingController{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private: 
   ros::NodeHandle n_;
   void readTaskParameters();

   /* bool isInitialized() */
   /* { */
   /*   return initializedKDL_; */
   /* } */


   std::vector<double> jointPosL_;
   std::vector<double> jointPosR_;
 
   std::vector<double> dualDofLower_; ///< Lower bounds of the joint limits of the two arms 
   std::vector<double> dualDofUpper_; ///< Upper bounds of the joint limits of the two arms 
   std::vector<double> dualDofVLimits_; ///< Joint velocity bounds of the joi

   boost::shared_ptr<rostopicCommunication> commmunicationPointer_;   
   
 public:    
   robotPaddlingController();   

   ~robotPaddlingController(){   
   

   }

   bool isOK(){
     return n_.ok();
   }
   void publishJointVelocities();
};


# endif
