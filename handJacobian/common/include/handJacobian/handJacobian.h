# ifndef HANDJACOBIAN_H
# define HANDJACOBIAN_H

# include <ros/ros.h>
# include <sstream>
# include <iostream>
# include <fstream>
/* # include <string.h> */
/* # include <stdio.h> */

# include <pr2_mechanism_model/chain.h>
# include <pr2_mechanism_model/robot.h>

# include <boost/shared_ptr.hpp>
# include <boost/thread/condition.hpp>


# include <kdl/chain.hpp>
# include <kdl/chainjnttojacsolver.hpp>
# include <kdl/chainfksolverpos_recursive.hpp>
# include <kdl/frames.hpp>
# include <kdl/jacobian.hpp>
# include <kdl/jntarray.hpp>
# include <kdl/jntarrayvel.hpp>
# include <Eigen/Geometry>

/* #include <pr2_mechanism_model/kinematichelpers.h> */

# include <adjointMap/adjointMap.h>
/* # include <jacobianNumericalGradient/jacobianNumericalGradient.h> */

class handJacobian{
 public:
 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private: 
  enum{
    DOF = 7,
  };


  pr2_mechanism_model::Chain chain_;  
  /* pr2_mechanism_model::KinematicHelpers kin_; */

  Eigen::Matrix<double, 6, DOF> spatialJacobian_;
  Eigen::Matrix<double, 6, DOF> bodyJacobian_;
  Eigen::Affine3d spatialTransform_; 
      


  boost::shared_ptr<KDL::ChainJntToJacSolver> jntToJacSolver_;      
  boost::shared_ptr<KDL::ChainFkSolverPos>    jntToPoseSolver_;
  KDL::JntArray jointPos_;

  adjointMap * adjointMapP_; 
  
  void updateJointPositions();	  

  void kdlJacToEigen(const KDL::Jacobian jac, Eigen::Matrix<double, 6, DOF> &jacEigen);
  void updateSpatialJacobian();
  void updateSpatialTransform();
  void kdlTransformToEigen(const KDL::Frame transform, Eigen::Affine3d &transformEigen);
  // How to add the adjointMap? 
      
 public:
  // constructors: 
  handJacobian(pr2_mechanism_model::RobotState *robot, std::string &rootName, std::string &leafName);
  /* handJacobian(const int & input  ){ */
  /* handJacobian(const char* inString){ */
  /*   std::string foo("noConstruction"); */
    /* if(!(input==1001)) */
    /*   throw "illeagal argument."; */
  /* } */
  ~handJacobian(){
    free(adjointMapP_);
    /* std::cout<<"deconstructing handJacobian"<<std::endl; */
  }
  // update
  void update();

  // readers: 
  KDL::JntArray readJointPositions() const{
    return jointPos_;
  }
  std::vector<double>  readJointVelocities(){
    std::vector<double> jointV;
    jointV.resize(DOF);
    jointV.assign(DOF, 0);
    
    chain_.getVelocities(jointV);
      
    return jointV;
  }
  Eigen::Affine3d readTransform() const {
    return spatialTransform_;
  }
  Eigen::Matrix<double, 6, 6>  adgSF() const {
    return adjointMapP_->read();
  }
  Eigen::Matrix<double, 6, DOF> readSpatialJacobian() const {
    return spatialJacobian_;
  }
  Eigen::Matrix<double, 6, DOF> readBodyJacobian() const {
    return bodyJacobian_;
  }
  friend class ellipsoidNumericalGradient;
};
# endif
