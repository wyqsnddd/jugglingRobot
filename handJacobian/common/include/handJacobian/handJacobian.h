# ifndef HANDJACOBIAN_H
# define HANDJACOBIAN_H

# include <ros/ros.h>
# include <sstream>
# include <iostream>
# include <fstream>
/* # include <string.h> */
/* # include <stdio.h> */
#include <vector>

# include <boost/scoped_ptr.hpp>
# include <boost/thread/condition.hpp>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
# include <kdl/chain.hpp>
# include <kdl/chainiksolvervel_wdls.hpp>
# include <kdl/chainjnttojacsolver.hpp>
# include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>


# include <kdl/frames.hpp>
# include <kdl/jacobian.hpp>
# include <kdl/jntarray.hpp>
# include <kdl/jntarrayvel.hpp>
# include <Eigen/Geometry>

# include <adjointMap/adjointMap.h>

class handJacobian{
 public:
 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private: 
  enum{
    DOF = 7,
  };

  Eigen::Matrix<double, 6, DOF> spatialJacobian_;
  Eigen::Matrix<double, 6, DOF> bodyJacobian_;
  Eigen::Affine3d spatialTransform_; 
      



  boost::scoped_ptr<KDL::ChainIkSolverVel_wdls> ikSolver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jntToJacSolver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jntToPoseSolver_;


  KDL::JntArray jointPos_;
  
  KDL::Chain armChain_;


  adjointMap * adjointMapP_; 
  
  void updateJointPositions(std::vector<double> &jointPosition);	  

  void kdlJacToEigen(const KDL::Jacobian jac, Eigen::Matrix<double, 6, DOF> &jacEigen);
  void updateSpatialJacobian();
  void updateSpatialTransform();
  void kdlTransformToEigen(const KDL::Frame transform, Eigen::Affine3d &transformEigen);
      
// initializes the chain using the tip of the tool frame
  bool initializeChain_(ros::NodeHandle *n,
			std::string chainTip,
			std::string chainRoot = "arm_base_link");

  bool getTreeFromURDF_(ros::NodeHandle *n, KDL::Tree &tree);
	
 public:
  // constructors: 
  handJacobian(
	       ros::NodeHandle *n, std::string &rootName, std::string &leafName);

  ~handJacobian(){
    free(adjointMapP_);
    /* std::cout<<"deconstructing handJacobian"<<std::endl; */
  }
  // update
  void update(std::vector<double> &jointPosition);

  // readers: 
  KDL::JntArray readJointPositions() const{
    return jointPos_;
  }
  /* std::vector<double>  readJointVelocities(){ */
  /*   std::vector<double> jointV; */
  /*   jointV.resize(DOF); */
  /*   jointV.assign(DOF, 0); */
    
  /*   armChain_.getVelocities(jointV); */
      
  /*   return jointV; */
  /* } */
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
