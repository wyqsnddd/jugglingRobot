# include <handJacobian/handJacobian.h>

void handJacobian::updateJointPositions(){
  chain_.getPositions(jointPos_);
  jointPos_(4) = fmod(jointPos_(4), 6.28319);
  jointPos_(6) = fmod(jointPos_(6), 6.28319);

}
 
void handJacobian::updateSpatialJacobian(){
  // KDL::JntArray kdlJoint;
  // kdlJoint.resize(DOF);
  KDL::Jacobian kdlJ;
  kdlJ.resize(DOF);


  jntToJacSolver_->JntToJac( jointPos_, kdlJ);
  kdlJacToEigen(kdlJ, spatialJacobian_);
  // kin_.KDLtoEigen(kdlJ, spatialJacobian_);
}    
void handJacobian::updateSpatialTransform(){
  KDL::Frame kdlT;
  jntToPoseSolver_->JntToCart(jointPos_, kdlT);
  // kin_.KDLtoEigen(kdlT, spatialTransform_);
  kdlTransformToEigen(kdlT, spatialTransform_);
}


void handJacobian::kdlJacToEigen(const KDL::Jacobian jac, Eigen::Matrix<double, 6, DOF> &jacEigen){
  for(unsigned int i=0; i<6; i++)
    for(unsigned int j=0; j<DOF; j++)
      jacEigen(i, j) = jac(i, j);
}
void handJacobian::kdlTransformToEigen(const KDL::Frame transform, Eigen::Affine3d &transformEigen){
  for(unsigned int i=0; i<3; i++)
    for(unsigned int j=0; j<4; j++)
      transformEigen(i, j) = transform(i, j);
}
				 
handJacobian::handJacobian(pr2_mechanism_model::RobotState *robot, std::string &rootName, std::string &leafName){

  spatialJacobian_.setZero();
  bodyJacobian_.setZero();
  spatialTransform_.translation().setZero();
  jointPos_.resize(DOF);
  if (!chain_.init(robot, rootName, leafName))
    {
      ROS_ERROR("pr2 controller could not use the chain from '%s' to '%s'",
		rootName.c_str(), leafName.c_str());
    }
  // std::cout<<"kdl chain is initialized"<<std::endl;
	
  KDL::Chain kdlChain;
  chain_.toKDL(kdlChain);
  if(!(kdlChain.getNrOfJoints() == DOF))
    throw "The number of joints on the right arm chain is NOT DOF";


  jntToPoseSolver_.reset(new KDL::ChainFkSolverPos_recursive(kdlChain));
  jntToJacSolver_.reset(new KDL::ChainJntToJacSolver(kdlChain));
  // std::cout<<"kdl  is initialized"<<std::endl;
  adjointMapP_ = new adjointMap(spatialTransform_); 
  // std::cout<<"adjointmap  is initialized"<<std::endl;
  update();
  // std::cout<<"finished the first update"<<std::endl;
    
}
void handJacobian::update(){

  updateJointPositions();
  // std::cout<<"Joint positions are updated"<<std::endl;
  updateSpatialJacobian();
  // std::cout<<"spatial jacobian is updated"<<std::endl;
  updateSpatialTransform();
  // std::cout<<"spatial transform is updated"<<std::endl;
  adjointMapP_->updateTransform(readTransform());
  // std::cout<<"adjoint map is updated"<<std::endl;
  //  update the body jacobian: 


  Eigen::Matrix<double, 6, 6> transition;
  transition.setZero();
  transition.block<3,3>(0,0) = readTransform().rotation().transpose(); 
  transition.block<3,3>(3,3) = readTransform().rotation().transpose(); 

  bodyJacobian_ = adjointMapP_->inverse()*readSpatialJacobian();
  // std::cout<<"body jacobian is updated"<<std::endl;
}
