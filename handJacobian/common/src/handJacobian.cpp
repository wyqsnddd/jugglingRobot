# include <handJacobian/handJacobian.h>

void handJacobian::updateJointPositions(std::vector<double> &jointPosition ){
  
  jointPos_(0) = jointPosition[0];
  jointPos_(1) = jointPosition[1];
  jointPos_(2) = jointPosition[2];
  jointPos_(3) = jointPosition[3];
  jointPos_(4) = fmod(jointPosition[4], 6.28319);
  jointPos_(5) = jointPosition[5];
  jointPos_(6) = fmod(jointPosition[6], 6.28319);

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


bool handJacobian::getTreeFromURDF_(ros::NodeHandle *n, KDL::Tree &tree)
{
	/// Get robot_description from ROS parameter server
	std::string param_name = "robot_description";
	std::string full_param_name;
	std::string xml_string;

	n->searchParam(param_name, full_param_name);
	if (n->hasParam(full_param_name))
	{
		n->getParam(full_param_name.c_str(), xml_string);
	}

	else
	{
	  ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
	  n->shutdown();
		return false;
	}

	if (xml_string.size() == 0)
	{
		ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
		n->shutdown();
		return false;
	}
	ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

	/// Get urdf model out of robot_description
	urdf::Model model;
	// if(!model.initString(xml_string))
	//   {
	//     ROS_ERROR("Failed to parse urdf file");
	//     n->shutdown();
	//     return false;
	//   }
	// ROS_DEBUG("Successfully parsed urdf file");

	if(!kdl_parser::treeFromUrdfModel(model, tree)){
	  ROS_ERROR("Failed to construct kdl tree");
	  n->shutdown();
	  return false;
	}

	return true;
}


bool handJacobian::initializeChain_(ros::NodeHandle *n, 
				    std::string chainTip, std::string chainRoot){
 
  KDL::Tree tree;
  if(!getTreeFromURDF_(n, tree))
    {
      return false;
    }

  // chainRoot << "arm_base_link";
  // chainTip << armSelect << "_arm_tool_link";

  if(!tree.getChain(chainRoot.c_str(), chainTip.c_str(), armChain_))
    {
      ROS_ERROR("Error getting KDL chain");
      return false;
    }
  ROS_DEBUG("Number of segments: %d", armChain_.getNrOfSegments());
  ROS_DEBUG("Number of joints in chain: %d", armChain_.getNrOfJoints());
  
  ikSolver_.reset( new KDL::ChainIkSolverVel_wdls(armChain_, 0.1));
  jntToJacSolver_.reset(new KDL::ChainJntToJacSolver(armChain_));
  jntToPoseSolver_.reset(new KDL::ChainFkSolverPos_recursive(armChain_));

  ROS_INFO("Successfully initialized handJacobian");
  return true;
  
  
}
			 
handJacobian::handJacobian(
			   ros::NodeHandle *n, std::string &rootName, std::string &leafName){

  spatialJacobian_.setZero();
  bodyJacobian_.setZero();
  spatialTransform_.translation().setZero();
  jointPos_.resize(DOF);


  if(!initializeChain_(n, leafName, rootName)){
    ROS_ERROR("pr2 controller could not use the chain from '%s' to '%s'",
	      rootName.c_str(), leafName.c_str());
    }
  // std::cout<<"kdl chain is initialized"<<std::endl;

  if(!(armChain_.getNrOfJoints() == DOF))
    throw "The number of joints on the right arm chain is NOT DOF";

  // jntToPoseSolver_.reset(new KDL::ChainFkSolverPos_recursive(kdlChain));
  // jntToJacSolver_.reset(new KDL::ChainJntToJacSolver(kdlChain));
  // std::cout<<"kdl  is initialized"<<std::endl;


  adjointMapP_ = new adjointMap(spatialTransform_); 
  // std::cout<<"adjointmap  is initialized"<<std::endl;
  //  update();
  // std::cout<<"finished the first update"<<std::endl;
    
}
void handJacobian::update(std::vector<double> &jointPosition){

  updateJointPositions(jointPosition);
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
