# include <robotPaddlingControl/robotPaddlingController.h>


robotPaddlingController::robotPaddlingController(){
  try {

    loopController_.init();
    readTaskParameters();

    
    commmunicationPointer_.reset( new rostopicCommunication());
  
    std::string leftRootName, leftLeafName;
    if (n_.hasParam("kdlParameters/leftRootName")
	&& n_.hasParam("kdlParameters/leftName")) {
      n_.getParam("kdlParameters/leftRootName", leftRootName);
      n_.getParam("kdlParameters/leftName", leftLeafName);
    } else
      throw "Left chain variables are not set.";

    std::string rightRootName, rightLeafName;
    if (n_.hasParam("kdlParameters/rightRootName")
	&& n_.hasParam("kdlParameters/rightName")) {
      n_.getParam("kdlParameters/rightRootName", rightRootName);
      n_.getParam("kdlParameters/rightName", rightLeafName);
    } else
      throw "Right chain variables are not set.";

    leftArmP_ = new handJacobian(&n_, leftRootName, leftLeafName);
    rightArmP_ = new handJacobian(&n_, rightRootName, rightLeafName);

    leftArmP_->update(jointPosL_);
    rightArmP_->update(jointPosR_);

    // initialize gurobi

    // this is missing
    
    ROS_INFO("Initialized robot paddling controller.\n");
    

  } catch (const char* msg) {
    ROS_ERROR("%s, ", msg);
  }catch (std::exception& e) {
    ROS_INFO("Standard exception: %s . ", e.what());
  } catch (...) {
    ROS_INFO("Error! Unknown exception caught when updating.");
  }



}



void robotPaddlingController::readTaskParameters()
{
  ///< missing parameters ! 

  
  ///< read the joint position/velocity limits
  XmlRpc::XmlRpcValue Dual_dof_lower_xml, Dual_dof_upper_xml, Dual_dof_v_limits_xml;
  if (n_.hasParam("Dual_dof_lower") && n_.hasParam("Dual_dof_upper") && n_.hasParam("Dual_dof_v_limits") ){
    n_.getParam("Dual_dof_lower", Dual_dof_lower_xml);
    n_.getParam("Dual_dof_upper", Dual_dof_upper_xml);
    n_.getParam("Dual_dof_v_limits", Dual_dof_v_limits_xml);
  }else{
    ROS_ERROR("Parameter Dual_dof_lower is not set, shutting down node...");
    n_.shutdown();
  }

  dualDofLower_.resize(Dual_dof_v_limits_xml.size());
  dualDofUpper_.resize(Dual_dof_v_limits_xml.size());
  dualDofVLimits_.resize(Dual_dof_v_limits_xml.size());

  for (int i = 0; i < Dual_dof_lower_xml.size(); i++)
    {
      dualDofLower_[i] = static_cast<double>(Dual_dof_lower_xml[i]);
      dualDofUpper_[i] = static_cast<double>(Dual_dof_upper_xml[i]);
      dualDofVLimits_[i] =static_cast<double>(Dual_dof_v_limits_xml[i]);
    }

  ///< read the initial joint positions
  XmlRpc::XmlRpcValue Dual_dof_initial_xml;
  if (n_.hasParam("joint_initial_values")){
    n_.getParam("joint_initial_values", Dual_dof_initial_xml);
  }else{
    ROS_ERROR("Parameter Dual_dof_lower is not set, shutting down node...");
    n_.shutdown();
  }
  
  jointPosL_.resize(DOF);
  jointPosL_.assign(DOF, 0);
  jointPosR_.resize(DOF);
  jointPosR_.assign(DOF, 0);


  for (unsigned int i = 0; i < DOF; i++){
    jointPosL_[i] = static_cast<double>(Dual_dof_initial_xml[i]);
    jointPosR_[i] = static_cast<double>(Dual_dof_initial_xml[i + DOF]);
  }


}
void robotPaddlingController::publishOptimalJointVelocities(){

  ROS_DEBUG("Publish velocities to both the left and the right arms ");

  if(QuaProg()){
    
    /// update the new joint velocities 

    ///< Publish joint velocities
    // commmunicationPointer_->publishJointVelocities(?, ?);
  }

}

bool robotPaddlingController::QuaProg(){
  loopController_.time +=  loopController_.samplingPeroid;
  ROS_DEBUG("Quaprog started  initilized ");
  

  // update the joint positions
  jointPosL_ = commmunicationPointer_->readLeftArmJoints();
  jointPosR_ = commmunicationPointer_->readLeftArmJoints();
  
  // Update the joint position dependent vairalbes.
  leftArmP_->update(jointPosL_);
  rightArmP_->update(jointPosR_);
  
  std::cout << "seperation line:---------------------------- "
	    << std::endl;

  // missing the gurobi part 
  
  // gurobiP_->optimize(armJoints_, robotArmP_, jacobianGradientPointer_,
  // 		     pseudoTime)
    // int optimStatus = modelP_->get(GRB_IntAttr_Status);
    // if(optimStatus == GRB_OPTIMAL){


  // I may need joint limits update 
  // gurobiP_->jointLimitsUpdate(n_, robotArmP_, armJoints_,
  // 				  loopController_.samplingPeroid);


  double pseudoTime = loopController_.loopCount
    *loopController_.samplingPeroid;
      // gurobiP_->optimize(armJoints_, robotArmP_, jacobianGradientPointer_,
      // 			 pseudoTime);
  
  loopController_.loopCount++;
      
  return true;
}
