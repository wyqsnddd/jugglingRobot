# include <robotPaddlingControl/robotPaddlingController.h>


robotPaddlingController::robotPaddlingController(){
  commmunicationPointer_.reset( new rostopicCommunication());
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
void robotPaddlingController::publishJointVelocities(){

}

