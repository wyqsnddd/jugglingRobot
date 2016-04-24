# include <robotPaddlingControl/robotPaddlingController.h>

Eigen::Matrix<double, 3, 3>
skew_build( const Eigen::Matrix<double, 3, 1>  &vector_in){
  //    Matrix_skew Matrix;
  Eigen::Matrix<double, 3, 3>  Matrix;
    
  Matrix << 0, -vector_in(2), vector_in(1),
    vector_in(2), 0, -vector_in(0),
    -vector_in(1), vector_in(0), 0;

  return Matrix;
}

void robotPaddlingController::gurobiOptimizer::clearCoefficents(){

  for(int i = 0; i<DOF; i++){
    leftJointVVarPArray_[i]->set(GRB_DoubleAttr_Obj, 0.0);
    rightJointVVarPArray_[i]->set(GRB_DoubleAttr_Obj, 0.0);
  }
  // We have to update it .
  modelP_->update();
}
robotPaddlingController::gurobiOptimizer::gurobiOptimizer(
							  ros::NodeHandle n, 
							  handJacobian * leftArmPointer,
							  handJacobian * rightArmPointer,
							  std::vector<double> &jointPosL,
							  std::vector<double> &jointPosR,
							  double samplingPeroid,
							  double pseudoTime
							  ){
  ROS_INFO("Starting initializing Gurobi.\n");
  
  objValue_ = 0;
  envP_.reset(new GRBEnv());

  int methodIndicator(0), barHomogeneous(0);

  if (n.hasParam("methodIndicator")
      &&n.hasParam("barHomogeneous")
      ){
    n.getParam("methodIndicator", methodIndicator);
    n.getParam("barHomogeneous", barHomogeneous);
  }
  envP_->set(GRB_IntParam_Method, methodIndicator);

  ROS_INFO("Gurobi environmnet is set.\n");

  parameters.gainKEquality = 0;
  parameters.gainKInequality = 0;
  parameters.marginInequality = 0;

  if (
      n.hasParam("gainKEquality")
      &&n.hasParam("gainKInequality")
      &&n.hasParam("marginInequality")){
    n.getParam("gainKEquality", parameters.gainKEquality);
    n.getParam("gainKInequality", parameters.gainKInequality);
    n.getParam("marginInequality", parameters.marginInequality);
  }else
    throw "Unable to set the constraint margins or gains.";
  
  modelP_.reset(new GRBModel(*envP_));

  leftJointVVarPArray_.resize(DOF);
  rightJointVVarPArray_.resize(DOF);
  initializeJointVelocityVariables(leftJointVVarPArray_);
  initializeJointVelocityVariables(rightJointVVarPArray_);
  
}


void initializeJointVelocityVariables( std::vector<
				       boost::shared_ptr<GRBVar>
				       > jointVVarPArray_
				       ){

}
