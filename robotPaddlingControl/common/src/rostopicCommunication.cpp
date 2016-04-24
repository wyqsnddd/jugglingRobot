# include <rostopicCommunication/rostopicCommunication.h>

rostopicCommunication::rostopicCommunication(){
  n_ = ros::NodeHandle("~");

  readParameters_();

  /// setup the publishers
  leftJointVelocityPublisher_ = n_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/left_arm_controller/command_vel", 1);
  rightJointVelocityPublisher_ = n_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("/right_arm_controller/command_vel", 1);

  /// setup the subscribers
  leftJointPositionSubscriber_ = n_.subscribe("/left_arm_controller/state", 10, &rostopicCommunication::leftTopicCallback_, this);         
  rightJointPositionSubscriber_ = n_.subscribe("/right_arm_controller/state", 10, &rostopicCommunication::rightTopicCallback_, this);         
  
}

void rostopicCommunication::leftTopicCallback_(const brics_actuator::JointPositions::ConstPtr& msg)
{
  std::vector<double >jointBuffer;
  jointBuffer.resize(DOF);
  jointBuffer.assign(DOF, 0);
  
  if(msg->positions.size()==DOF){
    for(unsigned int i=0; i<DOF; i++){
      if(msg->positions[i].joint_uri !=jointNames_[i]){
	ROS_ERROR("Error in received joint name");
	return;
      }else{
	jointBuffer[i] = msg->positions[i].value;
      }
      
    }

  }else{
    ROS_ERROR("Error in the number ofjoints ");
    return;
  }
  ROS_DEBUG("New left arm joint positions received");
  leftJointPositions_ = jointBuffer;
  // leftJointPositionStamp_ = msg->positions[0].timeStamp;
  leftReceivedJointPosition_ = true;

}

void rostopicCommunication::rightTopicCallback_(const brics_actuator::JointPositions::ConstPtr& msg)
{
  std::vector<double >jointBuffer;
  jointBuffer.resize(DOF);
  jointBuffer.assign(DOF, 0);
  
  if(msg->positions.size()==DOF){
    for(unsigned int i=0; i<DOF; i++){
      if(msg->positions[i].joint_uri !=jointNames_[i + DOF]){
	ROS_ERROR("Error in received joint name");
	return;
      }else{
	jointBuffer[i] = msg->positions[i].value;
      }
      
    }

  }else{
    ROS_ERROR("Error in the number ofjoints ");
    return;
  }
  ROS_DEBUG("New right arm joint positions received");
  rightJointPositions_ = jointBuffer;
  // rightJointPositionStamp_ = msg->positions[0].timeStamp;
  rightReceivedJointPosition_ = true;
}



void rostopicCommunication::readParameters_()
{
  // For two arms 
  // Get joint names
  XmlRpc::XmlRpcValue jointNamesXmlRpc, samplingPeroidxmlRpc;
  std::vector<std::string> jointNames;
  double testFrequency;

  if(n_.hasParam("joint_names")&&n_.hasParam("Sampling_peroid")){
    n_.getParam("joint_names", jointNamesXmlRpc);
    n_.getParam("Sampling_peroid", testFrequency);
  }else{
    ROS_ERROR("Parameter joint_names or sampling peroid not set, shutting down node...");
    n_.shutdown();
    // params_OK=false;
  }


  /// Resize and assign of values to the JointNames
  jointNames_.resize(jointNamesXmlRpc.size());
  for(int i = 0; i < jointNamesXmlRpc.size(); i++){
    jointNames_[i] = (std::string)jointNamesXmlRpc[i];
  }
  // samplingPeroid_ = testFrequency;
    
  /// initialize the std::vectors 
  /// The velocities received 
  // leftJointVelocities_.resize(DOF);
  // leftJointVelocities_.assign(DOF, 0);
    
  // rightJointVelocities_.resize(DOF);
  // rightJointVelocities_.assign(DOF, 0);

  leftJointPositions_.resize(DOF);
  leftJointPositions_.assign(DOF, 0);

  rightJointPositions_.resize(DOF);
  rightJointPositions_.assign(DOF, 0);

  /// read the initial values: 

  XmlRpc::XmlRpcValue dualDofInitialXml;
  if (n_.hasParam("joint_initial_values")){
    n_.getParam("joint_initial_values", dualDofInitialXml);
  }else{
    ROS_ERROR("Parameter Dual_dof_lower is not set, shutting down node...");
    n_.shutdown();
    // params_OK=false;
  }

  for (unsigned int i = 0; i < DOF; i++){
    leftJointPositions_[i] = static_cast<double>(dualDofInitialXml[i]);
    rightJointPositions_[i] = static_cast<double>(dualDofInitialXml[i + DOF]);
  }

}
void * rostopicCommunication::publishArmVelocity(void * data){

  struct rostopicCommunication::publish_thread_data *my_data;
  
  my_data = (struct rostopicCommunication::publish_thread_data *)data;
  
  my_data->publisherPtr->publish(*(my_data->jointVelMsgPtr));

  // return the status 
  pthread_exit(NULL);

}

void rostopicCommunication::publishDualarmJointVelocities(
						   std::vector<double> & leftJointVelocities,
						   std::vector<double> & rightJointVelocities
							  ){
  ros::Time now = ros::Time::now();
  for(unsigned int i=0; i<DOF; i++){
    leftMsg_.velocities[i].value = leftJointVelocities[i];
    leftMsg_.velocities[i].joint_uri =jointNames_[i];
    rightMsg_.velocities[i].value = rightJointVelocities[i];
    rightMsg_.velocities[i].joint_uri =jointNames_[i + DOF];
  }
	
  leftMsg_.velocities[0].timeStamp = now;
  rightMsg_.velocities[0].timeStamp = now;
      
  int num_threads(2);
  struct rostopicCommunication::publish_thread_data thread_data_array[2];

  pthread_t publish_threads[num_threads];
  pthread_attr_t attr;
  pthread_attr_init(&attr);  
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  // left arm velocities
  thread_data_array[0].jointVelMsgPtr = &leftMsg_;
  thread_data_array[0].publisherPtr = &leftJointVelocityPublisher_;

  // right arm velocities
  thread_data_array[1].jointVelMsgPtr = &rightMsg_;
  thread_data_array[1].publisherPtr = &rightJointVelocityPublisher_;


  int lc = pthread_create(&publish_threads[0], &attr, &(rostopicCommunication::publishArmVelocity), (void *)&thread_data_array[0]);
  if(lc!=0){
    std::cout<<"In rostopicCommunication publish_velocities: creating thread 0 gives error code: "<<lc<<std::endl;
  }


  int rc = pthread_create(&publish_threads[1], &attr, &(rostopicCommunication::publishArmVelocity), (void *)&thread_data_array[1]);
  if(rc!=0){
    std::cout<<"In rostopicCommunication publish_velocities: creating thread 1 gives error code: "<<rc<<std::endl;
  }

  pthread_attr_destroy(&attr);

  void * lResultCatcher, * rResultCatcher; 
  int lStatus = pthread_join(publish_threads[0], &lResultCatcher);
  if(lStatus){
    ROS_ERROR("return code from pthread_join() is %d ", lStatus);
    exit(-1);
  }
  int rStatus = pthread_join(publish_threads[1], &rResultCatcher);
  if(rStatus){
    ROS_ERROR("return code from pthread_join() is %d ", rStatus);
    exit(-1);
  }

  ROS_DEBUG("rostopicCommunication:: pthread_join() is finished ");
  

}
