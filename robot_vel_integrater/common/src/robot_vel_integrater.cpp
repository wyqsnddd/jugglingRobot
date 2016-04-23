#include <robot_vel_integrater/robot_vel_integrater.h>

void robot_vel_integrater::getROSParameters()
{
    
    // For two arms 
    // Get joint names
    XmlRpc::XmlRpcValue JointNamesXmlRpc, Sampling_peroidxmlRpc;
    std::vector<std::string> JointNames;
    double test_frequency;
    if(n_s.hasParam("joint_names")&&n_s.hasParam("Sampling_peroid")){
      n_s.getParam("joint_names", JointNamesXmlRpc);
      //    n_s.getParam("Sampling_peroid", Sampling_peroidxmlRpc);
      n_s.getParam("Sampling_peroid", test_frequency);
    }else{
      ROS_ERROR("Parameter joint_names or sampling peroid not set, shutting down node...");
      n_s.shutdown();
      params_OK=false;
    }


    /// Resize and assign of values to the JointNames
    JointNames.resize(JointNamesXmlRpc.size());
    for(int i = 0; i < JointNamesXmlRpc.size(); i++){
      JointNames[i] = (std::string)JointNamesXmlRpc[i];
    }

    Sampling_peroid_ = test_frequency;

    m_JointNames = JointNames;
    m_DOF = JointNames.size()/2;
  
    // The velocities received 
    Joint_v_l_.resize(m_DOF);
    Joint_v_r_.resize(m_DOF);
    Joint_v_.resize(2*m_DOF);
    Dual_dof_initial_.resize(2*m_DOF);
    Joint_v_buffer_l_.resize(m_DOF);
    Joint_v_buffer_r_.resize(m_DOF);
  
    Joint_pos_l_.resize(m_DOF);
    Joint_pos_r_.resize(m_DOF);
    Joint_pos_.resize(2*m_DOF);
    
    Joint_pos_l_.assign(m_DOF, 0);
    Joint_pos_r_.assign(m_DOF, 0);
    Joint_pos_.assign(2*m_DOF, 0);

    Joint_pos_l_last_.resize(m_DOF);
    Joint_pos_r_last_.resize(m_DOF);
    Joint_pos_last_.resize(2*m_DOF);
    
    Joint_pos_l_last_.assign(m_DOF, 0);
    Joint_pos_r_last_.assign(m_DOF, 0);
    Joint_pos_last_.assign(2*m_DOF, 0);

    // Get the initial positions for both arms 
    try{

      XmlRpc::XmlRpcValue Dual_dof_initial_xml;
      if (n_s.hasParam("joint_initial_values")){
	n_s.getParam("joint_initial_values", Dual_dof_initial_xml);
      }else{
	ROS_ERROR("Parameter Dual_dof_lower is not set, shutting down node...");
	n_s.shutdown();
	params_OK=false;
      }
      //  std::cout<<"yes done"<<std::endl;

      /// Resize and assign of values to the JointNames

   
      for (unsigned int i = 0; i < m_DOF; i++){
	Dual_dof_initial_[i] = static_cast<double>(Dual_dof_initial_xml[i]);
	Joint_pos_last_[i] = static_cast<double>(Dual_dof_initial_xml[i]);
	Joint_pos_[i] = static_cast<double>(Dual_dof_initial_xml[i]);
	Dual_dof_initial_[i + m_DOF] = static_cast<double>(Dual_dof_initial_xml[i + m_DOF]);
	Joint_pos_last_[i + m_DOF] = static_cast<double>(Dual_dof_initial_xml[i + m_DOF]);
	Joint_pos_[i + m_DOF] = static_cast<double>(Dual_dof_initial_xml[i + m_DOF]);
	
	Joint_pos_l_[i] = static_cast<double>(Dual_dof_initial_xml[i]);
	Joint_pos_r_[i] = static_cast<double>(Dual_dof_initial_xml[i + m_DOF]);

	Joint_pos_l_last_[i] = static_cast<double>(Dual_dof_initial_xml[i]);
	Joint_pos_r_last_[i] = static_cast<double>(Dual_dof_initial_xml[i + m_DOF]);
	
      }

    //    Joint_pos_l_


    // To be published 
    controller_state_l_msg_.joint_names.resize(m_DOF);
    controller_state_r_msg_.joint_names.resize(m_DOF);
    controller_state_l_msg_.actual.positions.resize(m_DOF);
    controller_state_r_msg_.actual.positions.resize(m_DOF);
    joint_states_msg_.name.resize(2*m_DOF+2 + 2);
    joint_states_msg_.position.resize(2*m_DOF+2 +2);
  
    // Write the joint names 
    for(unsigned int ii = 0; ii < m_DOF; ii++){
      controller_state_l_msg_.joint_names[ii] = m_JointNames[ii];
      controller_state_r_msg_.joint_names[ii] = m_JointNames[ii + m_DOF];
      joint_states_msg_.name[ii] = m_JointNames[ii];
      joint_states_msg_.name[ii + m_DOF] = m_JointNames[ii + m_DOF];
    }
  }
  catch( exception& e ){  
    cout << "Standard exception: " << e.what() << endl;
    ROS_ERROR("Error occurs when evaluating ROS parameters! Stopping...");
  }

    
}

void robot_vel_integrater::topicCallback_joint_v_l_(const brics_actuator::JointVelocities::ConstPtr& msg)
{
  ROS_DEBUG("Received joint velocities from the Multi_obj_control node");
  control_is_initialized_ = true;
  
  //  std::cout << "RECEIVED left JOINT VEL" << std::endl;
  if(msg->velocities.size()==m_DOF){
    for(unsigned int i=0; i<m_DOF; i++){
      if(msg->velocities[i].joint_uri !=m_JointNames[i]){
	ROS_ERROR("Error in received joint name");
	return;
      }else{
	Joint_v_buffer_l_[i] = msg->velocities[i].value;
      }
      
    }

  }else{
    ROS_ERROR("Error in the number ofjoints ");
    return;
  }
  ROS_DEBUG("New vel for left arm received");
  Joint_v_l_ = Joint_v_buffer_l_;
  m_Joint_v_stamp_l_ = msg->velocities[0].timeStamp;
  m_received_js_v_l_ = true;

}


void robot_vel_integrater::topicCallback_joint_v_r_(const brics_actuator::JointVelocities::ConstPtr& msg)
{
  ROS_DEBUG("Received joint velocities from the Multi_obj_control node");
  control_is_initialized_ = true;
  //  std::cout << "RECIEVED right JOINT VEL" << std::endl;
  if(msg->velocities.size()==m_DOF){
    for(unsigned int i=0; i<m_DOF; i++){
      if(msg->velocities[i].joint_uri !=m_JointNames[i + m_DOF]){
	ROS_ERROR("Error in received joint name");
	return;
      }else{
	Joint_v_buffer_r_[i] = msg->velocities[i].value;
      }
    }
  }else{
    ROS_ERROR("Error in the number ofjoints ");
    return;
  }
  ROS_DEBUG("New vel for right arm received");
  Joint_v_r_ = Joint_v_buffer_r_;
  /*
    for(int i=0; i < 7; i ++ ){	
    cout<<"At receiver: The received Joint_v_r_ "<<i<<" is: "<<Joint_v_buffer_r_[i]<<endl;
    cout<<"At receiver: The stored Joint_v_r_  "<<i<<" is: "<<Joint_v_r_[i]<<endl;
    }
  */
  m_Joint_v_stamp_r_ = msg->velocities[0].timeStamp;
  m_received_js_v_r_ = true;
}



void * robot_vel_integrater::publish_arm_velocity(void * data){
  struct robot_vel_integrater::publish_pr2_thread_data *my_data;
  
  my_data = (struct robot_vel_integrater::publish_pr2_thread_data *)data;
  
  my_data->publisher_ptr->publish(*(my_data->joint_vel_msg_ptr));

  // return the status 
  pthread_exit(NULL);

}

void * robot_vel_integrater::publish_robot_velocity(void * data){
  struct robot_vel_integrater::publish_sensor_thread_data *my_data;
  
  my_data = (struct robot_vel_integrater::publish_sensor_thread_data *)data;
  
  my_data->publisher_ptr->publish(*(my_data->joint_vel_msg_ptr));

  // return the status 
  pthread_exit(NULL);

}


void robot_vel_integrater::publish_joints(){
  // publisher
  ros::Time now = ros::Time::now();
  if((m_received_js_v_l_ && m_received_js_v_r_)){

    ROS_DEBUG("Publish joint values to both the left and the right arms ");
    //      if(((m_JointPos_stamp - now).toSec()<0.1))
    if(
       ((m_Joint_v_stamp_l_ - now).toSec()<0.1)||((m_Joint_v_stamp_r_ - now).toSec()<0.1)
       ){
      if(one_step_forward()){
	// brics_actuator::JointVelocities joint_vel_msg;
	// joint_vel_msg.velocities.resize(m_DOF);
	for(unsigned int i=0; i<m_DOF; i++){
	  controller_state_l_msg_.actual.positions[i] = Joint_pos_l_[i];
	  controller_state_r_msg_.actual.positions[i] = Joint_pos_r_[i];
	  Joint_pos_r_last_[i] = Joint_pos_r_[i];
	  Joint_pos_l_last_[i] = Joint_pos_l_[i];
	  joint_states_msg_.position[i] = Joint_pos_[i];
	  joint_states_msg_.position[i + m_DOF] = Joint_pos_[i + m_DOF];
	  Joint_pos_last_[i] = Joint_pos_[i];
	  Joint_pos_last_[i + m_DOF] = Joint_pos_[i + m_DOF];
	}
	
	controller_state_l_msg_.header.stamp = now;
	controller_state_r_msg_.header.stamp = now;
	joint_states_msg_.header.stamp = now;
	
	joint_states_msg_.name[2*m_DOF] = "left_arm_bottom_finger_joint";
	joint_states_msg_.position[2*m_DOF] = 0.0;

	joint_states_msg_.name[2*m_DOF+1] = "left_arm_top_finger_joint";
	joint_states_msg_.position[2*m_DOF+1] = 0.0;

	joint_states_msg_.name[2*m_DOF+2] = "right_arm_bottom_finger_joint";
	joint_states_msg_.position[2*m_DOF+2] = 0.0;

	joint_states_msg_.name[2*m_DOF+3] = "right_arm_top_finger_joint";
	joint_states_msg_.position[2*m_DOF+3] = 0.0;

	
	// Publish the dummy joint positions for both arms    


	int num_threads(3);
	struct robot_vel_integrater::publish_pr2_thread_data thread_pr2_data_array[2];
	struct robot_vel_integrater::publish_sensor_thread_data thread_sensor_data;
	
        pthread_t publish_threads[num_threads];

	pthread_attr_t attr;
        pthread_attr_init(&attr);  
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	// left arm velocities
	thread_pr2_data_array[0].joint_vel_msg_ptr = &controller_state_l_msg_;
	thread_pr2_data_array[0].publisher_ptr = &topicpub_joint_l_;
  
	// right arm velocities 
	thread_pr2_data_array[1].joint_vel_msg_ptr = &controller_state_r_msg_;
	thread_pr2_data_array[1].publisher_ptr = &topicpub_joint_r_;
	// robot state 
	thread_sensor_data.joint_vel_msg_ptr = &joint_states_msg_;
	thread_sensor_data.publisher_ptr = &topicpub_joint_states_;
	
	int rc; 
	for(long t = 0; t < 2; t++){
	  //	  std::cout<<"In robot_vel_integrater publish_velocities: creating thread "<<t<<std::endl;
	  rc = pthread_create(&publish_threads[t], &attr, &(robot_vel_integrater::publish_arm_velocity), (void *)&thread_pr2_data_array[t]);
	  if(rc!=0){
	    std::cout<<"In robot_vel_integrater publish_velocities: creating thread "<<t<<" gives error code: "<<rc<<std::endl;
	  }
	}
	//	std::cout<<"In robot_vel_integrater publish_velocities: creating thread 2 "<<std::endl;
	rc = pthread_create(&publish_threads[2], &attr, &(robot_vel_integrater::publish_robot_velocity), (void *)&thread_sensor_data);
	if(rc!=0){
	  std::cout<<"In robot_vel_integrater publish_velocities: creating thread 2 gives error code: "<<rc<<std::endl;
	}
	
	pthread_attr_destroy(&attr);
	int status;
	void * result_catcher;

	for(long t =0; t<num_threads; t++){
	  status = pthread_join(publish_threads[t], &result_catcher);
	  
	  if(status){
	    printf("ERROR; return code from pthread_join() is %d\n", status);
	    exit(-1);
	  }
	}

	printf("pthread_join() is finished\n");

	//	topicpub_joint_l_.publish(controller_state_l_msg_);
	//	topicpub_joint_r_.publish(controller_state_r_msg_);
	// Publish the joint_states for the visualization  
	//	topicpub_joint_states_.publish(joint_states_msg_);	
	
    {
    	// now publish the SDH hand's joint states
    	// send zero position message
    	ros::Time time = ros::Time::now();

    	std::vector<std::string> joint_names_;
    	joint_names_.push_back("sdh_knuckle_joint");
    	joint_names_.push_back("sdh_thumb_2_joint");
    	joint_names_.push_back("sdh_thumb_3_joint");
    	joint_names_.push_back("sdh_finger_12_joint");
    	joint_names_.push_back("sdh_finger_13_joint");
    	joint_names_.push_back("sdh_finger_22_joint");
    	joint_names_.push_back("sdh_finger_23_joint");


    	unsigned int DOF_ = joint_names_.size();

    	// create joint_state message
    	sensor_msgs::JointState msg;
    	msg.header.stamp = time;
    	msg.name.resize(DOF_);
    	msg.position.resize(DOF_);
    	msg.velocity.resize(DOF_);
    	msg.effort.resize(DOF_);
    	// set joint names and map them to angles
    	msg.name = joint_names_;
    	//['sdh_knuckle_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']
    	// pos
    	msg.position[0] = 0.0; // sdh_knuckle_joint
    	msg.position[1] = 0.0; // sdh_thumb_2_joint
    	msg.position[2] = 0.0; // sdh_thumb_3_joint
    	msg.position[3] = 0.0; // sdh_finger_12_joint
    	msg.position[4] = 0.0; // sdh_finger_13_joint
    	msg.position[5] = 0.0; // sdh_finger_22_joint
    	msg.position[6] = 0.0; // sdh_finger_23_joint
    	// vel
    	msg.velocity[0] = 0.0; // sdh_knuckle_joint
    	msg.velocity[1] = 0.0; // sdh_thumb_2_joint
    	msg.velocity[2] = 0.0; // sdh_thumb_3_joint
    	msg.velocity[3] = 0.0; // sdh_finger_12_joint
    	msg.velocity[4] = 0.0; // sdh_finger_13_joint
    	msg.velocity[5] = 0.0; // sdh_finger_22_joint
    	msg.velocity[6] = 0.0; // sdh_finger_23_joint
    	// publish message
    	topicpub_joint_states_.publish(msg);

    	// because the robot_state_publisher doen't know about the mimic joint, we have to publish the coupled joint separately
    	sensor_msgs::JointState  mimicjointmsg;
    	mimicjointmsg.header.stamp = time;
    	mimicjointmsg.name.resize(1);
    	mimicjointmsg.position.resize(1);
    	mimicjointmsg.velocity.resize(1);
    	mimicjointmsg.name[0] = "sdh_finger_21_joint";
    	mimicjointmsg.position[0] = msg.position[0]; // sdh_knuckle_joint = sdh_finger_21_joint
    	mimicjointmsg.velocity[0] = msg.velocity[0]; // sdh_knuckle_joint = sdh_finger_21_joint
    	topicpub_joint_states_.publish(mimicjointmsg);
    }

	ROS_DEBUG("Joint values of two arms are published");
	ROS_DEBUG("'joint_states' to the 'robot_state_publisher' is published");
	last_publish_time_ = ros::Time::now();
	cout<<"Joint state published "  <<endl;
	
      }else{
	ROS_ERROR("Unable to calculate one step forward");
	return;
      } //end 

    }else{
      ROS_ERROR("Joint velocity are too old");
      return;
    } //end 

    m_received_js_v_l_ = false;
    m_received_js_v_r_ = false;
  
  }else if(!control_is_initialized_){
    // publish zero joint states for initialization 
    ROS_DEBUG("The control node is not published yet.");    
    // for(unsigned int i=0; i<m_DOF; i++){
    //   controller_state_l_msg_.actual.positions[i] = 0.5;
    //   controller_state_r_msg_.actual.positions[i] = 0.5;
    //   joint_states_msg_.position[i] = 0.5;
    //   joint_states_msg_.position[i + m_DOF] = 0.5;
    // }

    for(unsigned int i=0; i<m_DOF; i++){
      controller_state_l_msg_.actual.positions[i] = Dual_dof_initial_[i];
      controller_state_r_msg_.actual.positions[i] = Dual_dof_initial_[i + m_DOF];
      joint_states_msg_.position[i] = Dual_dof_initial_[i];
      joint_states_msg_.position[i + m_DOF] = Dual_dof_initial_[i + m_DOF];
    }

    controller_state_l_msg_.header.stamp = now;
    controller_state_r_msg_.header.stamp = now;
    joint_states_msg_.header.stamp = now;

    joint_states_msg_.name[2*m_DOF] = "left_arm_bottom_finger_joint";
    joint_states_msg_.position[2*m_DOF] = 0.0;

    joint_states_msg_.name[2*m_DOF+1] = "left_arm_top_finger_joint";
    joint_states_msg_.position[2*m_DOF+1] = 0.0;

    joint_states_msg_.name[2*m_DOF + 2] = "right_arm_bottom_finger_joint";
    joint_states_msg_.position[2*m_DOF + 2] = 0.0;

    joint_states_msg_.name[2*m_DOF+3] = "right_arm_top_finger_joint";
    joint_states_msg_.position[2*m_DOF+3] = 0.0;
    
    
    topicpub_joint_l_.publish(controller_state_l_msg_);
    topicpub_joint_r_.publish(controller_state_r_msg_);
    topicpub_joint_states_.publish(joint_states_msg_);	
    
    {
    	// now publish the SDH hand's joint states
    	// send zero position message
    	ros::Time time = ros::Time::now();

    	std::vector<std::string> joint_names_;
    	joint_names_.push_back("sdh_knuckle_joint");
    	joint_names_.push_back("sdh_thumb_2_joint");
    	joint_names_.push_back("sdh_thumb_3_joint");
    	joint_names_.push_back("sdh_finger_12_joint");
    	joint_names_.push_back("sdh_finger_13_joint");
    	joint_names_.push_back("sdh_finger_22_joint");
    	joint_names_.push_back("sdh_finger_23_joint");


    	unsigned int DOF_ = joint_names_.size();

    	// create joint_state message
    	sensor_msgs::JointState msg;
    	msg.header.stamp = time;
    	msg.name.resize(DOF_);
    	msg.position.resize(DOF_);
    	msg.velocity.resize(DOF_);
    	msg.effort.resize(DOF_);
    	// set joint names and map them to angles
    	msg.name = joint_names_;
    	//['sdh_knuckle_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']
    	// pos
    	msg.position[0] = 0.0; // sdh_knuckle_joint
    	msg.position[1] = 0.0; // sdh_thumb_2_joint
    	msg.position[2] = 0.0; // sdh_thumb_3_joint
    	msg.position[3] = 0.0; // sdh_finger_12_joint
    	msg.position[4] = 0.0; // sdh_finger_13_joint
    	msg.position[5] = 0.0; // sdh_finger_22_joint
    	msg.position[6] = 0.0; // sdh_finger_23_joint
    	// vel
    	msg.velocity[0] = 0.0; // sdh_knuckle_joint
    	msg.velocity[1] = 0.0; // sdh_thumb_2_joint
    	msg.velocity[2] = 0.0; // sdh_thumb_3_joint
    	msg.velocity[3] = 0.0; // sdh_finger_12_joint
    	msg.velocity[4] = 0.0; // sdh_finger_13_joint
    	msg.velocity[5] = 0.0; // sdh_finger_22_joint
    	msg.velocity[6] = 0.0; // sdh_finger_23_joint
    	// publish message
    	topicpub_joint_states_.publish(msg);

    	// because the robot_state_publisher doen't know about the mimic joint, we have to publish the coupled joint separately
    	sensor_msgs::JointState  mimicjointmsg;
    	mimicjointmsg.header.stamp = time;
    	mimicjointmsg.name.resize(1);
    	mimicjointmsg.position.resize(1);
    	mimicjointmsg.velocity.resize(1);
    	mimicjointmsg.name[0] = "sdh_finger_21_joint";
    	mimicjointmsg.position[0] = msg.position[0]; // sdh_knuckle_joint = sdh_finger_21_joint
    	mimicjointmsg.velocity[0] = msg.velocity[0]; // sdh_knuckle_joint = sdh_finger_21_joint
    	topicpub_joint_states_.publish(mimicjointmsg);
    }

    ROS_DEBUG("Initial Joint values of two arms are published.");
    ROS_DEBUG("Initial 'joint_states' to the 'robot_state_publisher' is published.");
    last_publish_time_ = ros::Time::now();
    cout<<"Initial joint state published "  <<endl;
  }else{
    ROS_ERROR("Haven't received vel commands. The old joint states will be pulished." );    
    //    std::cout<<"the old joint states are:"<<std::endl;
    for(unsigned int i=0; i<m_DOF; i++){
      controller_state_l_msg_.actual.positions[i] = Joint_pos_l_last_[i];
      //      std::cout<<"The "<<i<<" old joint states is:"<<Joint_pos_l_last_[i] <<std::endl;
      //      std::cout<<"The "<<i<<" old joint states is:"<<Joint_pos_r_last_[i] <<std::endl;
      
      controller_state_r_msg_.actual.positions[i] = Joint_pos_r_last_[i];
      joint_states_msg_.position[i] = Joint_pos_last_[i];
      joint_states_msg_.position[i + m_DOF] = Joint_pos_last_[i + m_DOF];
    }
	
    controller_state_l_msg_.header.stamp = now;
    controller_state_r_msg_.header.stamp = now;
    joint_states_msg_.header.stamp = now;

    joint_states_msg_.name[2*m_DOF] = "left_arm_bottom_finger_joint";
    joint_states_msg_.position[2*m_DOF] = 0.0;

    joint_states_msg_.name[2*m_DOF+1] = "left_arm_top_finger_joint";
    joint_states_msg_.position[2*m_DOF+1] = 0.0;

    joint_states_msg_.name[2*m_DOF + 2] = "right_arm_bottom_finger_joint";
    joint_states_msg_.position[2*m_DOF + 2] = 0.0;

    joint_states_msg_.name[2*m_DOF+3] = "right_arm_top_finger_joint";
    joint_states_msg_.position[2*m_DOF+3] = 0.0;

	
    // Publish the dummy joint positions for both arms    
    topicpub_joint_l_.publish(controller_state_l_msg_);
    topicpub_joint_r_.publish(controller_state_r_msg_);
    // Publish the joint_states for the visualization  
    topicpub_joint_states_.publish(joint_states_msg_);	
	

    





    ROS_DEBUG("OLD Joint values of two arms are published");
    ROS_DEBUG("OLD 'joint_states' to the 'robot_state_publisher' is published");
    last_publish_time_ = ros::Time::now();
    cout<<"Old Joint state published "  <<endl;

    {
    	// now publish the SDH hand's joint states
    	// send zero position message
    	ros::Time time = ros::Time::now();

    	std::vector<std::string> joint_names_;
    	joint_names_.push_back("sdh_knuckle_joint");
    	joint_names_.push_back("sdh_thumb_2_joint");
    	joint_names_.push_back("sdh_thumb_3_joint");
    	joint_names_.push_back("sdh_finger_12_joint");
    	joint_names_.push_back("sdh_finger_13_joint");
    	joint_names_.push_back("sdh_finger_22_joint");
    	joint_names_.push_back("sdh_finger_23_joint");


    	unsigned int DOF_ = joint_names_.size();

    	// create joint_state message
    	sensor_msgs::JointState msg;
    	msg.header.stamp = time;
    	msg.name.resize(DOF_);
    	msg.position.resize(DOF_);
    	msg.velocity.resize(DOF_);
    	msg.effort.resize(DOF_);
    	// set joint names and map them to angles
    	msg.name = joint_names_;
    	//['sdh_knuckle_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint']
    	// pos
    	msg.position[0] = 0.0; // sdh_knuckle_joint
    	msg.position[1] = 0.0; // sdh_thumb_2_joint
    	msg.position[2] = 0.0; // sdh_thumb_3_joint
    	msg.position[3] = 0.0; // sdh_finger_12_joint
    	msg.position[4] = 0.0; // sdh_finger_13_joint
    	msg.position[5] = 0.0; // sdh_finger_22_joint
    	msg.position[6] = 0.0; // sdh_finger_23_joint
    	// vel
    	msg.velocity[0] = 0.0; // sdh_knuckle_joint
    	msg.velocity[1] = 0.0; // sdh_thumb_2_joint
    	msg.velocity[2] = 0.0; // sdh_thumb_3_joint
    	msg.velocity[3] = 0.0; // sdh_finger_12_joint
    	msg.velocity[4] = 0.0; // sdh_finger_13_joint
    	msg.velocity[5] = 0.0; // sdh_finger_22_joint
    	msg.velocity[6] = 0.0; // sdh_finger_23_joint
    	// publish message
    	topicpub_joint_states_.publish(msg);

    	// because the robot_state_publisher doen't know about the mimic joint, we have to publish the coupled joint separately
    	sensor_msgs::JointState  mimicjointmsg;
    	mimicjointmsg.header.stamp = time;
    	mimicjointmsg.name.resize(1);
    	mimicjointmsg.position.resize(1);
    	mimicjointmsg.velocity.resize(1);
    	mimicjointmsg.name[0] = "sdh_finger_21_joint";
    	mimicjointmsg.position[0] = msg.position[0]; // sdh_knuckle_joint = sdh_finger_21_joint
    	mimicjointmsg.velocity[0] = msg.velocity[0]; // sdh_knuckle_joint = sdh_finger_21_joint
    	topicpub_joint_states_.publish(mimicjointmsg);
    }



    
    return;
  } // end of received message 


} //end of publish function 



/*
  void robot_vel_integrater::publish_joints(){
  // publisher
  ros::Time now = ros::Time::now();
  
  if((m_received_js_v_l_ && m_received_js_v_r_)
  && ((m_Joint_v_stamp_l_ - now).toSec()<0.1)||((m_Joint_v_stamp_r_ - now).toSec()<0.1)
  && one_step_forward()){

  for(unsigned int i=0; i<m_DOF; i++){
  controller_state_l_msg_.actual.positions[i] = Joint_pos_l_[i];
  controller_state_r_msg_.actual.positions[i] = Joint_pos_r_[i];
  Joint_pos_r_last_[i] = Joint_pos_r_[i];
  Joint_pos_l_last_[i] = Joint_pos_l_[i];
  joint_states_msg_.position[i] = Joint_pos_[i];
  joint_states_msg_.position[i + m_DOF] = Joint_pos_[i + m_DOF];
  Joint_pos_last_[i] = Joint_pos_[i];
  Joint_pos_last_[i + m_DOF] = Joint_pos_[i + m_DOF];
  }
	
  controller_state_l_msg_.header.stamp = now;
  controller_state_r_msg_.header.stamp = now;
  joint_states_msg_.header.stamp = now;
	
  // Publish the dummy joint positions for both arms    
  topicpub_joint_l_.publish(controller_state_l_msg_);
  topicpub_joint_r_.publish(controller_state_r_msg_);
  // Publish the joint_states for the visualization  
  topicpub_joint_states_.publish(joint_states_msg_);	
	
  ROS_DEBUG("Joint values of two arms are published");
  ROS_DEBUG("'joint_states' to the 'robot_state_publisher' is published");
  last_publish_time_ = ros::Time::now();
  cout<<"Joint state published "  <<endl;
  m_received_js_v_l_ = false;
  m_received_js_v_r_ = false;
  
  }else if(!control_is_initialized_){
  // publish zero joint states for initialization 
  ROS_DEBUG("The control node is not published yet.");    
  for(unsigned int i=0; i<m_DOF; i++){
  controller_state_l_msg_.actual.positions[i] = 0.5;
  controller_state_r_msg_.actual.positions[i] = 0.5;
  joint_states_msg_.position[i] = 0.5;
  joint_states_msg_.position[i + m_DOF] = 0.5;
  }

  controller_state_l_msg_.header.stamp = now;
  controller_state_r_msg_.header.stamp = now;
  joint_states_msg_.header.stamp = now;
    
  topicpub_joint_l_.publish(controller_state_l_msg_);
  topicpub_joint_r_.publish(controller_state_r_msg_);
  topicpub_joint_states_.publish(joint_states_msg_);	
    
  ROS_DEBUG("Initial Joint values of two arms are published.");
  ROS_DEBUG("Initial 'joint_states' to the 'robot_state_publisher' is published.");
  last_publish_time_ = ros::Time::now();
  cout<<"Initial joint state published "  <<endl;
  }else{
  ROS_ERROR("Haven't received vel commands. The old joint states will be pulished." );    
  std::cout<<"the old joint states are:"<<std::endl;
  for(unsigned int i=0; i<m_DOF; i++){
  controller_state_l_msg_.actual.positions[i] = Joint_pos_l_last_[i];
  std::cout<<"The "<<i<<" old joint states is:"<<Joint_pos_l_last_[i] <<std::endl;
  std::cout<<"The "<<i<<" old joint states is:"<<Joint_pos_r_last_[i] <<std::endl;
      
  controller_state_r_msg_.actual.positions[i] = Joint_pos_r_last_[i];
  joint_states_msg_.position[i] = Joint_pos_last_[i];
  joint_states_msg_.position[i + m_DOF] = Joint_pos_last_[i + m_DOF];
  }
	
  controller_state_l_msg_.header.stamp = now;
  controller_state_r_msg_.header.stamp = now;
  joint_states_msg_.header.stamp = now;
	
  // Publish the dummy joint positions for both arms    
  topicpub_joint_l_.publish(controller_state_l_msg_);
  topicpub_joint_r_.publish(controller_state_r_msg_);
  // Publish the joint_states for the visualization  
  topicpub_joint_states_.publish(joint_states_msg_);	
	
  ROS_DEBUG("OLD Joint values of two arms are published");
  ROS_DEBUG("OLD 'joint_states' to the 'robot_state_publisher' is published");
  last_publish_time_ = ros::Time::now();
  cout<<"Joint state published "  <<endl;

    
  return;
  } // end of received message 


  } //end of publish function 


*/

bool robot_vel_integrater::one_step_forward(){

  try{
    // The delta joint values are added to current joint values 
    // realize Euler Forward:   
    for (unsigned int i_joint = 0; i_joint<m_DOF; i_joint++ ){
      Joint_pos_l_[i_joint] = Joint_pos_l_[i_joint] + Sampling_peroid_*Joint_v_l_[i_joint];
      //      cout<<"At one step forward: The last Joint_pos_r_  "<<i_joint<<" is: "<<Joint_pos_r_[i_joint]<<endl;
      Joint_pos_r_[i_joint] = Joint_pos_r_[i_joint] + Sampling_peroid_*Joint_v_r_[i_joint];
      //      cout<<"At one step forward: The received Joint_v_r_ "<<i_joint<<" is: "<<Joint_v_buffer_r_[i_joint]<<endl;
      //      cout<<"At one step forward: The stored Joint_v_r_  "<<i_joint<<" is: "<<Joint_v_r_[i_joint]<<endl;
      //      cout<<"At one step forward: The current Joint_pos_r_  "<<i_joint<<" is: "<<Joint_pos_r_[i_joint]<<endl;
      Joint_pos_[i_joint] = Joint_pos_l_[i_joint]; 
      Joint_pos_[i_joint + m_DOF] = Joint_pos_r_[i_joint]; 

    }
    //    Joint_pos_.insert(Joint_pos_.begin(), Joint_pos_l_.begin(), Joint_pos_l_.end());
    //    Joint_pos_.insert(Joint_pos_.end(), Joint_pos_r_.begin(), Joint_pos_r_.end());  
    
    //    for (unsigned int i_joint = 0; i_joint<2*m_DOF; i_joint++ ){
    //      cout<<"At one step forward: Now the Joint_pos_  "<<i_joint<<" is: "<<Joint_pos_[i_joint]<<endl;
    //    }      
  }
  catch( std::exception & e ){  
    // Maybe this catch is inproper, but let's fix it later 
    std::cout << "Standard exception: " << e.what() << std::endl;
    ROS_ERROR("Error! Unable to write 'next_waypoint'. Stopping...");
    return false;
  }
  return true;
}



int main(int argc, char **argv){

  ros::init(argc, argv, "robot_velocity_integrater_node");

  robot_vel_integrater  integrater;

  double frequency;
  
  if(integrater.n_s.hasParam("frequency")){
    integrater.n_s.getParam("frequency", frequency);
    //frequency of driver has to be much higher then controller frequency
    frequency *= 1;
  }else{
    ROS_ERROR("Parameter frequency of robot_vel_integrater is not available, shutting down node...");
    integrater.n_s.shutdown();
    integrater.params_OK = false;
    return 0;
  }

  ros::Duration min_publish_duration;
  
  if(integrater.n_s.hasParam("min_publish_duration")){
    double sec;
    integrater.n_s.getParam("min_publish_duration", sec);
    min_publish_duration.fromSec(sec);
  }else{
    ROS_ERROR("Parameter min_publish_time not available");
    integrater.params_OK = false;
    return 0;
  }

  ros::Rate loop_rate(frequency);
  
  if(!integrater.params_OK){
    ROS_ERROR("Incorrect controller parameters, shutting down node.");
    return 0;
  }

  while(integrater.n_s.ok()){
    //    if((ros::Time::now() - integrater.last_publish_time_) >= min_publish_duration){
      integrater.publish_joints();
      //    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
