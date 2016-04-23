# include <robotPaddlingControl/robotPaddlingController.h>
# include <rostopicCommunication/rostopicCommunication.h>
#include <signal.h>

void my_function(int sig){ // can be called asynchronously
  int return_value = EXIT_SUCCESS;
  exit(return_value);
  
}

int main(int argc, char **argv)
{
  // Register signals 
  signal(SIGINT, my_function); 

  int return_value;


  ros::init(argc, argv, "robotPaddlingControlNode");
  
  robotPaddlingController robotController;

  double frequency(100.0);

  ros::Rate loop_rate(frequency);

  while(robotController.isOK() ){
    robotController.publishJointVelocities();    
    ros::spinOnce();
    loop_rate.sleep();
  }
      
  return_value = EXIT_FAILURE;
  exit(return_value);
 
}



