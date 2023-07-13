#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "examples_common.h"

std::array<double,16> initial_pose;


int main(int argc, char** argv) 
{
  if (argc != 2) 
  {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try 
  {
    franka::Robot robot(argv[1]);


    franka::RobotState initial_state = robot.readOnce();                 //Get the initial state
    franka::Model model(robot.loadModel());
        
    std::array<double,16> new_pose;
    std::array<double,7> joint_angles;
    std::array<double,42> jacob = model.zeroJacobian(franka::Frame::kEndEffector,initial_state);

    new_pose = initial_state.O_T_EE;
    joint_angles = initial_state.q;

    	std::cout << "This is x-axis in base frame- " << new_pose[12] << std::endl; 
    	std::cout << "This is y-axis in base frame- " << new_pose[13] << std::endl;
    	std::cout << "This is z-axis in base frame- " << new_pose[14] << std::endl;

	std::cout << "--------------Joint Angles----------"  << std::endl;
	std::cout << "1st Joint - " << joint_angles[0] << std::endl;
	std::cout << "2nd Joint - " << joint_angles[1] << std::endl;
	std::cout << "3st Joint - " << joint_angles[2] << std::endl;
	std::cout << "4nd Joint - " << joint_angles[3] << std::endl;
	std::cout << "5nd Joint - " << joint_angles[4] << std::endl;
	std::cout << "6st Joint - " << joint_angles[5] << std::endl;
	std::cout << "7nd Joint - " << joint_angles[6] << std::endl;
	std::cout << "--------------END Joint Angles END----------"  << std::endl;

	std::array<double,6> external_force = initial_state.O_F_ext_hat_K;
	std::cout << "Force in X-direction " << external_force[0] << std::endl;
	std::cout << "Force in Y-direction " << external_force[1] << std::endl;
	std::cout << "Force in Z-direction " << external_force[2] << std::endl;

	std::cout << "--------------Jacobian Matrix----------"  << std::endl;	
	std::cout << "x-components. " << jacob[0] <<" "<<jacob[6] <<" "<<jacob[12] <<" "<<jacob[18] <<" "<<jacob[24] <<" "<<jacob[30] <<" 		"<<jacob[36] <<" "<< std::endl;
	
	std::cout << "y-components. " << jacob[1] <<" "<<jacob[7] <<" "<<jacob[13] <<" "<<jacob[19] <<" "<<jacob[25] <<" "<<jacob[31] <<" 		"<<jacob[37] <<" "<< std::endl;

	std::cout << "z-components. " << jacob[2] <<" "<<jacob[8] <<" "<<jacob[14] <<" "<<jacob[20] <<" "<<jacob[26] <<" "<<jacob[32] <<" 		"<<jacob[38] <<" "<< std::endl;

	std::cout << " ------------- END OF Jacobian Matrix ---------------" << std::endl;

  
 }  //T
  
  catch (const franka::Exception& e) 
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
  
  return 0;
}



