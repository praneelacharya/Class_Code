// This program main goal is to output the state information about the robot into the terminal.
// End effector position [x,y,z] has been displayed in base frame. And different joint values are displayed
// Though, robot state information refers to more than just those values but for this course, we are only focusing on those.
// Thus, it is suggested not to edit this program. This just dispays the information.

#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "examples_common.h"

std::array<double,16> initial_pose;
int main() 
{
  try	
  {
    franka::Robot robot("172.16.0.2");


    franka::RobotState initial_state = robot.readOnce();                 //Get the initial state
    franka::Model model(robot.loadModel());
        
    std::array<double,16> new_pose;
    std::array<double,7> joint_angles;

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

  
 }  //T
  
  catch (const franka::Exception& e) 
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
  
  return 0;
}



