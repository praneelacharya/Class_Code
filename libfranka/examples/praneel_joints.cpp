// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include <thread>
#include <franka/gripper.h>

class defined_gripper{
	public:
		defined_gripper(double wth, double fce, double ep_inner, double ep_outer){	//Constructor  width speed force ineer outer
		
			franka::Gripper gripper("172.16.0.2"); //This is an obj
			gripper.move(0.08,0.01); //Just as to start with open the gripper with 0.01 speed
			update(wth,fce,ep_inner,ep_outer);  //width speed force ineer outer	
		}
		
		void update(double width,double force,double inner,double outer){
			
			franka::Gripper gripper("172.16.0.2");
			gripper.grasp(width,force,inner,outer);
		}

};



/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{-0.5, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.05, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Wait 3s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(2000));

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 7> initial_position;
    double time = 0.0;
    double time_old = 0.0;
    double angle_old = 0.0;
    double angle_new = 0.0;
    double resultant_force = 0.0;

    defined_gripper(0.004,0.01,2,0.005); //width speed force ineer outer everything in SI unit
    
    robot.control([&](const franka::RobotState& robot_state,
                                             franka::Duration period) -> franka::JointPositions {
      time += period.toSec();

      if (time == 0.0) {
	time_old = time;
        initial_position = robot_state.q_d;
        angle_old = initial_position[0];
	
      }

      double deltaT = time - time_old;
      angle_new = angle_old + (0.05*deltaT);
      time_old = time;

 	franka::JointPositions output = {{angle_new, initial_position[1],
                                        initial_position[2], initial_position[3],
                                        initial_position[4], initial_position[5],
                                        initial_position[6]}};
	angle_old = angle_new;
	
	//Resultant Force//
	//franka::Robot robot
	//franka::RobotState initial_state = robot.readOnce();
	std::array<double,6> external_force = robot_state.O_F_ext_hat_K;


	resultant_force = std::pow((std::pow(external_force[0],2)+ std::pow(external_force[1],2) + std::pow(external_force[2],2)),0.5);
	std::array <double,16> obstacle_pose = robot_state.O_T_EE;
	double obstacle_angle = angle_new;
	std::cout << "Resultant Force" << resultant_force << std::endl;

      if (resultant_force >= 2) {

	//We want to record that point. Immagining we hit the center of an object //
	std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }

      return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}



