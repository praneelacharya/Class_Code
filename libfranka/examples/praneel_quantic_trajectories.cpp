#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include<Eigen/Core>
#include<Eigen/SVD>

#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <fstream>

#include "examples_common.h"
#include "pseudo_inversion.h"


//FOR A1 = 0.2 and A2 = 0.8  8th order
double a5 = 0.0000823;
double a4 = -0.00144080;
double a3 = 0.00666;      //Thrid power Coefficient in X
double a2 = 0.00057373;      // Seconds power Coefficient in X
double a1 = 0;            // First power Coefficient in X
double a0 = 0;            // 0 power Coefficient in Y
double final_time = 7;  // IN seconds

//////////////////////////// END OF INPUT
int main(){
try{
	franka::Robot robot("172.16.0.2");
	setDefaultBehavior(robot);
    	robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
          MotionGenerator motion_generator(0.04, q_goal);
          robot.control(motion_generator);
          std::cout << "Finished moving to initial joint configuration." << std::endl;


	std::ofstream file_x;
	file_x.open("positionx.txt");

	std::array<double,16> initial_pose;
	std::array<double,16> new_pose;

  double delta_x = 0;
	double time = 0.0;
	robot.control([&](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      		time += period.toSec();

      		if (time == 0.0) {
        		initial_pose = robot_state.O_T_EE_c;
      		}
          else {
            delta_x = a5*(std::pow((time),5)) + a4*(std::pow((time),4)) + a3*(std::pow((time),3)) + a2*(std::pow((time),2))+ a1*time + a0;
          }

      		new_pose = initial_pose;
					new_pose[12] = new_pose[12] + delta_x;
					std::cout << new_pose[12] << std::endl;

					file_x << new_pose[12] << std::endl;  //Writing to a file

      		if (time >= final_time) {
        		std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        		return franka::MotionFinished(new_pose);
      		}
      		return new_pose;
    	});//}//For loop
  }//Try
	catch (const franka::Exception& e) {
    		std::cout << e.what() << std::endl;
    		return -1;
	}
	return 0;
}
