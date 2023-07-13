
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

/*
std::ofstream file_y;
file_y.open("positiony.txt");

std::ofstream file_z;
file_z.open("positionz.txt");

std::ofstream file_time;
file_time.open("time.txt");
*/
// Three sections of position //

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

        std::array<double, 7> q_goal_initial = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.01, q_goal_initial);
        robot.control(motion_generator);
        std::cout << "Is at Initial Location" << std::endl;

	std::ofstream file_x;
	file_x.open("positionx.txt");

  double first_a3 = 0.0013923;
  double first_a2 = 0.0058227;
  double first_a1 = 0;
  double first_a0 = 0;

  double second_a1 = 0.06;
  double second_a0 = -0.09000;

  double third_a3 = 0.00139;
  double thrid_a2 = -0.035060;
  double third_a1 = 0.286187;
  double thrid_a0 = -0.52288;

  double a = 3;
  double b = 4;
  double c = 7;

  double final_time = c;  // IN seconds  \\\ /// \\\ /// THIS IS IMPORTANT

	std::array<double,16> initial_pose;
	std::array<double,16> new_pose;
  double eq = 0;

	double time = 0.0;
	robot.control([&](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      		time += period.toSec();

      		if (time == 0.0) {
        		initial_pose = robot_state.O_T_EE_c;
      		}

          if (time >= 0 && time < a){

            eq = first_a3*(std::pow((time),3)) + first_a2*(std::pow((time),2)) + first_a1*time + first_a0;

          }
          else if (time >= a && time < b) {

            eq = second_a1*time + second_a0;
          }
          else if (time >= b && time <= c) {

            eq = third_a3*(std::pow((time),3)) + thrid_a2*(std::pow((time),2)) + third_a1*time + thrid_a0;
          }


		//double delta_x = trajectoryX(time);
    new_pose = initial_pose;
    new_pose[12] = new_pose[12] + eq;

		std::cout << new_pose[12] << std::endl;

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
