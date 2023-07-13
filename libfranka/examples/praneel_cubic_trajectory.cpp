
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

#include <franka/gripper.h>

class defined_gripper{
	public:
		defined_gripper(double wth, double fce, double ep_inner, double ep_outer){	//Constructor  width speed force ineer outer

			franka::Gripper gripper("172.16.0.2"); //This is an obj
			//gripper.move(0.08,0.01); //Just as to start with open the gripper with 0.01 speed
			update(wth,fce,ep_inner,ep_outer);  //width speed force ineer outer
		}

		void update(double width,double force,double inner,double outer){

			franka::Gripper gripper("172.16.0.2");
			gripper.grasp(width,force,inner,outer);
		}

};



/*
std::ofstream file_y;
file_y.open("positiony.txt");

std::ofstream file_z;
file_z.open("positionz.txt");

std::ofstream file_time;
file_time.open("time.txt");
*/
//////////// THIS IS A CUBIC trajectoryY -- INPUT ME THOSE VALUES BEST NOT TO LEAVE TO ZERO


/* FOR ACCELERATION = 0.2
double a5 = 0.020357;
double a4 = -0.10684;
double a3 = 0.14834;      //Thrid power Coefficient in X
double a2 = 0.00376;      // Seconds power Coefficient in X
double a1 = 0;            // First power Coefficient in X
double a0 = 0;            // 0 power Coefficient in Y
double final_time = 2.1;  // IN seconds
*/

/*//FOR ACCELERATION = 0.4
double a5 = 0.0407152;
double a4 = -0.213693;
double a3 = 0.2966924;      //Thrid power Coefficient in X
double a2 = 0.0075354;      // Seconds power Coefficient in X
double a1 = 0;            // First power Coefficient in X
double a0 = 0;            // 0 power Coefficient in Y
double final_time = 2.1;  // IN seconds
*/

/*//FOR ACCELERATION = 0.47
double a5 = 0.047840414;
double a4 = -0.2510898;
double a3 = 0.34861365;      //Thrid power Coefficient in X
double a2 = 0.00885416;      // Seconds power Coefficient in X
double a1 = 0;            // First power Coefficient in X
double a0 = 0;            // 0 power Coefficient in Y
double final_time = 2.1;  // IN seconds
*/

/*//FOR ACCELERATION = 0.7
double a5 = 0.240020;
double a4 = -0.840110;
double a3 = 0.7777117;      //Thrid power Coefficient in X
double a2 = 0.013495;      // Seconds power Coefficient in X
double a1 = 0;            // First power Coefficient in X
double a0 = 0;            // 0 power Coefficient in Y
double final_time = 1.4;  // IN seconds
*/

/*//FOR ACCELERATION = 0.8
double a5 = 0.27430920;
double a4 = -0.9601257;
double a3 = 0.8888133397;      //Thrid power Coefficient in X
double a2 = 0.0154236;      // Seconds power Coefficient in X
double a1 = 0;            // First power Coefficient in X
double a0 = 0;            // 0 power Coefficient in Y
double final_time = 1.4;  // IN seconds
*/

/*///FOR ACCELERATION = 0.9
double a5 = 0.30859785;
double a4 = -1.08014147;
double a3 = 0.9999150;      //Thrid power Coefficient in X
double a2 = 0.01735163;      // Seconds power Coefficient in X
double a1 = 0;            // First power Coefficient in X
double a0 = 0;            // 0 power Coefficient in Y
double final_time = 1.4;  // IN seconds
*/

/*//FOR ACCELERATION = 1
double a5 = 0.34288650;
double a4 = -1.20015719;
double a3 = 1.11101674;      //Thrid power Coefficient in X
double a2 = 0.019279;      // Seconds power Coefficient in X
double a1 = 0;            // First power Coefficient in X
double a0 = 0;            // 0 power Coefficient in Y
double final_time = 1.4;  // IN seconds
*/


//FOR A1 = 0.2 and A2 = 0.8  8th order
double a8 = 0.00999;
double a7 = -0.1060884;
double a6 = 0.44171494;
double a5 = -0.9060668;
double a4 = 0.930824;
double a3 = -0.43770;      //Thrid power Coefficient in X
double a2 = 0.16195;      // Seconds power Coefficient in X
double a1 = 0;            // First power Coefficient in X
double a0 = 0;            // 0 power Coefficient in Y
double final_time = 2.768;  // IN seconds



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

	std::cout << "Hello Program" << std::endl;

	std::ofstream file_x;
	file_x.open("positionx.txt");

	auto trajectoryX = [](double tme){
		//Defining way points
		// x = [0 0.1 0.2]   dv = [0 0.01 0] // t = [0  5  10]
		/* COMMENT START
		double q0=0; double q1=0; double t0=0; double t1=0; double v0=0; double v1=0;
		if (tme >= 0 && tme <=5){
			q0 = 0;   // Start Position
			q1 = 0.1; // End Position in terms of deltaX;
			t0 = 0;   // Start Time in Sec
			t1 = 5; // Final time in seconds
			v0 = 0;	 // Initial Velocity
			v1 = 0; 	 // Final Velocity
			//std::cout << "First one" << std::endl;
		}
		//if (tme >= 0 && tme <=10){
		else{
			q0 = 0.1;   // Start Position
			q1 = 0; // End Position in terms of deltaX;
			t0 = 5;   // Start Time in Sec
			t1 = 6; // Final time in seconds
			v0 = 0;	 // Initial Velocity
			v1 = 0; 	 // Final Velocity
			//std::cout << "Second one" << std::endl;

		}*/ //COMMENT END

			//double a0 = 0;  //POWER 0 Coefficient
			//double a1 = 0;  //POWER 1 Coefficient
			//double a2 = 0; //Power 2 Coefficient
			//double a3 = 0;  //Power 3 Coefficient

			//double eq = a3*(std::pow((tme-t0),3)) + a2*(std::pow((tme-t0),2)) + a1*(tme-t0) + a0;
			//double eq = a5*(std::pow((tme),5)) + a4*(std::pow((tme),4)) + a3*(std::pow((tme),3)) + a2*(std::pow((tme),2)) + a1*(tme) + a0;

double eq = a8*(std::pow((tme),8)) + a7*(std::pow((tme),7)) + a6*(std::pow((tme),6)) + a5*(std::pow((tme),5)) + a4*(std::pow((tme),4)) + a3*(std::pow((tme),3)) + a2*(std::pow((tme),2)) + a1*(tme) + a0;

			return eq;

	};

	std::array<double,16> initial_pose;
	std::array<double,16> new_pose;
	//Gripper to grasp something
	//defined_gripper(0.012,0.01,2,0.012); //width speed force ineer outer everything in SI unit
	std::cout << "gripper done" << std::endl;

	//for (int i =0; i<= 10; i=i+1){
	double time = 0.0;
	robot.control([&](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      		time += period.toSec();

      		if (time == 0.0) {
        		initial_pose = robot_state.O_T_EE_c;
      		}


		double delta_x = trajectoryX(time);
		//double delta_y = trajectoryY(time);
      		new_pose = initial_pose;
      		//new_pose[12] += delta_x;
		      //new_pose[14] += delta_y;

					new_pose[12] = new_pose[12] + delta_x;

					std::cout << new_pose[12] << std::endl;

										file_x << new_pose[12] << std::endl;  //Writing to a file
										/*
										file_y << new_pose[13] << std::endl;
										file_z << new_pose[14] << std::endl;
										file_time << time << std::endl;
										*/

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
