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


int main(){
try{
	franka::Robot robot("172.16.0.2");
	setDefaultBehavior(robot);
    	robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

				// load the kinematics and dynamics model
				franka::Model model = robot.loadModel();
				franka::RobotState initial_state = robot.readOnce();

				/*
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
          MotionGenerator motion_generator(0.04, q_goal);
          robot.control(motion_generator);
          std::cout << "Finished moving to initial joint configuration." << std::endl;
					*/


   //Generate scuve
    std::array<double,16> initial_pose = initial_state.O_T_EE;
    double P0 = initial_pose[12];


    double tme = 0.0;

    double T1_3 = 1;  //Change me
    double T1_2 = 1;
    double T1_1 = 1;
    double T2_3 = 1;
    double T2_2 = 1;

    double A1_peak = 0.03;	//Change me
    double A2_peak = 0.03;
    double J1_peak = A1_peak/T1_3;
    double J2_peak = A2_peak/T2_3;

    std::array<double,9> T;

    T[0] = 0;   //To make it compatible with MATLAB
    T[1] = 0;   //This is the start
    T[2] = T[1] + T1_3;
    T[3] = T[2] + T1_2;
    T[4] = T[3] + T1_3;
    T[5] = T[4] + T1_1;
    T[6] = T[5] + T2_3;
    T[7] = T[6] + T2_2;
    T[8] = T[7] + T2_3;

		double A1=0; double A2=0; double A3=0;  double A4=0; 	double A5=0; 	double A6=0; 	double A7=0;
		double V1=0; double V2=0; double V3=0;  double V4=0; 	double V5=0; 	double V6=0; 	double V7=0;
		double P1=0; double P2=0; double P3=0;  double P4=0; 	double P5=0; 	double P6=0; 	double P7=0;

    double total_time = T[8];

		//Eigen::VectorXd desired_position;

		std::array<double,7002> desired_position;

    int count = 0;

		std::ofstream file_x;
		file_x.open("scurvex.txt");
		std::ofstream file_time;
		file_time.open("time.txt");

    while (tme <= T[8])
    {

      count = count + 1;

      if (tme<=T[2]){
        A1 = J1_peak*tme;
        V1 = 0.5*J1_peak*pow(tme,2);
        P1 = (1/6)*J1_peak*pow(tme,3);

        desired_position[count] = P1;

      }

      if (tme > T[2] && tme <= T[3]){
        A2 = A1;
        V2 = V1 + A1*(tme - T[2]);
        P2 = P1 + V1*tme + 0.5*J1_peak*T[2]*pow((tme - T[2]),2);

        desired_position[count] = P2;


      }

      if (tme > T[3] && tme <= T[4]){
        A3 = A2 - J1_peak*(tme - T[3]);
        V3 = V2 + A2*(tme - T[3]) - 0.5*J1_peak*pow((tme - T[3]),2);
        P3 = P2 + V2*(tme - T[3]) + (1/6)*(pow(tme - T[3],2))*(3*J1_peak*T1_3 - J1_peak*(tme - T[3]));

        desired_position[count] = P3;

      }

      if (tme > T[4] && tme <= T[5]){
        A4 = A3;
        V4 = V3;
        P4 = P3 + V3*(tme - T[4]);

        desired_position[count] = P4;

      }

      if (tme > T[5] && tme <= T[6]){
        A5 = - J2_peak*(tme - T[5]);
        V5 = V4 - 0.5*J2_peak*pow((tme - T[5]),2);
        P5 = P4 + V4*(tme - T[5]) - (1/6)*(pow(tme - T[5],3))*(J2_peak);

        desired_position[count] = P5;

      }

      if (tme > T[6] && tme <= T[7]){
        A6 = A5;
        V6 = V5 + A5*(tme - T[6]);
        P6 = P5 + V5*(tme - T[6]) - (0.5)*(pow(tme - T[6],2))*(J2_peak*T2_3);

        desired_position[count] = P6;
      }

      if (tme > T[7] && tme <= T[8]){
        A7 = A6 + J2_peak*(tme - T[7]);
        V7 = V6 + A6*(tme - T[7]) + 0.5*J2_peak*pow((tme - T[7]),2);
        P7 = P6 + V6*(tme - T[7]) + (0.5)*(pow(tme - T[7],2))*A6 + (1/6)*(pow(tme - T[7],3))*(J2_peak);

        desired_position[count] = P7;
      }

			file_x << desired_position[count] << std::endl;  //Writing to a file
			file_time << tme << std::endl;

			tme = tme + 0.001;
			std::cout << desired_position[count] << std::endl;
    }


  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
