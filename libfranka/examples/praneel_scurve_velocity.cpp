#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

#include <fstream>

int main() {

  try {
    franka::Robot robot("172.16.0.2");
    setDefaultBehavior(robot);

    std::ofstream file_time;
    file_time.open("time.txt");
    std::ofstream file_x;
		file_x.open("velocityx.txt");

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.06, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set the joint impedance.
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

        double T1_3 = 0.078;  //Change me  Jerk 9 A1 1 A2 0.3
        double T1_2 = 0.351;
        double T1_1 = 0.683;
        double T2_3 = 0.044;
        double T2_2 = 0.706;

        double Vpeak = 0.3;

		    double J1_peak = 9;
		    double J2_peak = 9;

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

				double time = 0.0;
				double v_x = 0.0;


    robot.control([&](const franka::RobotState&,
                             franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();

			if ((time>0) && (time<T[2])){
        A1 = J1_peak*time;
        V1 = 0.5*J1_peak*pow(time,2);
        P1 = (1/6)*J1_peak*pow(time,3);

        //desired_position[count] = P1;
				//desired_velocity[count] = V1;
				v_x = V1;
			}

			if (time > T[2] && time <= T[3]){
				A2 = A1;
				V2 = V1 + A1*(time - T[2]);
				P2 = P1 + V1*time + 0.5*J1_peak*T[2]*pow((time - T[2]),2);

				//desired_position[count] = P2;
				//desired_velocity[count] = V2;
				v_x = V2;

			}

			if (time > T[3] && time <= T[4]){
				A3 = A2 - J1_peak*(time - T[3]);
				V3 = V2 + A2*(time - T[3]) - 0.5*J1_peak*pow((time - T[3]),2);
				P3 = P2 + V2*(time - T[3]) + (1/6)*(pow(time - T[3],2))*(3*J1_peak*T1_3 - J1_peak*(time - T[3]));

				//desired_position[count] = P3;
				//desired_velocity[count] = V3;
				v_x = V3;

			}

			if (time > T[4] && time <= T[5]){
				A4 = A3;
				V4 = V3;
				P4 = P3 + V3*(time - T[4]);

				//desired_position[count] = P4;
				//desired_velocity[count] = V4;
				v_x = V4;

			}

			if (time > T[5] && time <= T[6]){
				A5 = - J2_peak*(time - T[5]);
				V5 = V4 - 0.5*J2_peak*pow((time - T[5]),2);
				P5 = P4 + V4*(time - T[5]) - (1/6)*(pow(time - T[5],3))*(J2_peak);

				//desired_position[count] = P5;
				//desired_velocity[count] = V5;
				v_x = V5;

			}

			if (time > T[6] && time <= T[7]){
				A6 = A5;
				V6 = V5 + A5*(time - T[6]);
				P6 = P5 + V5*(time - T[6]) - (0.5)*(pow(time - T[6],2))*(J2_peak*T2_3);

				//desired_position[count] = P6;
				//desired_velocity[count] = V6;
				v_x = V6;
			}

			if (time > T[7] && time <= T[8]){
				A7 = A6 + J2_peak*(time - T[7]);
				V7 = V6 + A6*(time - T[7]) + 0.5*J2_peak*pow((time - T[7]),2);
				P7 = P6 + V6*(time - T[7]) + (0.5)*(pow(time - T[7],2))*A6 + (1/6)*(pow(time - T[7],3))*(J2_peak);

				//desired_position[count] = P7;
				//desired_velocity[count] = V7;
				v_x = V7;
			}

			if (v_x > Vpeak)
			{
				v_x = Vpeak;
			}

      file_time << time << std::endl;
      file_x << v_x << std::endl;  //Writing to a file

      franka::CartesianVelocities output = {{v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};
      if (time >= total_time) {
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
