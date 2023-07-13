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

int joint_limit_test(double q1,double q2,double q3,double q4,double q5,double q6,double q7)
{
	int joint_limit;

	if (q1>= 2.5 || q1<= -2.5)
	{
		joint_limit = 0;
    std::cout << "Sorry joint 1 failed" << std::endl;
	}
	else if (q2>= 1.6 || q2<= -1.6)
	{
		joint_limit = 0;
    std::cout << "Sorry joint 2 failed" << std::endl;
	}
	else if (q3>= 2.5 || q3<= -2.5)
	{
		joint_limit = 0;
    std::cout << "Sorry joint 3 failed" << std::endl;
	}
	else if (q4>= -0.06 || q4<= -3.0)
	{
		joint_limit = 0;
    std::cout << "Sorry joint 4 failed" << std::endl;
	}
	else if (q5>= 2.8 || q5<= -2.8)
	{
		joint_limit = 0;
    std::cout << "Sorry joint 5 failed" << std::endl;
	}
	else if (q6>= 3.75 || q6<= -0.01)
	{
		joint_limit = 0;
    std::cout << "Sorry joint 6 failed" << std::endl;
	}
	else if (q7>= 2.7 || q7<= -2.7)
	{
		joint_limit = 0;
    std::cout << "Sorry joint 7 failed" << std::endl;
	}
	else
	{
		joint_limit = 1;
	}

	return joint_limit;
}

int main()
{
  Eigen::Vector3d position_delta;  //This is xyz position of final end effector from initial end effector position expressed in intial end effector frame
  position_delta << 0.1,0.0,0;

  double Kp = 0.2;         //This is the proportional control freaks out for 4  an 0.8 is like maximum
	double Kpz = 0;   //0.8  0.01

	double Kd = 0;
	double Kdz = 0;   //0.5  0.02

	double Kq = 1.0;

  double time = 0;
	double P0 = 0;
  Eigen::Vector3d error;
	error << 0,0,0;
	Eigen::Vector3d error_dot;
	error_dot << 0,0,0;

  double T1_3 = 1;
  double T1_2 = 1;
  double T1_1 = 1;
  double T2_3 = T1_3;
  double T2_2 = T1_2;

  double A1_peak = 0.03;
  double A2_peak = A1_peak;
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

  double total_time = T[8];
  //double current_time = 0;

  double A1=0; double A2=0; double A3=0;  double A4=0; 	double A5=0; 	double A6=0; 	double A7=0;
  double V1=0; double V2=0; double V3=0;  double V4=0; 	double V5=0; 	double V6=0; 	double V7=0;
  double P1=0; double P2=0; double P3=0;  double P4=0; 	double P5=0; 	double P6=0; 	double P7=0;
  //double V1; double V2; double V3;
  //double P1; double P2; double P3;

  try{

    franka::Robot robot("172.16.0.2");
    franka::Model model(robot.loadModel());

    Eigen::Vector3d initial_position;
    Eigen::Vector3d desired_position;
		Eigen::Vector3d desired_velocity;

		std::ofstream file_x;
		file_x.open("positionx.txt");

		std::ofstream desired;
		desired.open("desired.txt");

		std::ofstream desired_z;
		desired_z.open("desiredz.txt");

		std::ofstream error_x;
		error_x.open("errorx.txt");

		std::ofstream error_z;
		error_z.open("errorz.txt");

    std::ofstream file_time;
    file_time.open("time.txt");

		std::ofstream velocity_x;
    velocity_x.open("velocityx.txt");

		std::ofstream errordot_x;
    errordot_x.open("errordotx.txt");

		std::ofstream errordot_z;
		errordot_z.open("errordotz.txt");



    std::array<double,16> initial_pose;

		std::array<double, 7> q_goal_initial = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
		//std::array<double, 7> q_goal_initial = {{0, -1.29533, 0, -2.50826, 0, 1.2,0.7850}};
		MotionGenerator motion_generator(0.01, q_goal_initial);
		robot.control(motion_generator);
		std::cout << "Is at Initial Location" << std::endl;

    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE;
        initial_position << initial_pose[12],initial_pose[13],initial_pose[14];

				P0 = initial_pose[12];
				desired_position[1] = initial_position[1];
				desired_position[2] = initial_position[2];

      }
      if (time<=T[2]){
        A1 = J1_peak*time;
        V1 = 0.5*J1_peak*pow(time,2);
        P1 = P0 + (1/6)*J1_peak*pow(time,3);

        desired_position[0] = P1;
        //desired_position[1] = initial_pose[13];
        //desired_position[2] = initial_pose[14];

				desired_velocity[0] = V1;
				desired_velocity[1] = 0;
        desired_velocity[2] = 0;

				//file_x << desired_position[0] << std::endl;  //Writing to a file
				//file_time << time << std::endl;
      }

      if (time > T[2] && time <= T[3]){
        A2 = A1;
        V2 = V1 + A1*(time - T[2]);
        P2 = P1 + V1*time + 0.5*J1_peak*T[2]*pow((time - T[2]),2);

        desired_position[0] = P2;
        //desired_position[1] = initial_pose[13];
        //desired_position[2] = initial_pose[14];

				desired_velocity[0] = V2;
				desired_velocity[1] = 0;
        desired_velocity[2] = 0;

				//file_x << desired_position[0] << std::endl;  //Writing to a file
				//file_time << time << std::endl;

      }

			if (time > T[3] && time <= T[4]){
				A3 = A2 - J1_peak*(time - T[3]);
				V3 = V2 + A2*(time - T[3]) - 0.5*J1_peak*pow((time - T[3]),2);
				P3 = P2 + V2*(time - T[3]) + (1/6)*(pow(time - T[3],2))*(3*J1_peak*T1_3 - J1_peak*(time - T[3]));

				desired_position[0] = P3;
				//desired_position[1] = initial_pose[13];
				//desired_position[2] = initial_pose[14];

				desired_velocity[0] = V3;
				desired_velocity[1] = 0;
        desired_velocity[2] = 0;

				//file_x << desired_position[0] << std::endl;  //Writing to a file
				//file_time << time << std::endl;
			}

			if (time > T[4] && time <= T[5]){
			  A4 = A3;
			  V4 = V3;
			  P4 = P3 + V3*(time - T[4]);

			  desired_position[0] = P4;
			  //desired_position[1] = initial_pose[13];
			  //desired_position[2] = initial_pose[14];

				desired_velocity[0] = V4;
				desired_velocity[1] = 0;
        desired_velocity[2] = 0;

			  //file_x << desired_position[0] << std::endl;  //Writing to a file
			  //file_time << time << std::endl;
			}

			if (time > T[5] && time <= T[6]){
			  A5 = - J2_peak*(time - T[5]);
			  V5 = V4 - 0.5*J2_peak*pow((time - T[5]),2);
			  P5 = P4 + V4*(time - T[5]) - (1/6)*(pow(time - T[5],3))*(J2_peak);

			  desired_position[0] = P5;
			  //desired_position[1] = initial_pose[13];
			  //desired_position[2] = initial_pose[14];

				desired_velocity[0] = V5;
				desired_velocity[1] = 0;
        desired_velocity[2] = 0;

			  //file_x << desired_position[0] << std::endl;  //Writing to a file
			  //file_time << time << std::endl;
			}

			if (time > T[6] && time <= T[7]){
			  A6 = A5;
			  V6 = V5 + A5*(time - T[6]);
			  P6 = P5 + V5*(time - T[6]) - (0.5)*(pow(time - T[6],2))*(J2_peak*T2_3);

			  desired_position[0] = P6;
			  //desired_position[1] = initial_pose[13];
			  //desired_position[2] = initial_pose[14];

				desired_velocity[0] = V6;
				desired_velocity[1] = 0;
        desired_velocity[2] = 0;

			  //file_x << desired_position[0] << std::endl;  //Writing to a file
			  //file_time << time << std::endl;
			}

			if (time > T[7] && time <= T[8]){
			  A7 = A6 + J2_peak*(time - T[7]);
			  V7 = V6 + A6*(time - T[7]) + 0.5*J2_peak*pow((time - T[7]),2);
			  P7 = P6 + V6*(time - T[7]) + (0.5)*(pow(time - T[7],2))*A6 + (1/6)*(pow(time - T[7],3))*(J2_peak);

			  desired_position[0] = P7;
			  //desired_position[1] = initial_pose[13];
			  //desired_position[2] = initial_pose[14];

				desired_velocity[0] = V7;
				desired_velocity[1] = 0;
        desired_velocity[2] = 0;

			  //file_x << desired_position[0] << std::endl;  //Writing to a file
			  //file_time << time << std::endl;
			}

      //franka::RobotState robot_state = robot.readOnce();
        std::array<double,16> current_state = robot_state.O_T_EE;
        std::array<double,7>  joint_velc = robot_state.dq;
				Eigen::Map<const Eigen::Matrix<double,7,1>> joint_velocity(joint_velc.data());

				std::array<double,42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
				Eigen::Map<const Eigen::Matrix<double,6,7>> jacobian(jacobian_array.data());

				Eigen::Matrix<double,6,1> current_twist = jacobian*joint_velocity;

		    //Eigen::Map<const Eigen::Matrix<double, 6, 1> > current_twist(inbuilt_twist.data());


        Eigen::Vector3d current_position;
        current_position << current_state[12],current_state[13],current_state[14];

				Eigen::Vector3d current_velocity;
        current_velocity << current_twist[0],current_twist[1],current_twist[2];


        error << desired_position - current_position;
				error_dot << desired_velocity - current_velocity;
				//error_dot = error_dot.abs();
        //std::cout << "Error" << error << std::endl;

				file_x << current_position[0] << std::endl;  //Writing to a file
				desired << desired_position[0] << std::endl;
				desired_z << desired_position[2] << std::endl;
        file_time << time << std::endl;
				error_x << error[0] << std::endl;
				error_z << error[2] << std::endl;
				errordot_x << error_dot[0] << std::endl;
				errordot_z << error_dot[2] << std::endl;
				velocity_x << desired_velocity[0] << std::endl;


        std::array<double,6> x_dot;
        x_dot[0] = Kp*error[0] + Kd*error_dot[0];
        x_dot[1] = 0;
        x_dot[2] = Kpz*error[2] + Kdz*error_dot[2];

        x_dot[3] = 0;
        x_dot[4] = 0;
        x_dot[5] = 0;

        //if (x_dot[0]> 0.4){
          //x_dot[0] = 0.4;
        //}
        //if (x_dot[1]> 0.4){
          //x_dot[1] = 0.4;
        //}
        //if (x_dot[2]> 0.4){
          //x_dot[2] = 0.4;
        //}

        //Eigen::Map<const Eigen::Vector3d> Vee(x_dot.data());


        Eigen::Map<const Eigen::Matrix<double,6,1>> V_ee(x_dot.data());
        //std::cout << "xdot" << V_ee;

        //Taking pseudo Inverse of Jacobian MatrixXd
        Eigen::MatrixXd jacobian_inverse;
        pseudoInverse(jacobian, jacobian_inverse);

        Eigen::Matrix<double,7,1> qdot = jacobian_inverse*V_ee;

				qdot = Kq*qdot;

				if (qdot[0] > 0.08) {
					qdot[0] = 0.08;
				}

				if (qdot[1] > 0.08) {
					qdot[1] = 0.08;
				}

				if (qdot[2] > 0.08) {
					qdot[2] = 0.08;
				}

				if (qdot[3] > 0.08) {
					qdot[3] = 0.08;
				}

				if (qdot[4] > 0.08) {
					qdot[4] = 0.08;
				}

				if (qdot[5] > 0.08) {
					qdot[5] = 0.08;
				}

				if (qdot[6] > 0.08) {
					qdot[6] = 0.08;
				}

        //std::cout << "This is q dot desired" << '\n' << qdot << std::endl;

        //franka::JointVelocities velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        franka::JointVelocities velocities = {{qdot[0],qdot[1],qdot[2],qdot[3],qdot[4],qdot[5],qdot[6]}};

        double error_norm = error.norm();
        std::cout << "Norm" << error_norm << std::endl;

        if (time >= (total_time+15)) {
          std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
          return franka::MotionFinished(velocities);
        }
        return velocities;
    });

    }


  catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
}
