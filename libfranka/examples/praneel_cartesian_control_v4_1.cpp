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

  double Kp = 0.5;         //This is the proportional control
  double time = 0;
  Eigen::Vector3d error;


  try{

    franka::Robot robot("172.16.0.2");
    franka::Model model(robot.loadModel());

    Eigen::Vector3d initial_position;
    Eigen::Vector3d desired_position;

		std::ofstream file;
		file.open("position.txt");

		std::ofstream file_time;
		file_time.open("time.txt");

    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();

      if (time == 0.0) {
        std::array<double,16> initial_pose = robot_state.O_T_EE;
        initial_position << initial_pose[12],initial_pose[13],initial_pose[14];
        desired_position = initial_position + position_delta;

				file << initial_pose[12] << std::endl;
				file_time << time << std::endl;
      }

    //franka::RobotState robot_state = robot.readOnce();
      std::array<double,16> current_state = robot_state.O_T_EE;
      std::array<double,7>  joint_angles = robot_state.q;

      Eigen::Vector3d current_position;
      current_position << current_state[12],current_state[13],current_state[14];

			file << current_state[12] << std::endl;  //Writing to a file
			file_time << time << std::endl;          //Writing to a file

      error << desired_position - current_position;
      std::cout << "Error" << error << std::endl;

      std::array<double,6> x_dot;
      x_dot[0] = Kp*error[0];
      x_dot[1] = Kp*error[1];
      x_dot[2] = Kp*error[2];

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
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());

      Eigen::MatrixXd jacobian_inverse;
      pseudoInverse(jacobian, jacobian_inverse);

      Eigen::Matrix<double,7,1> qdot = jacobian_inverse*V_ee;
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
      std::cout << "This is q dot desired" << '\n' << qdot << std::endl;

      //franka::JointVelocities velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      franka::JointVelocities velocities = {{qdot[0],qdot[1],qdot[2],qdot[3],qdot[4],qdot[5],qdot[6]}};

      double error_norm = error.norm();
      std::cout << "Norm" << error_norm;

      if (error_norm <= 0.001) {
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
