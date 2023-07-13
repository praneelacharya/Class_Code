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

#include "examples_common.h"
#include "pseudo_inversion.h"

Eigen::Vector4d rotation_matrix_to_angle_axis(Eigen::Matrix3d turning){

  double angle_compoent = (0.5*(turning.trace() - 1));
  double angle = acos(angle_compoent);

  //std::cout << "This c23" << turning(0,0);
  auto axis1 = (turning(2,1) - turning(1,2))/(2*sin(angle));
  auto axis2 = (turning(0,2) - turning(2,0))/(2*sin(angle));
  auto axis3 = (turning(1,0) - turning(0,1))/(2*sin(angle));

  Eigen::Vector4d angle_axis;
  angle_axis << angle,axis1,axis2,axis3;

  return angle_axis;
}

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
  position_delta << 0.2,0,0;

  double Kp = 0.5;         //This is the proportional control
  double delta_t = 0.003;
  Eigen::Vector3d error;

  try{

    franka::Robot robot("172.16.0.2");
    franka::Model model(robot.loadModel());
    franka::RobotState start_state = robot.readOnce();
    franka::Duration period;

    std::array<double,16> initial_state = start_state.O_T_EE;
    Eigen::Vector3d initial_position;
    initial_position << initial_state[12],initial_state[13],initial_state[14];

    Eigen::Vector3d desired_position;
    desired_position = initial_position + position_delta;

    //At this point we should check if we have the correct results
    std::cout << "initial_position" << '\n' << initial_position << std::endl;
    std::cout << "final_position" << '\n' << desired_position << std::endl;
    //So the above was the checking to verify it makes SENSE

    error << desired_position - initial_position;
    std::cout << "Error" << error << std::endl;

    double error_norm = error.norm();
    std::cout << "Norm" << error_norm;

    int count = 1;
    while (count < 2){
      //error_norm > 0.005

      franka::RobotState robot_state = robot.readOnce();
      std::array<double,16> current_state = robot_state.O_T_EE;
      std::array<double,7>  joint_angles = robot_state.q;

      std::array<double,6> x_dot;
      x_dot[0] = Kp*error[0];
      x_dot[1] = Kp*error[1];
      x_dot[2] = Kp*error[2];

      x_dot[3] = 0;
      x_dot[4] = 0;
      x_dot[5] = 0;

      if (x_dot[0]> 0.4){
        x_dot[0] = 0.4;
      }
      if (x_dot[1]> 0.4){
        x_dot[1] = 0.4;
      }
      if (x_dot[2]> 0.4){
        x_dot[2] = 0.4;
      }

      //Eigen::Map<const Eigen::Vector3d> Vee(x_dot.data());


      Eigen::Map<const Eigen::Matrix<double,6,1>> V_ee(x_dot.data());
      std::cout << "xdot" << V_ee;


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
      std::cout << "This is q dot" << '\n' << qdot << std::endl;




      count = count + 1;
    }

  }

  catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
}
