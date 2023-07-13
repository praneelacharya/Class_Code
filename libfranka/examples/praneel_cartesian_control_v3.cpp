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
  position_delta << 0.3,0,0;

  double time_max = 10;  //This is the time under loop
  double Kp = 50;         //This is the proportional control
  double time_old;
  double time_new;
  double delta_t = 0.003;

  try{

    franka::Robot robot("172.16.0.2");
    franka::Model model(robot.loadModel());
    franka::RobotState start_state = robot.readOnce();
    franka::Duration period;

    time_old = period.toSec();

    std::array<double,16> initial_state = start_state.O_T_EE;

    //This is our transformation Matrix WE CARE ABOUT INITIAL TRANSFORM BECAUSE WE NEED ROTATION MATRIX
    Eigen::Map<const Eigen::Matrix<double, 4, 4> > initial_transformation(initial_state.data());
    Eigen::Vector3d initial_position;
    Eigen::Matrix3d initial_rotation = initial_transformation.block<3,3>(0,0); //We don't need to d mapping as we are making matrix
    initial_position << initial_transformation(0,3),initial_transformation(1,3),initial_transformation(2,3);

    Eigen::Vector3d desired_position;
    Eigen::Matrix3d desired_rotation;
    desired_position = initial_position + position_delta;
    desired_rotation = initial_rotation;   //In this case desired rotation is same as initial rotation
    Eigen::Matrix4d desired_transformation;
    desired_transformation.setIdentity();
    desired_transformation.block<3,3>(0,0) = desired_rotation;
    desired_transformation(0,3) = desired_position(0);
    desired_transformation(1,3) = desired_position(1);
    desired_transformation(2,3) = desired_position(2);

    //At this point we should check if we have the correct results
    std::cout << "initial_transformation" << '\n' << initial_transformation << std::endl;
    std::cout << "final_transformation" << '\n' << desired_transformation << std::endl;
    //So the above was the checking to verify it makes SENSE

    int count = 1;
    //From here loop can start
    while (count<100)
    {
      time_new = period.toSec();
      //std::cout << "Time old " << time_new << std::endl;

      franka::RobotState robot_state = robot.readOnce();
      std::array<double,16> current_state = robot_state.O_T_EE;
      std::array<double,7>  joint_angles = robot_state.q;

      Eigen::Map<const Eigen::Matrix<double, 4, 4> > current_transformation(current_state.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1> > current_joint_angles(joint_angles.data());
      //current_joint_angles << joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4],joint_angles[5],joint_angles[6];

      std::cout << "This is current joint angles " << '\n' << current_joint_angles << std::endl;

      //Calculating delta_transformation
      Eigen::Matrix<double, 4, 4> delta_transformation;
      delta_transformation = current_transformation.inverse()*desired_transformation;

      //std::cout << "delta_transformation" << '\n' << delta_transformation << std::endl;
      //std::cout << delta_transformation(0,3);
      //check if delta_transformation makes SENSE

      std::array<double,3> x_dot;
      x_dot[0] = Kp*delta_transformation(0,3);
      x_dot[1] = Kp*delta_transformation(1,3);
      x_dot[2] = Kp*delta_transformation(2,3);

      Eigen::MatrixXd delta_rotation;
      delta_rotation = delta_transformation.block<3,3>(0,0);

      Eigen::Vector4d angleaxis = rotation_matrix_to_angle_axis(delta_rotation);
      Eigen::Vector3d angular_velocity;
      angular_velocity[0] = angleaxis[0]*angleaxis[1];  //Assuming Unit time
      angular_velocity[1] = angleaxis[0]*angleaxis[2];  //Assuming unit time
      angular_velocity[2] = angleaxis[0]*angleaxis[3];  //Assuming Unit time
      //std::cout << "This is angular_velocity" << '\n' << angular_velocity << std::endl;

      std::array<double,6> Vee = {x_dot[0],x_dot[1],x_dot[2],angular_velocity[0],angular_velocity[1],angular_velocity[2]};
      if (Vee[0]> 0.5){
        Vee[0] = 0.5;
      }
      if (Vee[1]> 0.5){
        Vee[1] = 0.5;
      }
      if (Vee[2]> 0.5){
        Vee[2] = 0.5;
      }
      if (Vee[3]> 0.08){
        Vee[3] = 0.08;
      }
      if (Vee[4]> 0.08){
        Vee[4] = 0.08;
      }
      if (Vee[5]> 0.08){
        Vee[5] = 0.08;
      }

      Eigen::Map<const Eigen::Matrix<double,6,1>> V_ee(Vee.data());
      //std::cout << "This is Twist" << '\n' << V_ee << std::endl;

      //Taking pseudo Inverse of Jacobian MatrixXd
      std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());

      Eigen::MatrixXd jacobian_inverse;
      pseudoInverse(jacobian, jacobian_inverse);
      //std::cout<< "This is jacobian Inverse" <<'\n' <<jacobian_inverse <<std::endl;



      Eigen::Matrix<double,7,1> qdot = jacobian_inverse*V_ee;
      if (qdot[0] > 0.174) {
        qdot[0] = 0.174;
      }

      if (qdot[1] > 0.174) {
        qdot[1] = 0.174;
      }

      if (qdot[2] > 0.174) {
        qdot[2] = 0.174;
      }

      if (qdot[3] > 0.174) {
        qdot[3] = 0.174;
      }

      if (qdot[4] > 0.174) {
        qdot[4] = 0.174;
      }

      if (qdot[5] > 0.174) {
        qdot[5] = 0.174;
      }

      if (qdot[6] > 0.174) {
        qdot[6] = 0.174;
      }

      std::cout << "This is q dot" << '\n' << qdot << std::endl;


      //Now qdot to q
      Eigen::VectorXd q;
      q = current_joint_angles + qdot*delta_t;
      //std::cout << '\n' << q << std::endl;

      int joint_limit = joint_limit_test(q[0],q[1],q[2],q[3],q[4],q[5],q[6]);

      std::array<double,7> q_goal = {{q[0],q[1],q[2],q[3],q[4],q[5],q[6]}};
      std::cout << "Final Joint angles" << std::endl;
      std::cout << q_goal[0] << std::endl;
      std::cout << q_goal[1] << std::endl;
      std::cout << q_goal[2] << std::endl;
      std::cout << q_goal[3] << std::endl;
      std::cout << q_goal[4] << std::endl;
      std::cout << q_goal[5] << std::endl;
      std::cout << q_goal[6] << std::endl;

      if (joint_limit == 1)
    {
          std::cout << "!!!!!   Joint Limit you gave are within accepted range    !!!" << std::endl;
          //Once joint limit is within range we execute the motion //
          //MotionGenerator motion_generator(0.1,q_goal);
          //robot.control(motion_generator);
          //std::cout << "Motion done" << std::endl;
    }
      else
    {
          std::cout << "Sorry!! Joint Limit you gave are out of range / try new ones" << std::endl;
    }



      time_old = period.toSec();
      //std::cout << "Time old " << time_old << std::endl;
      count = count + 1;


    }

  }

  catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }
}
