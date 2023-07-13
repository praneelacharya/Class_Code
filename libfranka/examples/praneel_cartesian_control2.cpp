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


int main()
{
  Eigen::Vector3d position_delta;  //This is xyz position of final end effector from initial end effector position expressed in intial end effector frame
  position_delta << 0.1,0,0;

  double time_max = 10;  //This is the time under loop
  double Kp = 1;         //This is the proportional control

  try{

    franka::Robot robot("172.16.0.2");
    franka::Model model = robot.loadModel();
    franka::RobotState start_state;

    robot.automaticErrorRecovery();

    std::array<double,16> initial_state = start_state.O_T_EE;

    //This is our transformation Matrix WE CARE ABOUT INITIAL TRANSFORM BECAUSE WE NEED ROTATION MATRIX
    Eigen::Map<const Eigen::Matrix<double, 4, 4> > initial_transformation(initial_state.data());
    Eigen::Vector3d initial_position;
    Eigen::Matrix3d initial_rotation = initial_transformation.block<3,3>(0,0); //We don't need to d mapping as we are making matrix
    initial_position << initial_transformation(0,3),initial_transformation(1,3),initial_transformation(2,3);

    Eigen::Vector3d desired_position;
    Eigen::Matrix3d desired_rotation;
    desired_position = initial_position + position_delta;
    desired_rotation = initial_rotation;
    Eigen::Matrix4d desired_transformation;
    desired_transformation.setIdentity();
    desired_transformation.block<3,3>(0,0) = desired_rotation;
    desired_transformation(0,3) = desired_position(0);
    desired_transformation(1,3) = desired_position(1);
    desired_transformation(2,3) = desired_position(2);

    //At this point we should check if we have the correct results
    //std::cout << "initial_transformation" << '\n' << initial_transformation << std::endl;
    //std::cout << "final_transformation" << '\n' << desired_transformation << std::endl;

    double time = 0.0;

    robot.control(
        [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
          time += period.toSec();

          robot.automaticErrorRecovery();

          std::array<double,16> current_state = robot_state.O_T_EE; //This is our transformation Matrix
          Eigen::Map<const Eigen::Matrix<double, 4, 4> > current_transformation(current_state.data());

          Eigen::Matrix4d delta_transformation = current_transformation.inverse()*desired_transformation;
          // CHECK THIS OUTPUT IF IT MAKES SENSE

          Eigen::Vector3d delta_position;
          Eigen::Matrix3d delta_rotation;
          delta_position << delta_transformation(0,3),delta_transformation(1,3),delta_transformation(2,3);
          delta_rotation = delta_transformation.block<3,3>(0,0);

          double angle_compoent = (0.5*(delta_rotation.trace() - 1));
          double angle = acos(angle_compoent);

          //std::cout << "This c23" << turning(0,0);
          auto axis1 = (delta_rotation(2,1) - delta_rotation(1,2))/(2*sin(angle));
          auto axis2 = (delta_rotation(0,2) - delta_rotation(2,0))/(2*sin(angle));
          auto axis3 = (delta_rotation(1,0) - delta_rotation(0,1))/(2*sin(angle));

          Eigen::Vector4d angleaxis;
          Eigen::Vector3d angular_velocity;
          angleaxis << angle,axis1,axis2,axis3;

          angular_velocity[0] = angleaxis[0]*angleaxis[1];  //Assuming Unit time
          angular_velocity[1] = angleaxis[0]*angleaxis[2];  //Assuming unit time
          angular_velocity[2] = angleaxis[0]*angleaxis[3];  //Assuming Unit time

          std::array<double,6> Vee = {Kp*delta_position(0),Kp*delta_position(1),Kp*delta_position(2),angular_velocity[0],angular_velocity[1],angular_velocity[2]};
          if (Vee[0]> 0.05){
            Vee[0] = 0.05;
          }
          if (Vee[1]> 0.05){
            Vee[1] = 0.05;
          }
          if (Vee[2]> 0.05){
            Vee[2] = 0.05;
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
          std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
          Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());

          Eigen::MatrixXd jacobian_inverse;
          pseudoInverse(jacobian,jacobian_inverse);


          //std::cout << "This is the jacobian inverse" << '\n' << jacobian_inverse << std::endl;
          //Eigen::Matrix<double,7,6> inv_jacobian = jacobian.inverse();
          //Eigen::VectorXd qdot = inv_jacobian*V_ee;

          //franka::JointVelocities velocities = {{qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5], qdot[6]}};
          franka::JointVelocities velocities = {{0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

          if (time >= 10 * time_max) {
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
