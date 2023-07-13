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

std::array<double,16> initial_pose;


int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try
  {
    franka::Robot robot(argv[1]);


    franka::RobotState initial_state = robot.readOnce();                 //Get the initial state
    franka::Model model(robot.loadModel());

    std::array<double,16> new_pose;
    std::array<double,7> joint_angles;
    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, initial_state);
    //std::array<double,42> jacob = model.zeroJacobian(franka::Frame::kEndEffector,initial_state);

    new_pose = initial_state.O_T_EE;
    joint_angles = initial_state.q;


    std::array<double, 7> coriolis_array = model.coriolis(initial_state);   //This gives all the RobotState
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis_vector(coriolis_array.data());

    std::array<double,49> mass_array = model.mass(initial_state);
    Eigen::Map<const Eigen::Matrix<double,7,7>> mass_matrix(mass_array.data());

    std::array<double,7> gravity_array = model.gravity(initial_state);
    Eigen::Map<const Eigen::Matrix<double,7,7>> gravity_vector(gravity_array.data());


    std::array<double,7> torque_array = initial_state.tau_J;
    Eigen::Map<const Eigen::Matrix<double,7,1>> torque_vector(torque_array.data());

    std::cout << "Mass MATRIX" << mass_matrix << std::endl;
    std::cout << "coriolis"    << coriolis_vector<< std::endl;
    std::cout << "Gravity"     << gravity_vector << std::endl;
    std::cout << "torqure"     << torque_vector  << std::endl;


    Eigen::Map<const Eigen::Matrix<double,7,1>> joint_vectors(joint_angles.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());


    Eigen::Matrix<double,6,1> twist = jacobian*joint_vectors;

    std::cout << "Twist" << twist << std::endl;

    std::array<double, 6> inbuilt_twist = initial_state.O_dP_EE_c;
    Eigen::Map<const Eigen::Matrix<double, 6, 1> > ttoo(inbuilt_twist.data());
    std::cout << "Twist" << ttoo << std::endl;


 }  //T

  catch (const franka::Exception& e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
