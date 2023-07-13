
#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include<iostream>
#include<Eigen/Core>
#include<Eigen/SVD>

#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

// Below is the function
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


int main(){
  //Define Values
  double Kp = 1;


  try{
    franka::Robot robot("172.16.0.2"); //Now franka robot will be known as robot_state
    franka::Model model = robot.loadModel();

    //setDefaultBehavior(robot);

    franka::RobotState robot_state = robot.readOnce(); //Initial State of robot
    std::array<double,16> initial_pose = robot_state.O_T_EE; //This is our transformation Matrix

    //Map that to Eigen
    Eigen::Map<const Eigen::Matrix<double, 4, 4> > current_transformation(initial_pose.data());
    Eigen::Vector3d current_position;
    Eigen::Matrix3d current_rotation = current_transformation.block<3,3>(0,0); //We don't need to d mapping as we are making matrix

    auto translation_x = current_transformation(0,3);
    auto translation_y = current_transformation(1,3);
    auto translation_z = current_transformation(2,3);
    current_position << translation_x,translation_y,translation_z;

    std::cout << "This is transformation Matrix from Eigen " << '\n' << current_transformation << std::endl;
    //std::cout << "This is the rotation Matrix from Eigen " << '\n' << current_rotation << std::endl;
    //std::cout << "This is the translation from Eigen " << '\n' << current_position << std::endl;

    //Desired transformation
    Eigen::Vector3d desired_position;
    Eigen::Matrix3d desired_rotation;
    Eigen::Matrix4d desired_transformation;

    Eigen::Vector3d position_delta; //This is chnage in x required
    position_delta << 0.10,0,0;

    desired_position = current_position + position_delta;
    desired_rotation = current_rotation;

    desired_transformation.setIdentity();
    desired_transformation.block<3,3>(0,0) = desired_rotation;
    desired_transformation(0,3) = desired_position(0);
    desired_transformation(1,3) = desired_position(1);
    desired_transformation(2,3) = desired_position(2);

    //std::cout << "Desired Orientation " << '\n' << desired_orientation << std::endl;
    //std::cout << "Desired Position" << '\n' << desired_position << std::endl;
    std::cout << "Desired transformation" << "\n" << desired_transformation << std::endl;

    //Doing The Transformation inverse required to get deltaX
    Eigen::Matrix4d transformation_delta;
    transformation_delta.setIdentity();

    //transformation_delta = current_transformation.inverse()*desired_transformation;   //Because of way inverse works there is computation error associated
    transformation_delta(0,3) = position_delta(0);  //Placing x
    transformation_delta(1,3) = position_delta(1);  //Placing y
    transformation_delta(2,3) = position_delta(2);  //Placing z

    Eigen::Matrix3d rotation_delta = current_rotation.transpose()*desired_rotation;
    transformation_delta.block<3,3>(0,0) = rotation_delta;
    std::cout << "This is the transformation from current to desired" << '\n' << transformation_delta << std::endl;

    //Now for rotations
    Eigen::Vector4d angleaxis = rotation_matrix_to_angle_axis(rotation_delta);
    std::cout << "Angle Axis Representation" << '\n';
    std::cout << "This is the angle and axis" << '\n'<< angleaxis << std::endl;
    Eigen::Vector3d angular_velocity;
    angular_velocity[0] = angleaxis[0]*angleaxis[1];  //Assuming Unit time
    angular_velocity[1] = angleaxis[0]*angleaxis[2];  //Assuming unit time
    angular_velocity[2] = angleaxis[0]*angleaxis[3];  //Assuming Unit time
    std::cout << "This is angular_velocity" << '\n' << angular_velocity << std::endl;

    std::array<double,6> Vee = {position_delta(0),position_delta(1),position_delta(2),angular_velocity[0],angular_velocity[1],angular_velocity[2]};
    Eigen::Map<const Eigen::Matrix<double,6,1>> V_ee(Vee.data());
    std::cout << "WHole w " << V_ee << std::endl;

    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
    Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
    std::cout << "Jacobian" << '\n' << jacobian << std::endl;

    Eigen::Matrix<double,7,6> inv_jacobian = pseudoinverse(jacobian);
    std::cout << "Inverse Jacobian" << '\n' << inv_jacobian << std::endl;

    //Eigen::Matrix<double,1,7> qdot = inv_jacobian*V_ee;
    Eigen::VectorXd qdot = inv_jacobian*V_ee;
    std::cout << "This is qdot" << '\n' << qdot << std::endl;

    if (qdot[0] > 0.087) {
      qdot[0] = 0.087;
    }

    if (qdot[1] > 0.087) {
      qdot[1] = 0.087;
    }

    if (qdot[2] > 0.087) {
      qdot[2] = 0.087;
    }

    if (qdot[3] > 0.087) {
      qdot[3] = 0.087;
    }

    if (qdot[4] > 0.087) {
      qdot[4] = 0.087;
    }

    if (qdot[5] > 0.087) {
      qdot[5] = 0.087;
    }

    if (qdot[6] > 0.087) {
      qdot[6] = 0.087;
    }


  }
  // Now for Control



  //auto motion_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities
  //{

    //return new_pose;
  //};

  //robot.control(motion_callback);

  catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

}
