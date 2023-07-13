#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <fstream>

#include "examples_common.h"


double T1_3 = 1;
double T1_2 = 1;
double T1_1 = 1;
double T2_3 = T1_3;
double T2_2 = T1_2;

double A1_peak = 0.03;
double A2_peak = A1_peak;
double J1_peak = A1_peak/T1_3;
double J2_peak = A2_peak/T2_3;

double A1=0; double A2=0; double A3=0;  double A4=0; 	double A5=0; 	double A6=0; 	double A7=0;
double V1=0; double V2=0; double V3=0;  double V4=0; 	double V5=0; 	double V6=0; 	double V7=0;
double P1=0; double P2=0; double P3=0;  double P4=0; 	double P5=0; 	double P6=0; 	double P7=0;

int main(){

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

  try{

    franka::Robot robot("172.16.0.2");
    setDefaultBehavior(robot);
    franka::Model model(robot.loadModel());
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});


    std::array<double, 7> q_goal_initial = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.01, q_goal_initial);
    robot.control(motion_generator);
    std::cout << "Is at Initial Location" << std::endl;

    double time = 0.0;
    double total_time = T1_3 + T1_2 + T1_3 + T1_1 + T2_3 + T2_2 + T2_3;

    // Starting point
    franka::RobotState initial_state = robot.readOnce();
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d initial_position(initial_transform.translation());


    std::array<double, 16> initial_pose;
    double desired_position = 0;

    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose {

      time += period.toSec();

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      else if ((time > 0.0) && (time<=T[2])){
        A1 = J1_peak*time;
        V1 = 0.5*J1_peak*pow(time,2);
        P1 = (1/6)*J1_peak*pow(time,3);
        desired_position = P1;
      }

      else if ((time > T[2]) && (time <= T[3]))
      {
        A2 = A1_peak;
        V2 = (0.5*A1_peak*T1_3) + A1_peak*(time - T[2]);
        P2 = P1 + (0.5*A1_peak*T1_3)*(time - T[2]) + 0.5*A1_peak*(pow((time - T[2]),2));
        desired_position = P2;
      }

      else if ((time > T[3])){
        desired_position = 0;
      }

     std::array<double, 16> new_pose = initial_pose;

      new_pose[12] = new_pose[12];
      std::cout << new_pose[12] + desired_position << std::endl;

      if (time >= (T[2]+T[3])) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
