#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

int main() {

  try {
    franka::Robot robot("172.16.0.2");
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
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


    double time = 0.0;
    double v_x = 0.0;

    double first_a3 = -4.393055;
    double first_a2 = 3.2007416;
    double first_a1 = -0.106502;
    double first_a0 = 0;

    double second_a1 = 0.00;
    double second_a0 = 0.199989;

    double third_a3 = 3.6049901;
    double thrid_a2 = -24.2579;
    double third_a1 = 53.78500;
    double thrid_a0 = -39.17794;

    double a = 0.486;
    double b = 1.992;
    double c = 2.493;

    double final_time = c;  // IN seconds  \\\ /// \\\ /// THIS IS IMPORTANT

    robot.control([=, &time, &v_x](const franka::RobotState&,
                             franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();

      if (time >= 0 && time < a){

        v_x = first_a3*(std::pow((time),3)) + first_a2*(std::pow((time),2)) + first_a1*time + first_a0;

      }
      else if (time >= a && time < b) {

        v_x = second_a1*time + second_a0;

      }
      else if (time >= b && time <= c) {

        v_x = third_a3*(std::pow((time),3)) + thrid_a2*(std::pow((time),2)) + third_a1*time + thrid_a0;
      }

      franka::CartesianVelocities output = {{v_x, 0.0, 0.0, 0.0, 0.0, 0.0}};
      if (time >= final_time) {
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
