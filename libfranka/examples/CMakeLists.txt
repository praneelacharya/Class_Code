cmake_minimum_required(VERSION 3.0)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

set(EXAMPLES
  cartesian_impedance_control
  communication_test
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_velocity_motion
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  print_joint_poses
  praneel_test
  praneel_force_m
  praneel_resultant
  praneel_print_O_T_EE
  praneel_obstacle
  praneel_matrix
  praneel_obstacle_up
  praneel_matrix_inverse
  praneel_obstacle_upp
  praneel_function
  praneel_joints
  praneel_wiggle
  praneel_cubic_trajectory
  robotics_class_state_output
  robotics_class_linearization
  eigen_playground
  psudo_inverse
  praneel_cartesian_control2
  praneel_cartesian_control_v3
  praneel_cartesian_control_v4
  praneel_cartesian_control_v4_1
  praneel_scurve_position
  praneel_cartesian_control_v5
  praneel_cartesian_control_v6
  praneel_cartesian_control_v7
  praneel_cartesian_control_v8
  praneel_cartesian_control_v9
  praneel_cartesian_control_v10
  praneel_cartesian_control_v11
  praneel_sin_motion

  praneel_scurve_int_three_section
  praneel_scurve_velocity_interpol
  praneel_check_scurve
  praneel_scurve_velocity

  praneel_scurve_ofline_calculation

  praneel_quantic_trajectories
  praneel_dynamics
  praneel_move_to_initial_configration

  hussein_equilibruim
  hussein_position1
  hussein_position2
  hussein_position3
  hussein_position4
  hussein_position5
  hussein_position6
  hussein_position7
  hussein_position8
  hussein_position9
  hussein_position10
  hussein_position11
  hussein_position12
  hussein_position13
  hussein_position14
  hussein_position15
  hussein_position16
  hussein_position17
  hussein_position18
  hussein_position19
  hussein_position20
  hussein_position21
  hussein_position22
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
