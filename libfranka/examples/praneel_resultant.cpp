#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "examples_common.h"

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
    setDefaultBehavior(robot);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    
    robot.automaticErrorRecovery();  //Automatic Error Recovery

    franka::RobotState initial_state = robot.readOnce();                 //Get the initial state
    franka::Model model(robot.loadModel());

    //std::array<double,16> initial_pose = initial_state.O_T_EE;
 
    //for (int i = 0; i<=16; i++)
    //{ 
    //	std::cout << initial_pose[i] << std::endl;
    //}
     
    double time = 0.0;
    std::array<double,16> new_pose;
    //std::array<double,16> initial_pose;
    //} //T
 
    // Section of Traj equation calculation//

    //int time_scaling = 10; //Give the final time to rech to desired pos here;
    constexpr int t_s = 10;	    
    constexpr double delx = -0.10; //In cm 10 cm = 0.1 m
    constexpr double dely = 0.10;
    constexpr double delz = 0;
    //constexpr double kRadius = 10;
    auto motion_callback = [&time, &new_pose](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose
    {
	time = time + period.toSec();
	//int ts = 10;
	if (time == 0.0)
		{

			initial_pose = robot_state.O_T_EE_c;
                        new_pose = initial_pose;
			std::cout << "At t = 0 pose is " << std::endl;
			std::cout << initial_pose[12] << std::endl;
			//std::array<double,16> new_pose = initial_pose;
		}
	double  eq = 6*(pow((time/t_s),5)) - 15*(pow((time/t_s),4)) + 10*(pow((time/t_s),3));

	//double angle = M_PI/4 * (1 - std::cos(M_PI /5.0 * time));
	//double delta_x = kRadius*std::sin(angle); 
	double delta_x = delx*eq;   
	double delta_y = dely*eq;
	double delta_z = delz*eq;
	
	new_pose = initial_pose;
	new_pose[12] = new_pose[12] + delta_x;
	new_pose[13] = new_pose[13] + delta_y;
	new_pose[14] = new_pose[14] + delta_z;

	std::cout << "First one " << std::endl;
	std::cout << new_pose[12] << std::endl;
	if (time >= 10.0)
	{
		std::cout <<"END MOTION with this position" <<std::endl;
 		std::cout << initial_pose[12] << std::endl;               
		return franka::MotionFinished(new_pose);
	}
	return new_pose;       
    };

  robot.control(motion_callback);

  } //T
  
  
  
  catch (const franka::Exception& e) 
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
  
  std::cout << "MAIN IS RUNNING" << std::endl;
  return 0;
}

