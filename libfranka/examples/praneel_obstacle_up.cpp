#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "examples_common.h"

std::array<double,16> initial_pose;




//Trajectory coefficient Calculations based on quantic polynomial//



//User Defined Function

void move_back()
{
	//Initilization
        
        std::cout << "I am IN second program" << std::endl;
	franka::Robot robot("172.16.0.2");
	setDefaultBehavior(robot);
        robot.automaticErrorRecovery();  //Automatic Error Recovery
	
        constexpr double kRadius = 0.3;
        std::array<double, 16> initial_pose;
        double time = 0.0;
        robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      	time += period.toSec();

      	if (time == 0.0) {
        	initial_pose = robot_state.O_T_EE_c;
      	}

      	double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      	double delta_x = -kRadius * std::sin(angle);
      	//double delta_z = kRadius * (std::cos(angle) - 1);

      	std::array<double, 16> new_pose = initial_pose;
      	new_pose[12] += delta_x;
      	//new_pose[14] += delta_z;

      	if (time >= 10.0) {
        	std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        	return franka::MotionFinished(new_pose);
      	}
      	return new_pose;
        });
}
// End of User Defined Function


int main(int argc, char** argv) 
{
  constexpr double delx = 0.10;
  try 
  {
    franka::Robot robot("172.16.0.2");
    setDefaultBehavior(robot);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 0.0, 0.0, 0.0}}, {{20.0, 20.0, 20.0, 1.0, 1.0, 1.0}},
        {{0.0, 0.0, 0.0, 0.0, 0.0, 10.0}}, {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}});
    
    robot.automaticErrorRecovery();  //Automatic Error Recovery

    franka::RobotState initial_state = robot.readOnce();                 //Get the initial state
    franka::Model model(robot.loadModel());

    //Move to Start Position //
    std::array<double,7> q_goal = {{0,-M_PI_4,0,-3*M_PI_4,0,M_PI_2,M_PI_4}};
    MotionGenerator motion_generator(0.1,q_goal);
    robot.control(motion_generator);

    //std::array<double,16> initial_pose = initial_state.O_T_EE;
     
    double time = 0.0;
    std::array<double,16> new_pose;
    //std::array<double,16> initial_pose;
    //} //T
 
    // Section of Traj equation calculation//

    //int time_scaling = 10; //Give the final time to rech to desired pos here;
    //constexpr int t_s = 10;	    
    //constexpr double delx = 0.10; //In cm 10 cm = 0.1 m
    //constexpr double dely = 0.10;
    //constexpr double delz = 0;
    //constexpr double kRadius = 10;
    auto motion_callback = [&time, &new_pose](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose
    {
	time = time + period.toSec();
	int t_s = 5;
	if (time == 0.0)
		{

			initial_pose = robot_state.O_T_EE_c;
                        new_pose = initial_pose;
			std::cout << " bgs t = " << time << "pose is" << initial_pose[12] << std::endl;
			//std::cout << initial_pose[12] << std::endl;
			//std::array<double,16> new_pose = initial_pose;
		}

	std::cout << "-bgs t = " << time << " pose is - " << initial_pose[12] << std::endl;

	//double  eq = 6*(pow((time/t_s),5)) - 15*(pow((time/t_s),4)) + 10*(pow((time/t_s),3));
 
	double eq = +0.008*std::pow(time,3)/10;
	double delta_x = eq;	
	//double delta_x = delx*eq;   
	//double delta_y = dely*eq;
	//double delta_z = delz*eq;
	
	new_pose = initial_pose;
	new_pose[12] = new_pose[12] + delta_x;
	//new_pose[13] = new_pose[13] + delta_y;
	//new_pose[14] = new_pose[14] + delta_z;

	//std::cout << "First one " << std::endl;
	//std::cout << new_pose[12] << std::endl;
        
	franka::CartesianPose desired_pose = new_pose;
	if (time >= 3.0)
	{
		std::cout <<"END MOTION with this position" <<std::endl;
 		std::cout << initial_pose[12] << std::endl;               
		return franka::MotionFinished(desired_pose);
	}
	return desired_pose;       
    };

  robot.control(motion_callback);

  } //T
  
  
  
  catch (const franka::Exception& e) 
  {
    std::cout << e.what() << std::endl;
    franka::Robot robot("172.16.0.2");
    std::cout << "Recovery" << std::endl;
    robot.automaticErrorRecovery();  //Automatic Error Recovery   
    std::array<double,16> current_pose = robot.readOnce().O_T_EE;
    std::cout << "Obstacle [x,y,z] " << " [" << current_pose[12] << "," << current_pose[13] << "," << current_pose[12] << "]"<< std::endl;
    //move_back();
    return -1;
  }
  
  std::cout << "MAIN IS RUNNING" << std::endl;
  return 0;
} // main


