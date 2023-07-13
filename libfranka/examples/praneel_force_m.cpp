#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "examples_common.h"

std::array<double,16> initial_pose;



//User Defined Function

void move_back(int delx)
{
	//Initilization


	franka::Robot robot("172.16.0.2");
	setDefaultBehavior(robot);

	franka::RobotState initial_state = robot.readOnce();

	initial_pose = initial_state.O_T_EE_c;

	constexpr int t_s = 10;
	constexpr double kRadius = 10;
	//constexpr double delx = -0.10;
	double time = 0.0;
	std::array<double, 16> new_pose;
	auto motion_callback = [&time, &new_pose,&delx](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianPose
	{
		time = time + period.toSec();
		
		std::cout << "Time  " << time << std::endl;  
		//int ts = 10;

		double angle = M_PI/4 * (1 - std::cos(M_PI /5.0 * time));
		double delta_x = kRadius*std::sin(angle); 
		//double  eq = 6*(pow((time/t_s),5)) - 15*(pow((time/t_s),4)) + 10*(pow((time/t_s),3));

		//double delta_x = delx*eq;   
		//double delta_y = dely*eq;
		//double delta_z = delz*eq;
	
		new_pose = initial_pose;
		new_pose[12] = new_pose[12] + delta_x;
		//new_pose[13] = new_pose[13] + delta_y;
		//new_pose[14] = new_pose[14] + delta_z;

		//std::cout << "First one " << std::endl;
		//std::cout << new_pose[12] << std::endl;
		if (time >= 10.0)
		{
			std::cout <<"END MOTION with this position" <<std::endl;
 			std::cout << initial_pose[12] << std::endl;               
			return franka::MotionFinished(new_pose);
		}
		return new_pose;       
	};

	robot.control(motion_callback);
}
//






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
        {{20.0, 20.0, 20.0, 0.0, 0.0, 0.0}}, {{20.0, 20.0, 20.0, 1.0, 1.0, 1.0}},
        {{0.0, 0.0, 0.0, 0.0, 0.0, 10.0}}, {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}});

    franka::RobotState initial_state = robot.readOnce();                 //Get the initial state
    franka::Model model(robot.loadModel());
    
    //double time = 0.0;
    std::array<double,16> new_pose;
    new_pose = initial_state.O_T_EE_c;
    initial_pose = initial_state.O_T_EE_c;

    constexpr int t_s = 10;	    	    
    constexpr double delx = -0.10; //In cm 10 cm = 0.1 m 


    for (size_t i = 0; i < 1; i++) {
      std::cout << "Executing motion." << std::endl;
      try {

        double time = 0.0;
        robot.control([=, &time, &new_pose](const franka::RobotState&,
                                 franka::Duration period) -> franka::CartesianPose {
          time += period.toSec();
          
        double  eq = 6*(pow((time/t_s),5)) - 15*(pow((time/t_s),4)) + 10*(pow((time/t_s),3));
	double delta_x = delx*eq;   
	
	new_pose = initial_pose;

	new_pose[12] = new_pose[12] + delta_x;

          franka::CartesianPose final_pose = new_pose;

          if (time >= 10) {
            std::cout << std::endl << "Finished motion." << std::endl;
            return franka::MotionFinished(final_pose);
          }
          return final_pose;
        });
      } catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        std::cout << "This is where the obstacle is" << std::endl;
        std::cout << new_pose[12] << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot.automaticErrorRecovery();
	//move_back(-delx);
      }
    }

 } //try of the main function

  catch (const franka::Exception& e) 
  {
    std::cout << e.what() << std::endl;
    return -1;
  }
  
  std::cout << "MAIN IS RUNNING" << std::endl;
  return 0;
} //main function





