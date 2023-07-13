

#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "examples_common.h"

class linear_mapping{
	public:
		linear_mapping(double total_time){	//delta_x and total_time
			scaling = (total_time/360);
			//output_scaling();
		}
		
		double output_scaling(){
			return scaling;
		}
	
	private:
		double scaling;

};


int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
	try{

	std::cout << "Hello" << std::endl;
	franka::Robot robot(argv[1]);
    	setDefaultBehavior(robot);

	robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    	// First move the robot to a suitable joint configuration
    	std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
	
    	MotionGenerator motion_generator(0.05, q_goal);
	std::cout << "I am done" << std::endl;
    	std::cin.ignore();
    	robot.control(motion_generator);
    	std::cout << "Finished moving to initial joint configuration." << std::endl;


	linear_mapping deltaT(10); 			//Final Time
	double factor = deltaT.output_scaling();
	std::cout << factor << std::endl;	

	//Decelerations
	Eigen::Vector3d initial_position;
	double time = 0.0;
	double deltaX = 0.10;				//New pose will be x+deltaX
	std::array<double,16> initial_pose;
	std::array<double,16> new_pose;	

	auto getPosition = [](const franka::RobotState& robot_state){
		return Eigen::Vector3d(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]);
	}; 
	
	auto backNforth = [&](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::CartesianPose{
      		time += period.toSec();
		
		if (time == 0.0){
			initial_position = getPosition(robot_state);
			initial_pose = robot_state.O_T_EE_c;
			new_pose = initial_pose;
		}

		double angle = (time*(M_PI)*factor/180);		//Angle in Radian
		std::cout << angle << std::endl;			//Don't have this under loop
		
		new_pose[12] = 	initial_position[0] + deltaX*std::sin(angle);	

		if (time > 10){
        		std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        		return franka::MotionFinished(new_pose);
		}
		
		return new_pose;
				
	};

	robot.control(backNforth);

	} //T

	catch (const franka::Exception& e) {
    		std::cout << e.what() << std::endl;
    		return -1;
  	}
	return 0;
	
}
