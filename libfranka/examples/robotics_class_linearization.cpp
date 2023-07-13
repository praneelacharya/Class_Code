//This program takes input and gives an output. The input would be desired joint angles you want robot to attain. The output would be robot motion. One mistake we can’t do is, we can’t give any random joint angles. The reason is each joints has their own limit. So, as long as we pass joint angles that are within the limit, the robot would follow the command.


#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include <thread>
#include <franka/gripper.h>


int joint_limit_test(double q1,double q2,double q3,double q4,double q5,double q6,double q7)
{
	int joint_limit;

	if (q1>= 2.5 || q1<= -2.5)
	{
		joint_limit = 0;
		std::cout << "Sorry Joint Angle 1 is out of Range " << std::endl;
	}
	else if (q2>= 1.6 || q2<= -1.6)
	{
		joint_limit = 0;
		std::cout << "Sorry Joint Angle 2 is out of Range " << std::endl;
	}
	else if (q3>= 2.5 || q3<= -2.5)
	{
		joint_limit = 0;
		std::cout << "Sorry Joint Angle 3 is out of Range " << std::endl;
	}
	else if (q4>= -0.06 || q4<= -3.0)
	{
		joint_limit = 0;
		std::cout << "Sorry Joint Angle 4 is out of Range " << std::endl;
	}
	else if (q5>= 2.8 || q5<= -2.8)
	{
		joint_limit = 0;
		std::cout << "Sorry Joint Angle 5 is out of Range " << std::endl;
	}
	else if (q6>= 3.75 || q6<= -0.01)
	{
		joint_limit = 0;
		std::cout << "Sorry Joint Angle 6 is out of Range " << std::endl;
	}
	else if (q7>= 2.7 || q7<= -2.7)
	{
		joint_limit = 0;
		std::cout << "Sorry Joint Angle 7 is out of Range " << std::endl;
	}
	else
	{
		joint_limit = 1;
	}

	return joint_limit;
}


int main() {
  try {
    franka::Robot robot("172.16.0.2");
    setDefaultBehavior(robot);

     robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}});  //This was modified by praneel 200 :

	int joint_limit;
	double q1;  double q2;  double q3;  double q4;  double q5;  double q6;  double q7;

	//--------- EDIT SECTION STARTS HERE, give different joint values ------------//
	//q1 = 0.0225;         // 1st Joint Angle
	//q2 = -0.3208;        // 2nd Joint Angle
	//q3 = -0.0442;        // 3rd Joint Angle
	//q4 = -2.1638;        // 4th Joint Angle
	//q5 = -0.0019;        // 5th Joint Angle
	//q6 = 1.9781;         // 6th Joint Angle
	//q7 = 0.8664;         // 7th Joint Angle
	//--------- END OF EDIT SECTION-----------------------------------------------//

    	//std::array<double, 7> init_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    	//MotionGenerator motion_generator(0.01, init_goal);
			//robot.control(motion_generator);


    	std::cout << "Finished moving to initial joint configuration." << std::endl;


	std::cout << "Enter your desired 1st Joint Angle  " << std::endl;
	std::cin >> q1;

	std::cout << "Enter your desired 2nd Joint Angle  " << std::endl;
	std::cin >> q2;

	std::cout << "Enter your desired 3rd Joint Angle  " << std::endl;
	std::cin >> q3;

	std::cout << "Enter your desired 4th Joint Angle  " << std::endl;
	std::cin >> q4;

	std::cout << "Enter your desired 5th Joint Angle  " << std::endl;
	std::cin >> q5;

	std::cout << "Enter your desired 6th Joint Angle  " << std::endl;
	std::cin >> q6;

	std::cout << "Enter your desired 7th Joint Angle  " << std::endl;
	std::cin >> q7;


    	std::array<double, 7> q_goal = {{q1,q2,q3,q4,q5,q6,q7}};

    	joint_limit = joint_limit_test(q1,q2,q3,q4,q5,q6,q7);


    if (joint_limit == 1)
	{
    		std::cout << "!!!!!   Joint Limit you gave are within accepted range    !!!" << std::endl;

		// Once joint limit is within range we execute the motion //
		MotionGenerator motion_generator(0.04, q_goal);
    		std::cout << "WARNING: This example will move the robot! "
                          << "YOU DON'T WANT TO BE HIT BY ROBOT DO YOU? " << std::endl;
              		  std::cout << "Press Enter to continue..." << std::endl;

    		std::cin.ignore();
    		robot.control(motion_generator);
    		std::cout << "Finished moving to initial joint configuration." << std::endl;
	}
    else
	{
    		std::cout << "Sorry!! Joint Limit you gave are out of range / try new ones" << std::endl;
	}




  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
