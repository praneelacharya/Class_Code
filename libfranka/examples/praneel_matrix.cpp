#include <Eigen/Dense>

#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "examples_common.h"
using namespace Eigen;

std::array<double,16> initial_pose;


Eigen::Vector2f hello(int x_start,int x_end, int t_start,int t_end)
{
	std::cout <<"Really" << std::endl;

	//Create a 4*4 Matrix
	
	Eigen::Matrix4f A;

	A << 0,t_start,std::pow(t_start,2),std::pow(t_start,3),
	            0,1,2*t_start,3*std::pow(t_start,2),
	            1,t_end,std::pow(t_end,2),std::pow(t_end,3),
	            0,1,2*t_end,3*std::pow(t_end,2);

	//A << 1,1,1,1, 1,1,6,2, 7,8,9,2, 1,1,1,1;

	//End of Matrix Creation

	
	// Define Boundary Conditions
	Eigen::Vector4f boundary; 
	boundary<< 3,3,1,3;    //x0 velocity at x0 x1 velocity at x1 

	std::cout << "Boundary " << boundary << std::endl;
	std::cout << "end" << std::endl;
	std::cout << A << std::endl;
	//End of bounary condition


	//Eigen::Matrix4f inverse = A.inverse();
	//Eigen::Vector4f coefficients = inverse;
	//std::cout << "coed " << coefficients << std::endl;
	//Eigen::Matrix4f d = square_a.inverse();
	std::cout << "col" << A.ldlt().solve(boundary) << std::endl;
	//End of coefficient calulations

	Eigen::Vector2f b;
	b << 0,1;
	return b;
}


int main()
{

	float x0 = 0;
	float x1 = 1;
	int   t0 = 0;
	int   t1 = 1;

	std::cout << "Hello World" << std::endl;

	Eigen::Matrix4f a;

	a << 0,t0,std::pow(t0,2),std::pow(t0,3),
	     0,1,2*t0,3*std::pow(t0,2),
	     1,t1,std::pow(t1,2),std::pow(t1,3),
	     0,1,2*t1,3*std::pow(t1,2);

	//ColPivHouseholderQR<Matrix4f> dec(a);
	//Vector3f x = dec.solve(b); 

	Eigen::Matrix4f M1 = Eigen::Matrix4f::Random();
	std::cout << "Hepe" << M1.inverse() << std::endl;

        hello(x0,x1,t0,t1);


return 0;
}
