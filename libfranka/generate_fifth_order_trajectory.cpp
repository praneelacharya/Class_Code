

#include <iostream>
#include <eigen3/Eigen/Dense>
// #include <Eigen/Dense>
#include <fstream>

// #include "trajectory_functions.h"

// Here we are computing the coefficent of the fifth order polynomial
Eigen::MatrixXf fifth_order_compute_coefficent(double startT, double finalT, float p0, float pT){

    Eigen::Matrix<float, 6, 6> matrixA ;
    Eigen::Matrix<float, 6, 1> vectorb ;

    matrixA <<  pow(startT,5),    pow(startT,4),      pow(startT,3),      pow(startT,2),      startT,      1,
                pow(finalT,5),    pow(finalT,4),      pow(finalT,3),      pow(finalT,2),      finalT,      1,
                5*pow(startT,4),      4*pow(startT,3),        3*pow(startT,2),        2*startT,       1,   0,
                5*pow(finalT,4),      4*pow(finalT,3),        3*pow(finalT,2),        2*finalT,       1,   0,
                20*pow(startT,3),     12*pow(startT,2),       6*startT,       2,      0,      0,
                20*pow(finalT,3),     12*pow(finalT,2),       6*finalT,       2,      0,      0;

    // vectorb <<  p0,
    //             pT,
    //             0,  //v0 = 0
    //             0,  //vT = 0
    //             0,  //a0 = 0
    //             0;  //aT = 0

    vectorb <<  0,
                pT - p0,
                0,  //v0 = 0
                0,  //vT = 0
                0,  //a0 = 0
                0;  //aT = 0

    // std::cout << "This is the result";
    // std::cout << matrixA.inverse() * vectorb << std::endl;
    return matrixA.inverse() * vectorb;
}

// Here given the time and the polynomial coefficent we compute the current position
float current_position(double* time, double* startTime, double* endTime, float* start_pos, Eigen::MatrixXf* coeff)
{
    if ( ( *time < *startTime ) )
    {
        return *start_pos + 0;

    }
    else if (  *time > *endTime ) 
    {
        return *start_pos + pow(*endTime,5)*(*coeff)(0) +  pow(*endTime,4)*(*coeff)(1) + pow(*endTime,3)*(*coeff)(2) + pow(*endTime,2)*(*coeff)(3) + (*endTime)*(*coeff)(4) + (*coeff)(5);
    }
    else
    {
        return *start_pos + pow(*time,5)*(*coeff)(0) +  pow(*time,4)*(*coeff)(1) + pow(*time,3)*(*coeff)(2) + pow(*time,2)*(*coeff)(3) + (*time)*(*coeff)(4) + (*coeff)(5);
    }
    
}


int main()
{

    // Simulate the genereated 1D trajectory:  - INPUT FROM THE USER
    double startTime = 2.0;
    double endTime = 5.0;

    const float dt = 0.01;

    int Npoints = (endTime + 3.0 - startTime)/dt + 1;

    float x_t0 = 0.2;
    float x_tN = 0.4;

    float y_t0 = 0.4;
    float y_tN = 0.4;

    float z_t0 = 0.4;
    float z_tN = 0.4;

    // Extract the coefficents of the polynomial
    Eigen::MatrixXf Xpolynomial_coefficents = fifth_order_compute_coefficent(startTime, endTime, x_t0, x_tN);
    Eigen::MatrixXf Ypolynomial_coefficents = fifth_order_compute_coefficent(startTime, endTime, y_t0, y_tN);
    Eigen::MatrixXf Zpolynomial_coefficents = fifth_order_compute_coefficent(startTime, endTime, z_t0, z_tN);

    //  This is for simulating all the time points: just for simulation
    Eigen::VectorXd time_array = Eigen::VectorXd::LinSpaced( Npoints, startTime - 2, endTime + 3.0);
    
    // Let's open the file for the write up
    // Creating a text file
    std::fstream myFile;
    myFile.open("generated_trajectory.txt", std::ios::out);
    if ( myFile.is_open() )
    {
        for (int indx = 0; indx < time_array.size(); indx++)
        {
            myFile << time_array(indx) << " " << current_position(&time_array(indx), &startTime, &endTime, &x_t0, &Xpolynomial_coefficents) \
                                       << " " << current_position(&time_array(indx), &startTime, &endTime, &y_t0, &Ypolynomial_coefficents) \
                                       << " " << current_position(&time_array(indx), &startTime, &endTime, &z_t0, &Zpolynomial_coefficents) << std::endl;
        }
    }

    myFile.close();


    return 0;
}