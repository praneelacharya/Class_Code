#include<iostream>
#include<Eigen/Core>
#include<Eigen/SVD>
#include<cmath>

#include "pseudo_inversion.h"

int main()
{
    Eigen::MatrixXd A(2,3);
    A<< 1, 2, 3, 4, 5, 7;
    std::cout<<A<<std::endl<<std::endl;

    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(A, jacobian_transpose_pinv);

    std::cout<< jacobian_transpose_pinv <<std::endl;

    return 0;
}
