#include <iostream>
#include "ukf.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>



Eigen::MatrixXd A(3,3);
//using namespace  std;


int main(){


  ukf new_ukf();
//  A << 4,-1,2, -1,6,0, 2,0,5;
//  std::cout << "The matrix A is" <<std::endl << A << std::endl;
//  Eigen::LLT<Eigen::MatrixXd> lltOfA(A); // compute the Cholesky decomposition of A
//  Eigen::MatrixXd L = lltOfA.matrixL();

//  cout << "The Cholesky factor L is" << endl << L << endl;
//  cout << "To check this, let us compute L * L.transpose()" << endl;
//  cout << L * L.transpose() << endl;
//  cout << "This should equal the matrix A" << endl



A << Eigen::MatrixXd::Identity(3,3);

 std::cout<< A<< std::endl;
 A.setZero();
 std::cout<< A<< std::endl;

 std::cout <<"ok"<<std::endl;

}
