#ifndef UKF_H
#define UKF_H
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>

class ukf
{
public:
  ukf();
  ~ukf();

  int L;
  int x_size;
  int x_sigmavector_size;



  void predict();
  void update();



private:
 Eigen::VectorXd w_c ; //weight c
 Eigen::VectorXd w_m ;  //weight m
 Eigen::VectorXd x ; //states
 Eigen::VectorXd y ; //measurements
 Eigen::VectorXd x_sigmavector ;
 Eigen::VectorXd y_sigmavector ;

 Eigen::MatrixXd H ;

 Eigen::MatrixXd P ; //covariance matrix
 Eigen::MatrixXd P_xx ;
 Eigen::MatrixXd P_xy ;

 Eigen::MatrixXd Kalman_gain ;


 double alpha = 1e-3;
 double kappa = 0;
 double beta = 2;
 double lambda = 0.0;

 double y_mean = 0.0;
 double x_mean = 0.0;


};

#endif // UKF_H


