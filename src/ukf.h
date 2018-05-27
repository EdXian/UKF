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

  double L;
  //vector size
  int x_size;
  int y_size;
  int x_sigmavector_size;
  int y_sigmavector_size;

  Eigen::VectorXd dynamics(Eigen::VectorXd state);
  void predict();
  void update();



private:

 Eigen::VectorXd w_c ; //weight c
 Eigen::VectorXd w_m ;  //weight m
 Eigen::VectorXd x ; //states
 Eigen::VectorXd y ; //measurements

 Eigen::VectorXd x_mean; //x mean
 Eigen::VectorXd y_mean; //y mean


 Eigen::MatrixXd x_sigmavector ;
 Eigen::MatrixXd y_sigmavector ;
 Eigen::MatrixXd H ;    //measurement transform

 Eigen::MatrixXd P ; //covariance matrix
 Eigen::MatrixXd P_yy ;
 Eigen::MatrixXd P_xy ;

 Eigen::MatrixXd Kalman_gain ;

 double alpha = 1e-3;
 double kappa = 0;
 double beta = 2;
 double lambda = 0.0;

// double y_mean = 0.0;
// double x_mean = 0.0;


};

#endif // UKF_H


