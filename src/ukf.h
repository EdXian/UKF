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
  double dt;
  //vector size
  int x_size;
  int y_size;
  int x_sigmavector_size;
  int y_sigmavector_size;
  Eigen::MatrixXd  dynamics(Eigen::MatrixXd  sigma_state);
  Eigen::MatrixXd rotate(double roll , double yaw , double pitch);

  Eigen::MatrixXd  force_dynamics(Eigen::MatrixXd  sigma_state);
  void predict();
  void correct(double measure);

  Eigen::VectorXd x ; //states
  Eigen::VectorXd x_a;
   Eigen::VectorXd x_a_hat;
  Eigen::VectorXd y ; //measurements

  Eigen::VectorXd x_hat; //x mean
  Eigen::VectorXd y_hat; //y mean

private:

 Eigen::VectorXd w_c ; //weight c
 Eigen::VectorXd w_m ;  //weight m


 bool isinit = true;

 Eigen::MatrixXd x_a_sigmavector;
 Eigen::MatrixXd x_sigmavector ;
 Eigen::MatrixXd y_sigmavector ;
 Eigen::MatrixXd H ;    //measurement transform

 Eigen::MatrixXd P ; //covariance matrix
 Eigen::MatrixXd P_a ; //covariance matrix
 Eigen::MatrixXd P_ ; //covariance matrix
 Eigen::MatrixXd P_yy ;
 Eigen::MatrixXd P_xy ;

 Eigen::MatrixXd Kalman_gain ;

 double alpha = 1e-3;
 double kappa = 0;
 double beta = 2;
 double lambda = 0.0;

// double y_hat = 0.0;
// double x_hat = 0.0;


};

#endif // UKF_H


