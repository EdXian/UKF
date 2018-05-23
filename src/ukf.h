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
  int state_size;
  int x_sigmavector_size;



  void predict();
  void update();



private:
 Eigen::VectorXd weight_c;
 Eigen::VectorXd weight_m;
 Eigen::VectorXd x;
 Eigen::VectorXd x_sigmavector;

 double alpha = 1e-3;
 double kappa = 0;
 double beta = 2;
 double lambda = 0.0;

 double y_mean = 0.0;
 double x_mean = 0.0;


};

#endif // UKF_H


