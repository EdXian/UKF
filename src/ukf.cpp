#include "ukf.h"

ukf::ukf(){

  std::cout<<"ukf initialize"<<std::endl;
  x_size = 2;
  y_size =x_size;
  L=x_size;
  x_sigmavector_size=2*x_size+1;

  lambda= alpha * alpha * (L + kappa) -L;

  x.setZero(x_size);
  x.setOnes(x_size);


  x_sigmavector.setZero(x_size,x_sigmavector_size);



  y.setZero(y_size);

  y_sigmavector.setZero(x_size,x_sigmavector_size);  //to be determined


  std::cout<< "L  " <<L <<std::endl;
  std::cout << "lambda  " <<lambda<<std::endl;
  w_c.setZero(x_sigmavector_size);
  w_m.setZero(x_sigmavector_size);


  w_c(0) = (lambda / (L+lambda))+(1-alpha*alpha+beta);
  w_m(0) = (lambda)/(L+lambda);

  for(int i=1 ; i<x_sigmavector_size ; i++){
    w_c(i) = 1/(2*(L+lambda));
    w_m(i) = 1/(2*(L+lambda));
  }

  std::cout<< w_m(0) <<std::endl;

//  double sum=0.0;
//  for(int i = 0; i<x_sigmavector_size;i++){
//  sum+= w_m(i);
//  }
//  std::cout<<"sum"<<sum <<std::endl;
  std::cout<<"*******"<<std::endl;
  std::cout<< w_m<<std::endl;
  std::cout<<"*******"<<std::endl;


  //P.setZero(x_size,x_size);
  P=0.01*Eigen::MatrixXd::Identity(x_size, y_size);

//  std::cout<<"lambda ="<<lambda<<std::endl;
//  std::cout<<P<<std::endl;


//  std::cout <<L<<std::endl;
//  std::cout<<L+lambda<<std::endl;
//  std::cout<<(L+lambda)*P<<std::endl;


  P=(lambda+L)*P;
  Eigen::MatrixXd M= (P).llt().matrixL();
//  std::cout<<M.row(0).transpose()<<std::endl;

//  std::cout<< M <<std::endl;


  P_yy.setZero(x_size,x_size);

  P_xy.setZero(x_size,x_size);

  Kalman_gain.setZero(x_size,x_size);

}

//time update
void ukf::predict(){

  std::cout<<"correct"<<std::endl;

  P=(lambda+L)*P;
  Eigen::MatrixXd M= (P).llt().matrixL();
 // std::cout<<M.row(0).transpose()<<std::endl;
//  Eigen::VectorXd sigma_p = M.row(0).transpose();
//  Eigen::VectorXd sigma_n = M.row(1).transpose();

//  std::cout<< M <<std::endl;
//  std::cout<< x <<std::endl;

  std::cout<<"--------"<<std::endl;

  x_sigmavector(0,0) = x(0);
  x_sigmavector(1,0) = x(1);

  //find sigma point
  for(int i=0;i<x_size;i++)
  {
    Eigen::VectorXd data = x + M.row(i).transpose();
    //std::cout<<data<<std::endl;
    x_sigmavector(0,i+1)=data(0);
    x_sigmavector(1,i+1)=data(1);
  }
  for(int i=x_size;i<2*x_size;i++)
  {
    Eigen::VectorXd data = x - M.row(i-x_size).transpose();
    //std::cout<<data<<std::endl;
    x_sigmavector(0,i+1)=data(0);
    x_sigmavector(1,i+1)=data(1);
  }

  for(int i=0;i<x_size*2;i++){

  }


  std::cout << x_sigmavector <<std::endl;

}

//measurement update
void ukf::update(){

      std::cout<<"predict"<<std::endl;
  //    //predict the state based on dynamics




  //    //weightecov = ((n+lambda)* estimate_errcovar ) sqrt (L*L_T)
  //    // First sigma point is the current state X0=[x x+sigmap(1) x-sigmap(1)   ]

  //    //



}
ukf::~ukf(){



}
