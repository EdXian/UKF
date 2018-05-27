#include "ukf.h"

ukf::ukf(){

  std::cout<<"ukf initialize"<<std::endl;
  x_size = 2;
  y_size =1;
  L=x_size;
  x_sigmavector_size=2*x_size+1;

  lambda= alpha * alpha * (L + kappa) -L;

  x.setZero(x_size);

  x_sigmavector.setZero(x_size,x_sigmavector_size);



  y_sigmavector.setZero(x_size,1);


  H.setZero(1,2);  // measurement matrix

  H<<1,0;
  y = H*x;

  w_c.setZero(x_sigmavector_size);
  w_m.setZero(x_sigmavector_size);


  w_c(0) = (lambda / (L+lambda))+(1-alpha*alpha+beta);
  w_m(0) = (lambda)/(L+lambda);

  for(int i=1 ; i<x_sigmavector_size ; i++){
    w_c(i) = 1/(2*(L+lambda));
    w_m(i) = 1/(2*(L+lambda));
  }


  P=0.01*Eigen::MatrixXd::Identity(x_size, x_size);

  P_yy.setZero(x_sigmavector_size,1);

}






//time update
void ukf::predict(){



  //find sigma point
  P=(lambda+L)*P;
  Eigen::MatrixXd M= (P).llt().matrixL();


  x_sigmavector.col(0) = x;
  for(int i=0;i<x_size;i++)
  {
    Eigen::VectorXd data =  M.row(i).transpose();

    x_sigmavector.col(i+1) = x + data;

    x_sigmavector.col(i+x_size+1) = x - data;
  }


  //predict state based on system dynamics




  std::cout<<"x_sigma"<<std::endl<< x_sigmavector <<std::endl;
  x_mean.setZero(x_size);   //initialize x_mean
  for(int i=0;i<x_sigmavector_size;i++){
    x_mean += w_m(i)* x_sigmavector.col(i);
  }


  for(int i=0 ; i<x_sigmavector_size ;i++){
    P =   w_c(i) * x_sigmavector * x_sigmavector.transpose();
  }

  for(int i=0;i<x_sigmavector_size ; i++){
    y_sigmavector = H*x_sigmavector;
  }

  y_mean.setZero(y_size);
  for(int i=0;i< x_sigmavector_size;i++){
    y_mean += w_m(i) * y_sigmavector.col(i);
  }
}

//measurement update
void ukf::update(){


    for(int i=0;i<x_sigmavector_size;i++){

      Eigen::VectorXd err(2);
      Eigen::VectorXd err_t;
      err = y_sigmavector.col(i) - y_mean;
      err_t = err.transpose();

      P_yy += w_c(i) * err * err_t;
    }

    for(int i=0;i<x_sigmavector_size;i++){

      Eigen::VectorXd erry(2) , errx(2);

      erry = y_sigmavector.col(i) - y_mean;
      errx = x_sigmavector.col(i) - x_mean;
      P_xy += w_c(i) * errx * erry.transpose();
    }



    Kalman_gain = P_xy * P_yy.inverse();




    Eigen::VectorXd merr = y-y_mean;

    x = x_mean + Kalman_gain *(merr);

    P = P - Kalman_gain*P_yy*Kalman_gain.transpose();
    std::cout << "x" <<std::endl<< x <<std::endl;
    std::cout << "P" <<std::endl<< P <<std::endl;

    std::cout<<"update ok"<<std::endl;
}

Eigen::MatrixXd ukf::dynamics(Eigen::MatrixXd sigma_state){

  Eigen::MatrixXd predict_sigma_state(x_size,x_sigmavector_size);
  for(i=0;i<x_sigmavector_size;i++){
    predict_sigma_state(0,i) =   sigma_state(0,i)+ sigma_state(1,i)*dt;
    predict_sigma_state(1,i) =   cos(sigma_state(0,i));
  }
  return predict_sigma_state;
}


ukf::~ukf(){

}
