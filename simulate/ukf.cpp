#include "ukf.h"

ukf::ukf(){
  //dt = 0.02;
  std::cout<<"ukf initialize"<<std::endl;


  x_size = 2;
  y_size =1;
  L=x_size;
  x_sigmavector_size=2*x_size+1;

  lambda= alpha * alpha * (L + kappa) -L;

  x.setZero(x_size);
  y.setZero(y_size);

  x_hat.setZero(x_size);
  y_hat.setZero(y_size);

  x_a.setZero(x_size+x_size+y_size);
  x_a_hat.setZero(x_size+x_size+y_size);

  x_sigmavector.setZero(x_size,x_sigmavector_size);

//  x_a_sigmavector.setZero(x_size+x_size+1 , 2*(x_size+x_size+y_size) +1 );

  y_sigmavector.setZero(x_sigmavector_size,y_size);


  H.setZero(y_size,x_size);  // measurement matrix

  H<<1,0;
  //initialize y
  y = H*x;

  w_c.setZero(x_sigmavector_size);
  w_m.setZero(x_sigmavector_size);


  w_c(0) = (lambda / (L+lambda))+(1-alpha*alpha+beta);
  w_m(0) = (lambda)/(L+lambda);

  for(int i=1 ; i<x_sigmavector_size ; i++){
    w_c(i) = 1/(2*(L+lambda));
    w_m(i) = 1/(2*(L+lambda));
  }

  Q =5e-7*Eigen::MatrixXd::Identity(x_size, x_size);
  R =5e-4*Eigen::MatrixXd::Identity(y_size,y_size);

  P=1e-4*Eigen::MatrixXd::Identity(x_size, x_size);
  P_.setZero(x_size,x_size);
  P_yy.setZero(y_size,y_size);
  P_xy.setZero(x_size,y_size);

  x<<0.1,0.1;
  x_hat<<.1,0.1;


}






//time update
void ukf::predict(){



//find sigma point
  P=(lambda+L)*P;
  Eigen::MatrixXd M= (P).llt().matrixL();
  Eigen::MatrixXd buffer;
  x_sigmavector.col(0) = x;

  for(int i=0;i<x_size;i++)
  {

    Eigen::VectorXd sigma =(M.row(i)).transpose();
    x_sigmavector.col(i+1) = x + sigma;

    x_sigmavector.col(i+x_size+1) = x - sigma;
  }
   buffer = dynamics( x_sigmavector);
    x_sigmavector =buffer;

   //x_hat (mean)
  x_hat.setZero(x_size);   //initialize x_hat
  for(int i=0;i<x_sigmavector_size;i++){
    x_hat += w_m(i)* x_sigmavector.col(i);
  }
    //covariance
   P_.setZero(x_size,x_size);
  for(int i=0 ; i<x_sigmavector_size ;i++){
    P_+=   w_c(i) * (x_sigmavector.col(i)-x_hat) * ((x_sigmavector.col(i)-x_hat).transpose());
  }
  //add process noise covariance
    P_+= Q;

  for(int i=0;i<x_sigmavector_size ; i++){
    y_sigmavector = H*x_sigmavector;
  }

  //x_hat (mean)
  y_hat.setZero(y_size);
  for(int i=0;i< x_sigmavector_size;i++){
    y_hat += w_m(i) * y_sigmavector.col(i);
  }
}


//measurement update
void ukf::correct(double measure){
    y<< measure;
    P_yy.setZero(y_size,y_size);
    P_xy.setZero(x_size,y_size);

    for(int i=0;i<x_sigmavector_size;i++){
      Eigen::VectorXd err;
      Eigen::VectorXd err_t;
      err = y_sigmavector.col(i) - y_hat;
      err_t = err.transpose();
      P_yy += w_c(i) * err * err_t;
    }
     //add measurement noise covarinace
     P_yy +=R;

    for(int i=0;i<x_sigmavector_size;i++){

      Eigen::VectorXd err_y , err_x;

      err_y = y_sigmavector.col(i) - y_hat;
      err_x = x_sigmavector.col(i) - x_hat;
      P_xy += w_c(i) * err_x * err_y.transpose();
    }

    Kalman_gain = P_xy * (P_yy.inverse());

    x = x_hat + Kalman_gain *(y-y_hat);

    P = P_ - Kalman_gain*P_yy*(Kalman_gain.transpose());

}


Eigen::MatrixXd ukf::dynamics(Eigen::MatrixXd sigma_state){

  Eigen::MatrixXd predict_sigma_state(x_size,x_sigmavector_size);
  for(int i=0;i<x_sigmavector_size;i++){

    //system dynamics
    predict_sigma_state(0,i) =   sigma_state(0,i)+ sigma_state(1,i)*dt;
    predict_sigma_state(1,i) =   cos(sigma_state(0,i))+0.99*sigma_state(1,i);

  }
  return predict_sigma_state;

}


Eigen::MatrixXd ukf::rotate(double roll, double yaw, double pitch){

    Eigen::MatrixXd frame;
    frame.setZero(3,3);
    double c_r = cos(roll) , s_r = sin(roll);
    double c_p = cos(pitch) , s_p = sin(pitch);
    double c_y = cos(yaw) , s_y = sin(yaw);
    frame << c_p*c_y ,c_y*s_p*s_r-s_y*c_r , c_y*s_p*c_r+s_y*s_r,
             c_p*s_y ,s_y*s_p+c_r*c_y     , s_y*s_p*c_r-c_y*s_r,
             -1*s_p  ,s_r*c_p             , c_r*c_p            ;
    return frame;
}

//






ukf::~ukf(){

}

