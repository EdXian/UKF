#include "ukf.h"

ukf::ukf(){
  dt = 0.01;
  std::cout<<"ukf initialize"<<std::endl;
  x_size = 2;
  y_size =1;
  L=x_size;
  x_sigmavector_size=2*x_size+1;

  lambda= alpha * alpha * (L + kappa) -L;

  x.setZero(x_size);
  x_hat.setZero(x_size);
  x_a.setZero(x_size+x_size+y_size);
  x_a_hat.setZero(x_size+x_size+y_size);
  x_sigmavector.setZero(x_size,x_sigmavector_size);

  x_a_sigmavector.setZero(x_size+x_size+1 , 2*(x_size+x_size+y_size) +1 );

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


  P=1e-3*Eigen::MatrixXd::Identity(x_size, x_size);
  P_.setZero(x_size,x_size);
  P_yy.setZero(y_size,y_size);
  P_xy.setZero(x_size,y_size);

  //initialize state
  //std::cout<<"initialize"<<std::endl;
  x<<0,0;
  //std::cout<<"initialize"<<std::endl;
  x_hat<<0,0;
  //x_a  <<0,0,0,0,0;
  //std::cout<<"x_a"<<std::endl<<x_a<<std::endl;
  //std::cout<<"x_hat"<<std::endl<<x_hat<<std::endl;
  P = (x-x_hat)*(x-x_hat).transpose();
  //std::cout<<"P"<<std::endl<<P<<std::endl;

  P_a= (x_a-x_a_hat)*(x_a-x_a_hat).transpose();
  //std::cout<<"P_a"<<std::endl<<P_a<<std::endl;
}






//time update
void ukf::predict(){


 if(isinit){
     P_a=(lambda+L)*P_a;
     Eigen::MatrixXd M= (P_a).llt().matrixL();

    x_a_sigmavector.col(0) = x_a;
    for(int i=0;i<x_size+x_size+y_size;i++)
    {

        Eigen::VectorXd sigma =  (M.row(i)).transpose();
        x_sigmavector.col(i+1) = x_a + sigma;
        x_sigmavector.col(i+x_size+x_size+y_size+1) = x_a - sigma;
    }

    isinit = false;
 }else{

      P_a=(lambda+L)*P_a;

      //todo



 }

//find sigma point
//  Eigen::MatrixXd M= (P_a).llt().matrixL();
//  x_sigmavector.col(0) = x;
//  for(int i=0;i<x_size;i++)
//  {

//    Eigen::VectorXd sigma =  (M.row(i)).transpose();
//    x_sigmavector.col(i+1) = x + sigma;

//    x_sigmavector.col(i+x_size+1) = x - sigma;
//  }


  //predict state based on system dynamics
//    std::cout<< "----------------"<<std::endl;
//    std::cout<<"x_sigmavector" <<std::endl<<x_sigmavector<<std::endl;
//    x_sigmavector = dynamics(x_sigmavector);
//    std::cout<<"x_sigmavector" <<std::endl<<x_sigmavector<<std::endl;
//    std::cout<< "++++++++++++++++"<<std::endl;

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

//  std::cout<< "----------------"<<std::endl;
//  std::cout<<"P_" <<std::endl<<P_<<std::endl;
//  std::cout<< "++++++++++++++++"<<std::endl;

    //measurement sigma vector
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


    for(int i=0;i<x_sigmavector_size;i++){

      Eigen::VectorXd err_y(2) , err_x(2);

      err_y = y_sigmavector.col(i) - y_hat;
      err_x = x_sigmavector.col(i) - x_hat;
      P_xy += w_c(i) * err_x * err_y.transpose();
    }

    Kalman_gain = P_xy * (P_yy.inverse());

    x = x_hat + Kalman_gain *(y-y_hat);

    P = P_ - Kalman_gain*P_yy*(Kalman_gain.transpose());
    std::cout << "-------------------------"<<std::endl;

    std::cout<<"P_"<<std::endl <<P_<<std::endl;
     std::cout<<"P"<<std::endl <<P<<std::endl;

    std::cout<<"++++++++++++++++=+++++++++"<<std::endl;
}

Eigen::MatrixXd ukf::dynamics(Eigen::MatrixXd sigma_state){

  Eigen::MatrixXd predict_sigma_state(x_size,x_sigmavector_size);
  for(int i=0;i<x_sigmavector_size;i++){
    predict_sigma_state(0,i) =   sigma_state(0,i)+ sigma_state(1,i)*dt;
    predict_sigma_state(1,i) =   cos(sigma_state(0,i))+0.99*predict_sigma_state(1,i);
  }
  return predict_sigma_state;
}
Eigen::MatrixXd ukf::force_dynamics(Eigen::MatrixXd sigma_state){


}

Eigen::MatrixXd ukf::rotate(double roll, double yaw, double pitch){

    Eigen::MatrixXd frame;


    return frame;
}

ukf::~ukf(){

}
