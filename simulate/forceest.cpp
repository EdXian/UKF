#include "forceest.h"

Eigen::MatrixXd forceest::dynamics(Eigen::MatrixXd sigma_state){

    Eigen::MatrixXd predict_sigma_state(this->x_size,this->x_sigmavector_size);
    std::cout<<"virtual"<<std::endl;
    for(int i=0;i<this->x_sigmavector_size;i++){

      //system dynamics
      predict_sigma_state(0,i) =   sigma_state(0,i)+ sigma_state(1,i)*dt;
      predict_sigma_state(1,i) =   cos(sigma_state(0,i))+0.99*sigma_state(1,i);

    }
    return predict_sigma_state;


}
