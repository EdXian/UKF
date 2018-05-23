#include "ukf.h"

ukf::ukf(){

  std::cout<<"ukf initialize"<<std::endl;
  lambda= alpha * alpha * (state_size + kappa) - state_size;
  x.setZero();





}
ukf::~ukf(){



}


void ukf::predict(){

   std::cout<<"correct"<<std::endl;




}


void ukf::update(){

      std::cout<<"predict"<<std::endl;
  //    //predict the state based on dynamics



  //    //weightecov = ((n+lambda)* estimate_errcovar ) sqrt (L*L_T)
  //    // First sigma point is the current state X0=[x x+sigmap(1) x-sigmap(1)   ]

  //    //



}
