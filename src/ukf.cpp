#include "ukf.h"

ukf::ukf(){

  std::cout<<"ukf initialize"<<std::endl;


  lambda= alpha * alpha * (x_size + kappa) -x_size;
  x_sigmavector_size = x_size*2+1;
  x.setZero();





}


//time update
void ukf::predict(){

   std::cout<<"correct"<<std::endl;




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
