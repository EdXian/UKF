# UKF Simulation

Qtversion 5.9.2

position estimate 

![Alt Text](simulate/position.png)

velocity estimate

![Alt Text](simulate/velocity.png)

dynamics 

x_dot_k+1 = x_dot_k _ v*dt
v = cos(x)+0.99*x_dot

states

x 
x_dot

measurements

x position

H = [1 0]


# Use it

1 define state and measurement 
```
enum state{
    pos=0,
    velocity,
    statesize
};

enum measurement{
    mpos = 0,
    measurementsize
};

```
2 define the nonlinear function in xxx.h

```
Eigen::MatrixXd forceest::dynamics(Eigen::MatrixXd sigma_state){

    Eigen::MatrixXd predict_sigma_state(this->x_size,this->x_sigmavector_size);

    for(int i=0;i<this->x_sigmavector_size;i++){

        double p = sigma_state(pos,i) ;
        double v = sigma_state(velocity,i);
        double p_ ;
        double v_ ;

        p_ = p+v*dt;
        v_ = cos(0.6*p)* cos(0.6*p) - sin(0.6*p) * sin(0.6*p) +0.99*v;


        predict_sigma_state(pos,i) =  p_ ;
        predict_sigma_state(velocity,i) =  v_;

    }
    return predict_sigma_state;
}

```




3 create object
```
forceest forceest1(statesize,measurementsize);
```




4 parameter set

forceest1.set_measurement_noise(mnoise);
forceest1.set_process_noise(noise);
forceest1.set_measurement_matrix(measurement_matrix);
forceest1.dt = 0.02;


5 predict the next state

```

forceest1.predict();

```

6 correct from measurement
```
Eigen::VectorXd measure_vector;
measure_vector.setZero(1);
measure_vector<<measure ;
forceest1.correct(measure_vector);
```






