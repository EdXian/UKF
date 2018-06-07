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


