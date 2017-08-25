# CarND-PID-Control-Project
# Introduction 
  The project's purpose is to build a PID controller with tuned hyperparameters and run it against a simulator. The technics learned in the courses provide a baseline to implement the algorithms of PID controller in C++.
  
# PID Components' Effects Description
To demonstrate the effect of each type of controller, i tested four configurations of the hyperparameter array hyperparams = {kp,kd,ki} at the initialization step of each controller workflow applyed to steering value and throttle one.

P Controller : To enable it, the hyperparams = {0.15,0,0} configuration is used. The effect is immidiately observable. The car starts by steering to the right until it is far from the central lane and then it steers to the left until it is far from the central lane. This cycle lasts as the way is straight but when at the starting of the corner the CTE starts to grow which make  the car overshoots the lane completly at a certain value of CTE

PD Contoller : To enable it, the hyperparams = {0.15,0.0003,0} configuration is used.
