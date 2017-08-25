# CarND-PID-Control-Project
## Introduction 
  The project's purpose is to build a PID controller with tuned hyperparameters and run it against a simulator. The technics learned in the courses provide a baseline to implement the algorithms of PID controller in C++.
  
## PID Components' Effects Description
To demonstrate the effect of each type of controller, i tested four configurations of the hyperparameter array hyperparams = {kp,kd,ki} at the initialization step of each controller workflow applyed to steering value and throttle one.

### P Controller : 
To enable only the P component, the hyperparams = {0.15,0,0} configuration is used. The effect is immidiately observable. The car starts by steering to the right until it is far from the central lane and then it steers to the left until it is far from the central line. This cycle lasts as the way is straight but when the corner begin, the CTE starts to grow until it reaches a certain value which make  the car overshoots the lane completly.

### PD Controller : 
To enable only the P and D components, the hyperparams = {0.15,0.0003,0} configuration is used. The same behavior is noticed but in fast manner. The car overshoot the lane and crashes. A fine tuned parameter makes the car steer stablely along the central line. 

### PDI Controller : 
To enable it, the hyperparams = {0.15,0.0003,3} configuration is used. The result is perfect and the car behaves as expected. It steers smoothly along the central line.

## Hyperparameters Tuning :
All parameters were tuned manually.  This approach quickly lead to unrecoverable crashes of the car and require a manual restart of the simulator. The process used was as follows:
    1. Enable P controller, with hyperparams[1] and hyperparams[2] to zero
    2. Enable PD Controller by adding and increasing hyperparams[1] until oscillations. When the crashes 
    3. Enable PID Controller by adding and increasing hyperparams[1] until the car steers fine.

