# PID Controller for DC motors
A project to study the characteristics of a DC motor, determine the PID controller settings based on the model, and control the motor using a previously designed controller.

## Used hardware
- uC: STM32F303k8
- Motor: Pololu 2204
- Encoder: Pololu 4761
- Driver: TB6612FNG

## PID 
- sampling time: 10ms
## Step response of the motor
To measure the step response, set the STEP_RESPONSE constant to 1 in the Core/Src/main.c file and upload program to stm32. On the pc, run the port.cpp program, which will receive and write the data to a file.
## PID settings
The previously saved step response file should be completed with a column containing the step PWM value, it should look like this:
|      |         |
|------|---------|
| 0 | 0 |
| 1020 | 33.3333|
| 1020 | 133.333|
| 1020 | 200|
| 1020 | 233.333|
| 1020 | 250|
| 1020 | 266.667|
| 1020 | 283.333|
| 1020 | 283.333|
| 1020 | 300|
....

then run the engine.m script in Matlab, which will determine the PID settings based on the data in the file.

## PID controller

In the Core/Src/main.c file, change the value of the STEP_RESPONSE constant to 0, enter the controller settings calculated by matlab. Now your motor is controlled by the PID! 