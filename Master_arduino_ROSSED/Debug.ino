/***************************************DEBUGS**************************************/
/*
If any data is not received at all - 

CHECK CIRCUIT CONNECTIONS, FUSES AND BATTERY VOLTAGES (Hardware)

CHECK VARIABLES AND ALL THE OPERATIONS ON THEM (Software)

PRINT AND CHECK AT ALL THE AVAILABLE LOOPS (Software)


Initial Set up - Manual Switch ON and then RF module ON


A) COMPASS

>> Use Adafruit_HMC5883 magneto-sensor
>> Four pin connections and power (3.3V)
>> SCL1/SDA1 (Connected pins, circuit design and check of Wire/Wire1 in Adafruit library)
>> Connections and theta sign --> If soldered well, theta will be positive; else "theta = - theta" has to be used
>> Units in which the data is being sent - Radians or Degrees,
>> Check the coordinates for 279+ degrees and make sure they are correct with nav stack
>> Test with the "test.ino" code

B) MOTORS AND MOTOR CONTROLLERS

>> Use ESCON motors and motorcontrollers
>> Check connections of motor(terminals) and power (24V)
>> Check connections of motorcontroller and encoder (Appropriate LEDs should glow)
>> Ensure motor enable pins are HIGH (Software)
>> Recheck the "motor" class above --> Direction (dir) pin, enable pin and PWM pin
>> Use Configure/PID Tuning/Diagnosis tools available in ESCON software

C) ARDUINO 

>> Use Arduino Due for both master and slave controllers
>> Set the tools properly --> Port, Board and Programmer (Eg: Arduino Programming Port, Arduino DUE Programming and AVRISP mkII)
>> Upload the code again and check the values
>> Error: Unable to find device <|> Check if rosserial is running 
>> Check the data transmission between Master Arduino and Slave Arduino (SCL/SDA)
>> Slave arduino gives initial ticks in setup() to get reset effect as and when the Master resets itself. 

D) ROS AND ROSSERIAL

>> Use ROS Hydro version for the run
>> Error: Unable to sync with device <|> Comment all Serial.prints in the arduino code to be uploaded, before running rosserial (use printr function and comment out #define ROS_ON)
>> Match the port and the baud rate used
>> Error: Unable to sync with device. Possible version mismatch... <|> Define USE_USBCON in arduino code for rosserial version compatibility with "Arduino DUE"
>> Error: Lost sync with device. Reconnecting... <|> Use enough nh.spinonce() for all the required callbacks to be called frequently
   --> Wait a little longer for the connection to establish. It takes a while for Arduino DUE
>> Check ROS publishers and subscribers; rostopic names and frames --> Header frame = 'odom'; Child frame = 'base_link'
>> Check the robot tranformation configuration --> robot_tf_config

E) ODOMETRY 

>> Recheck all of the above once and proper coupling of motors/wheels
>> Check the units in which the data is being tranferred (To and fro)
>> Match the ROS data and Arduino data (For any accuracy loss in transmission) --> Print one in terminal and rostopic echo the other
>> Check the ticks per rotation set (TPRR / TPRL)
>> Check distance (Dist), pose and velocity calculations make sure they approx match v*dt
>> Manually rotate the wheel by exact one rotation and check the ticks obtained (Whether they approx. match with TPRs set)
>> 


*/
