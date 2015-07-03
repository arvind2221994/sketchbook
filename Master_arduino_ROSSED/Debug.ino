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
>> pin configuration for due 

  -----------------------
 motor 1 (L) - 8, 23, 25, 27 --- 8 - pwm, 23 - enable, 25 - dir, 27 - brake
 motor 2 (R) - 9, 22, 26, 27 --- 9 - pwm, 22 - enable, 24 - dir, 26 - brake
  -------------------------


C) ARDUINO 

>> Use Arduino Due for both master and slave controllers
>> Set the tools properly --> Port, Board and Programmer (Eg: Arduino Programming Port, Arduino DUE Programming and AVRISP mkII)
>> Upload the code again and check the values
>> Error: Unable to find device <|> Check if rosserial is running 
>> Check the data transmission between Master Arduino and Slave Arduino (SCL/SDA)
>> Slave arduino gives initial ticks in setup() to get reset effect as and when the Master resets itself
>> Data transmission from slave arduino has been shifted from IIC/I2C (SCL/SDA) to UART (Rx/Tx) communication to avoid sync problems between master and slave after reset.
    For only real time applications, IIC is preferred

D) ROS AND ROSSERIAL

>> Use ROS Hydro version for the run
>> Error: Unable to sync with device <|> Comment all Serial.prints in the arduino code to be uploaded, before running rosserial (use printr function and comment out #define ROS_ON)
>> Match the port and the baud rate used in the ROS command
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
>> Use this i2c code in Master in case UART communication fails 

    Wire.requestFrom(2, 9);    // request 6 bytes from slave device #2
    String a;
    
    while (Wire.available()){   // slave may send less than requested
      char c = Wire.read(); // receive a byte as character
      if (c != 's') 
        a += c; // print the character
      
      else 
        break;
      
    }
    
    init_ticksr = atoi(a.c_str());

F) DEBUG PRIORITIES

>> 0 - dtl,dtr,DistL,DistR
>> 1 - RPM-L,RPM-R (Set)
>> 2 - setvelocity,omega
>> 3 - pose_x,pose_y,theta
>> 4 - ticksl,ticksr,theta
>> 5 - RPMR,RPML (Feedback)
>> 6 - theta,error
*/
