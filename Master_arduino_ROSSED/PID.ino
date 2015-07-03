/****************** PID *************************/

float kp = 0.005,kd = 0.00004, ki = 0.00;
float error = 0, last_error = 0, prv_error = 0, ttl_error, output = 0, prv_output = 0;
float PID(float setvalue, float value , float dT)
{
  setvalue =0;
  error = setvalue - value;
if (error<-180 )
  error = error +360 ;
 else if (error >180)
  error += -360; 
  printr("theta:" + String(value) + ",  error:" + String(error),6);
  ttl_error =  error + prv_error;
  output = kp * error - kd * (error - last_error) / dT + ki * ttl_error * dT;
  prv_error = error;
  return (output);
}

/************************************************/
