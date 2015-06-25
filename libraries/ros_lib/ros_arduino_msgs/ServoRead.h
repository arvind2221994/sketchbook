#ifndef _ROS_SERVICE_ServoRead_h
#define _ROS_SERVICE_ServoRead_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_arduino_msgs
{

static const char SERVOREAD[] = "ros_arduino_msgs/ServoRead";

  class ServoReadRequest : public ros::Msg
  {
    public:
      uint8_t id;

<<<<<<< HEAD
    ServoReadRequest():
      id(0)
    {
    }

=======
>>>>>>> 114c33badc7e157f5d09deb42c0831bdbf295d3c
    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
     return offset;
    }

    const char * getType(){ return SERVOREAD; };
    const char * getMD5(){ return "541b98e964705918fa8eb206b65347b3"; };

  };

  class ServoReadResponse : public ros::Msg
  {
    public:
      float value;

<<<<<<< HEAD
    ServoReadResponse():
      value(0)
    {
    }

=======
>>>>>>> 114c33badc7e157f5d09deb42c0831bdbf295d3c
    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value = u_value.real;
      offset += sizeof(this->value);
     return offset;
    }

    const char * getType(){ return SERVOREAD; };
    const char * getMD5(){ return "0aca93dcf6d857f0e5a0dc6be1eaa9fb"; };

  };

  class ServoRead {
    public:
    typedef ServoReadRequest Request;
    typedef ServoReadResponse Response;
  };

}
#endif
