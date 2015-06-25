#ifndef _ROS_SERVICE_ServoWrite_h
#define _ROS_SERVICE_ServoWrite_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_arduino_msgs
{

static const char SERVOWRITE[] = "ros_arduino_msgs/ServoWrite";

  class ServoWriteRequest : public ros::Msg
  {
    public:
      uint8_t id;
      float value;

<<<<<<< HEAD
    ServoWriteRequest():
      id(0),
      value(0)
    {
    }

=======
>>>>>>> 114c33badc7e157f5d09deb42c0831bdbf295d3c
    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
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
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
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

    const char * getType(){ return SERVOWRITE; };
    const char * getMD5(){ return "f90a4a27fdac2d3886d60d19d2b742b2"; };

  };

  class ServoWriteResponse : public ros::Msg
  {
    public:

<<<<<<< HEAD
    ServoWriteResponse()
    {
    }

=======
>>>>>>> 114c33badc7e157f5d09deb42c0831bdbf295d3c
    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SERVOWRITE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ServoWrite {
    public:
    typedef ServoWriteRequest Request;
    typedef ServoWriteResponse Response;
  };

}
#endif
