#ifndef _ROS_beginner_tutorials_Gen_h
#define _ROS_beginner_tutorials_Gen_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace beginner_tutorials
{

  class Gen : public ros::Msg
  {
    public:
      const char* my_name;

    Gen():
      my_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_my_name = strlen(this->my_name);
      memcpy(outbuffer + offset, &length_my_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->my_name, length_my_name);
      offset += length_my_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_my_name;
      memcpy(&length_my_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_my_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_my_name-1]=0;
      this->my_name = (char *)(inbuffer + offset-1);
      offset += length_my_name;
     return offset;
    }

    const char * getType(){ return "beginner_tutorials/Gen"; };
    const char * getMD5(){ return "587f464e921a54e4ccb9c52f3f4f4bcb"; };

  };

}
#endif