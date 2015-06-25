#ifndef _ROS_scanning_table_msgs_scanning_tableResult_h
#define _ROS_scanning_table_msgs_scanning_tableResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace scanning_table_msgs
{

  class scanning_tableResult : public ros::Msg
  {
    public:
      const char* answer;

    scanning_tableResult():
      answer("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_answer = strlen(this->answer);
      memcpy(outbuffer + offset, &length_answer, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->answer, length_answer);
      offset += length_answer;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_answer;
      memcpy(&length_answer, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_answer; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_answer-1]=0;
      this->answer = (char *)(inbuffer + offset-1);
      offset += length_answer;
     return offset;
    }

    const char * getType(){ return "scanning_table_msgs/scanning_tableResult"; };
    const char * getMD5(){ return "d7e708f879c94bb931716d8f4f130f30"; };

  };

}
#endif