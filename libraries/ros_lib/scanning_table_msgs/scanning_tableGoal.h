#ifndef _ROS_scanning_table_msgs_scanning_tableGoal_h
#define _ROS_scanning_table_msgs_scanning_tableGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace scanning_table_msgs
{

  class scanning_tableGoal : public ros::Msg
  {
    public:
      float angle;
      float release_brake_timeout;

    scanning_tableGoal():
      angle(0),
      release_brake_timeout(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->release_brake_timeout);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->release_brake_timeout));
     return offset;
    }

    const char * getType(){ return "scanning_table_msgs/scanning_tableGoal"; };
    const char * getMD5(){ return "fb1cff9b3dded6ae8ed32031b19412d8"; };

  };

}
#endif