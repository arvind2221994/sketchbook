#ifndef _ROS_scanning_table_msgs_scanning_tableActionResult_h
#define _ROS_scanning_table_msgs_scanning_tableActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "scanning_table_msgs/scanning_tableResult.h"

namespace scanning_table_msgs
{

  class scanning_tableActionResult : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      scanning_table_msgs::scanning_tableResult result;

    scanning_tableActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "scanning_table_msgs/scanning_tableActionResult"; };
    const char * getMD5(){ return "02e1ac9d3e1bd734abafb2a87c014e4a"; };

  };

}
#endif