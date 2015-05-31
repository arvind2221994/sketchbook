#ifndef _ROS_scanning_table_msgs_scanning_tableActionFeedback_h
#define _ROS_scanning_table_msgs_scanning_tableActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "scanning_table_msgs/scanning_tableFeedback.h"

namespace scanning_table_msgs
{

  class scanning_tableActionFeedback : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalStatus status;
      scanning_table_msgs::scanning_tableFeedback feedback;

    scanning_tableActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "scanning_table_msgs/scanning_tableActionFeedback"; };
    const char * getMD5(){ return "8523755d9af2391e8a401e6c62b5a818"; };

  };

}
#endif