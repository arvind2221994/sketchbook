#ifndef _ROS_scanning_table_msgs_scanning_tableAction_h
#define _ROS_scanning_table_msgs_scanning_tableAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "scanning_table_msgs/scanning_tableActionGoal.h"
#include "scanning_table_msgs/scanning_tableActionResult.h"
#include "scanning_table_msgs/scanning_tableActionFeedback.h"

namespace scanning_table_msgs
{

  class scanning_tableAction : public ros::Msg
  {
    public:
      scanning_table_msgs::scanning_tableActionGoal action_goal;
      scanning_table_msgs::scanning_tableActionResult action_result;
      scanning_table_msgs::scanning_tableActionFeedback action_feedback;

    scanning_tableAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "scanning_table_msgs/scanning_tableAction"; };
    const char * getMD5(){ return "929fc3e1b4db4c1fb852d18d42f5940b"; };

  };

}
#endif