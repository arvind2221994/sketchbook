#ifndef _ROS_scanning_table_msgs_scanning_tableActionGoal_h
#define _ROS_scanning_table_msgs_scanning_tableActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "scanning_table_msgs/scanning_tableGoal.h"

namespace scanning_table_msgs
{

  class scanning_tableActionGoal : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      scanning_table_msgs::scanning_tableGoal goal;

    scanning_tableActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "scanning_table_msgs/scanning_tableActionGoal"; };
    const char * getMD5(){ return "4e80ec6e7ae030d3965a8db4ee46c8e2"; };

  };

}
#endif