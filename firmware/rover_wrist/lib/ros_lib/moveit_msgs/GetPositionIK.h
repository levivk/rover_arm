#ifndef _ROS_SERVICE_GetPositionIK_h
#define _ROS_SERVICE_GetPositionIK_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "moveit_msgs/PositionIKRequest.h"
#include "moveit_msgs/RobotState.h"

namespace moveit_msgs
{

static const char GETPOSITIONIK[] = "moveit_msgs/GetPositionIK";

  class GetPositionIKRequest : public ros::Msg
  {
    public:
      typedef moveit_msgs::PositionIKRequest _ik_request_type;
      _ik_request_type ik_request;

    GetPositionIKRequest():
      ik_request()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->ik_request.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->ik_request.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETPOSITIONIK; };
    const char * getMD5(){ return "eeefc1583005285ade0c3481acd909d0"; };

  };

  class GetPositionIKResponse : public ros::Msg
  {
    public:
      typedef moveit_msgs::RobotState _solution_type;
      _solution_type solution;
      typedef moveit_msgs::MoveItErrorCodes _error_code_type;
      _error_code_type error_code;

    GetPositionIKResponse():
      solution(),
      error_code()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->solution.serialize(outbuffer + offset);
      offset += this->error_code.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->solution.deserialize(inbuffer + offset);
      offset += this->error_code.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETPOSITIONIK; };
    const char * getMD5(){ return "2e043f38dab80c4f64398740f13be497"; };

  };

  class GetPositionIK {
    public:
    typedef GetPositionIKRequest Request;
    typedef GetPositionIKResponse Response;
  };

}
#endif
