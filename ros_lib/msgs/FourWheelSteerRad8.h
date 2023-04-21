#ifndef _ROS_msgs_FourWheelSteerRad8_h
#define _ROS_msgs_FourWheelSteerRad8_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msgs
{

  class FourWheelSteerRad8 : public ros::Msg
  {
    public:
      int8_t angle[4];
      int8_t angVel[4];
      typedef bool _stop_type;
      _stop_type stop;

    FourWheelSteerRad8():
      angle(),
      angVel(),
      stop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_anglei;
      u_anglei.real = this->angle[i];
      *(outbuffer + offset + 0) = (u_anglei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angle[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_angVeli;
      u_angVeli.real = this->angVel[i];
      *(outbuffer + offset + 0) = (u_angVeli.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angVel[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.real = this->stop;
      *(outbuffer + offset + 0) = (u_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_anglei;
      u_anglei.base = 0;
      u_anglei.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->angle[i] = u_anglei.real;
      offset += sizeof(this->angle[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_angVeli;
      u_angVeli.base = 0;
      u_angVeli.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->angVel[i] = u_angVeli.real;
      offset += sizeof(this->angVel[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.base = 0;
      u_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stop = u_stop.real;
      offset += sizeof(this->stop);
     return offset;
    }

    virtual const char * getType() override { return "msgs/FourWheelSteerRad8"; };
    virtual const char * getMD5() override { return "eb587027f631d39cd824a4dba7b5ae76"; };

  };

}
#endif
