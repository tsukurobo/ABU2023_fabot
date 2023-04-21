#ifndef _ROS_msgs_FourWheelSteerRad_h
#define _ROS_msgs_FourWheelSteerRad_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msgs
{

  class FourWheelSteerRad : public ros::Msg
  {
    public:
      float angle[4];
      float angVel[4];
      typedef bool _stop_type;
      _stop_type stop;

    FourWheelSteerRad():
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
        float real;
        uint32_t base;
      } u_anglei;
      u_anglei.real = this->angle[i];
      *(outbuffer + offset + 0) = (u_anglei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_anglei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_anglei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_angVeli;
      u_angVeli.real = this->angVel[i];
      *(outbuffer + offset + 0) = (u_angVeli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angVeli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angVeli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angVeli.base >> (8 * 3)) & 0xFF;
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
        float real;
        uint32_t base;
      } u_anglei;
      u_anglei.base = 0;
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle[i] = u_anglei.real;
      offset += sizeof(this->angle[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_angVeli;
      u_angVeli.base = 0;
      u_angVeli.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angVeli.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angVeli.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angVeli.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
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

    virtual const char * getType() override { return "msgs/FourWheelSteerRad"; };
    virtual const char * getMD5() override { return "7e8ad6506aee590fa635eb8c98738348"; };

  };

}
#endif
