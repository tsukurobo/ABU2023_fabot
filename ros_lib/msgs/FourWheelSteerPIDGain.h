#ifndef _ROS_msgs_FourWheelSteerPIDGain_h
#define _ROS_msgs_FourWheelSteerPIDGain_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace msgs
{

  class FourWheelSteerPIDGain : public ros::Msg
  {
    public:
      float Vkp[4];
      float Vki[4];
      float Vkd[4];
      float Pkp[4];
      float Pki[4];
      float Pkd[4];

    FourWheelSteerPIDGain():
      Vkp(),
      Vki(),
      Vkd(),
      Pkp(),
      Pki(),
      Pkd()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Vkpi;
      u_Vkpi.real = this->Vkp[i];
      *(outbuffer + offset + 0) = (u_Vkpi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Vkpi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Vkpi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Vkpi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Vkp[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Vkii;
      u_Vkii.real = this->Vki[i];
      *(outbuffer + offset + 0) = (u_Vkii.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Vkii.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Vkii.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Vkii.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Vki[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Vkdi;
      u_Vkdi.real = this->Vkd[i];
      *(outbuffer + offset + 0) = (u_Vkdi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Vkdi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Vkdi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Vkdi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Vkd[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Pkpi;
      u_Pkpi.real = this->Pkp[i];
      *(outbuffer + offset + 0) = (u_Pkpi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pkpi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Pkpi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Pkpi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Pkp[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Pkii;
      u_Pkii.real = this->Pki[i];
      *(outbuffer + offset + 0) = (u_Pkii.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pkii.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Pkii.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Pkii.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Pki[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Pkdi;
      u_Pkdi.real = this->Pkd[i];
      *(outbuffer + offset + 0) = (u_Pkdi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Pkdi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Pkdi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Pkdi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Pkd[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Vkpi;
      u_Vkpi.base = 0;
      u_Vkpi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Vkpi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Vkpi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Vkpi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Vkp[i] = u_Vkpi.real;
      offset += sizeof(this->Vkp[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Vkii;
      u_Vkii.base = 0;
      u_Vkii.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Vkii.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Vkii.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Vkii.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Vki[i] = u_Vkii.real;
      offset += sizeof(this->Vki[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Vkdi;
      u_Vkdi.base = 0;
      u_Vkdi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Vkdi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Vkdi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Vkdi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Vkd[i] = u_Vkdi.real;
      offset += sizeof(this->Vkd[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Pkpi;
      u_Pkpi.base = 0;
      u_Pkpi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pkpi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Pkpi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Pkpi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Pkp[i] = u_Pkpi.real;
      offset += sizeof(this->Pkp[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Pkii;
      u_Pkii.base = 0;
      u_Pkii.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pkii.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Pkii.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Pkii.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Pki[i] = u_Pkii.real;
      offset += sizeof(this->Pki[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_Pkdi;
      u_Pkdi.base = 0;
      u_Pkdi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Pkdi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Pkdi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Pkdi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Pkd[i] = u_Pkdi.real;
      offset += sizeof(this->Pkd[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "msgs/FourWheelSteerPIDGain"; };
    virtual const char * getMD5() override { return "b389fd25bce393643445f039d84cabc9"; };

  };

}
#endif
