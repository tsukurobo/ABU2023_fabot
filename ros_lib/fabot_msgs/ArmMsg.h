#ifndef _ROS_fabot_msgs_ArmMsg_h
#define _ROS_fabot_msgs_ArmMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fabot_msgs
{

  class ArmMsg : public ros::Msg
  {
    public:
      typedef int16_t _hand_type;
      _hand_type hand;
      typedef int16_t _arm_type;
      _arm_type arm;
      typedef int16_t _hand_duty_type;
      _hand_duty_type hand_duty;
      typedef int16_t _arm_duty_type;
      _arm_duty_type arm_duty;

    ArmMsg():
      hand(0),
      arm(0),
      hand_duty(0),
      arm_duty(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_hand;
      u_hand.real = this->hand;
      *(outbuffer + offset + 0) = (u_hand.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hand.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->hand);
      union {
        int16_t real;
        uint16_t base;
      } u_arm;
      u_arm.real = this->arm;
      *(outbuffer + offset + 0) = (u_arm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arm.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->arm);
      union {
        int16_t real;
        uint16_t base;
      } u_hand_duty;
      u_hand_duty.real = this->hand_duty;
      *(outbuffer + offset + 0) = (u_hand_duty.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hand_duty.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->hand_duty);
      union {
        int16_t real;
        uint16_t base;
      } u_arm_duty;
      u_arm_duty.real = this->arm_duty;
      *(outbuffer + offset + 0) = (u_arm_duty.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arm_duty.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->arm_duty);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_hand;
      u_hand.base = 0;
      u_hand.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hand.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->hand = u_hand.real;
      offset += sizeof(this->hand);
      union {
        int16_t real;
        uint16_t base;
      } u_arm;
      u_arm.base = 0;
      u_arm.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arm.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arm = u_arm.real;
      offset += sizeof(this->arm);
      union {
        int16_t real;
        uint16_t base;
      } u_hand_duty;
      u_hand_duty.base = 0;
      u_hand_duty.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hand_duty.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->hand_duty = u_hand_duty.real;
      offset += sizeof(this->hand_duty);
      union {
        int16_t real;
        uint16_t base;
      } u_arm_duty;
      u_arm_duty.base = 0;
      u_arm_duty.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arm_duty.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arm_duty = u_arm_duty.real;
      offset += sizeof(this->arm_duty);
     return offset;
    }

    virtual const char * getType() override { return "fabot_msgs/ArmMsg"; };
    virtual const char * getMD5() override { return "7e5814a5ce25cfec12c99674e013cee3"; };

  };

}
#endif
