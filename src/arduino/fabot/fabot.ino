#include <Arduino.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

#include <ros.h>
#include "msgs/FourWheelSteerRad.h"
#include "msgs/FourWheelSteerPIDGain.h"
#include <std_msgs/Int16MultiArray.h>

#include "fabot_msgs/ArmMsg.h"

ros::NodeHandle nh;

/*アーム制御のグローバル変数*/
#define HAND_MOTOR 11  // 手のモーター番号
#define ARM_MOTOR 10   // 腕のモーター番号
#define STOP 0
#define DO_OPEN 1
#define DO_CLOSE 2
#define DO_UP 1
#define DO_DOWN 2
#define ARM_ENC 4 // 腕のアブソリュートエンコーダ番号
#define UP_LIMIT 12000 // adbotの高さに合わせてあとで調整する
#define DOWN_LIMIT 5893

int hand_state = 0;
int arm_state = 0;
int hand_duty = 0;
int arm_duty = 0;
const int arm_bias_duty = 200;


/*ステア制御のグローバル変数*/
const uint8_t driveMotorNum[] = {4, 2, 0, 6}, steerMotorNum[] = {5, 3, 1, 7};
const uint8_t incEncNum[] = {2, 1, 0, 3}, absEncNum[] = {2, 1, 0, 3}; 

float angle[4], angVel[4];
float Vkp[4], Vki[4], Vkd[4], Pkp[4], Pki[4], Pkd[4];
const uint16_t INC_CPR = 2048;
const double steerCapableDuty = 0.3, driveCapableDuty[] = {0.312, 0.3, 0.325, 0.3};
bool Stop = true;

std_msgs::Int16MultiArray duty_msg, enc_msg;
msgs::FourWheelSteerRad rad_msg;


/*アーム制御のコールバック関数*/
void loadRingCallback(const fabot_msgs::ArmMsg &arm_msg) {
  // messageが0なら停止、1なら開く、2なら閉じる
  hand_state = arm_msg.hand;
  arm_state = arm_msg.arm;

  hand_duty = arm_msg.hand_duty;
  arm_duty = arm_msg.arm_duty;
}


/*ステア制御のコールバック関数*/
void targetCb(const msgs::FourWheelSteerRad &target) {
  for(int i = 0; i < 4; i++) {
    angle[i] = target.angle[i];
    angVel[i] = -target.angVel[i];
    Stop = target.stop;
  }
}

void gainCb(const msgs::FourWheelSteerPIDGain &gain) {
  for(int i = 0; i < 4; i++) {
    Vkp[i] = gain.Vkp[i];
    Vki[i] = gain.Vki[i];
    Vkd[i] = gain.Vkd[i];
    Pkp[i] = gain.Pkp[i];
    Pki[i] = gain.Pki[i];
    Pkd[i] = gain.Pkd[i];
  }
}


/*アーム制御のサブスクライバ*/
ros::Subscriber<fabot_msgs::ArmMsg> sub("hand_state", &loadRingCallback);


/*ステア制御のサブスクライバ・パブリッシャー*/
ros::Subscriber<msgs::FourWheelSteerRad> target_sub("target", targetCb);
ros::Subscriber<msgs::FourWheelSteerPIDGain> gain_sub("gain", gainCb);
ros::Publisher duty_pub("duty", &duty_msg);
ros::Publisher enc_pub("enc", &enc_msg);
ros::Publisher rad_pub("rad", &rad_msg);


void setup()
{
  Cubic::begin();
  
  nh.getHardware()->setBaud(2000000);
  nh.initNode();

  nh.subscribe(sub);

  duty_msg.data = (int16_t*)malloc(sizeof(int16_t)*8);
  duty_msg.data_length = 8;
  enc_msg.data = (int16_t*)malloc(sizeof(int16_t)*8);
  enc_msg.data_length = 8;

  nh.subscribe(target_sub);
  nh.subscribe(gain_sub);
  nh.advertise(duty_pub);
  nh.advertise(enc_pub);
  nh.advertise(rad_pub);
}

void loop()
{
  nh.spinOnce();

  /*アーム制御*/
  if (hand_state == STOP) {
    digitalWrite(23, HIGH);
    DC_motor::put(HAND_MOTOR, 0);
  } else if (hand_state == DO_OPEN) {
    digitalWrite(23, LOW);
    DC_motor::put(HAND_MOTOR, hand_duty);
  } else if (hand_state == DO_CLOSE) {
    digitalWrite(23, LOW);
    DC_motor::put(HAND_MOTOR, -hand_duty);
  }
  
  if (arm_state == STOP) {
    digitalWrite(24, HIGH);
    DC_motor::put(ARM_MOTOR, 0);
  } else if (arm_state == DO_UP) {
    digitalWrite(24, LOW);
    uint16_t enc_val = Abs_enc::get(ARM_ENC);
    if (enc_val < UP_LIMIT && enc_val != ABS_ENC_ERR && enc_val != ABS_ENC_ERR_RP2040) {
      double angle = (enc_val - DOWN_LIMIT) / (double)ABS_ENC_MAX * TWO_PI;
      DC_motor::put(ARM_MOTOR, -(arm_duty + cos(angle)*arm_bias_duty));
    }
    else {
      DC_motor::put(ARM_MOTOR, 0);
    }
  } else if (arm_state == DO_DOWN) {
    digitalWrite(24, LOW);
    uint16_t enc_val = Abs_enc::get(ARM_ENC);
    if (Abs_enc::get(ARM_ENC) > DOWN_LIMIT && enc_val != ABS_ENC_ERR && enc_val != ABS_ENC_ERR_RP2040) {
      double angle = (enc_val - DOWN_LIMIT) / (double)ABS_ENC_MAX * TWO_PI;
      DC_motor::put(ARM_MOTOR, +(arm_duty - cos(angle)*arm_bias_duty));
    }
    else {
      DC_motor::put(ARM_MOTOR, 0);
    }
  }


  /*ステア制御*/
  using namespace Cubic_controller;
  static Velocity_PID drivePID[] = {
    {driveMotorNum[0], incEncNum[0], encoderType::inc, INC_CPR, driveCapableDuty[0], Vkp[0], Vki[0], Vkd[0], angVel[0], false, false},
    {driveMotorNum[1], incEncNum[1], encoderType::inc, INC_CPR, driveCapableDuty[1], Vkp[1], Vki[1], Vkd[1], angVel[1], false, false},
    {driveMotorNum[2], incEncNum[2], encoderType::inc, INC_CPR, driveCapableDuty[2], Vkp[2], Vki[2], Vkd[2], angVel[2], false, false},
    {driveMotorNum[3], incEncNum[3], encoderType::inc, INC_CPR, driveCapableDuty[3], Vkp[3], Vki[3], Vkd[3], angVel[3], false, false},
  };
  static Position_PID steerPID[] = {
    {steerMotorNum[0], absEncNum[0], encoderType::abs, AMT22_CPR, steerCapableDuty, Pkp[0], Pki[0], Pkd[0], angle[0], false, false},
    {steerMotorNum[1], absEncNum[1], encoderType::abs, AMT22_CPR, steerCapableDuty, Pkp[1], Pki[1], Pkd[1], angle[1], true, false},
    {steerMotorNum[2], absEncNum[2], encoderType::abs, AMT22_CPR, steerCapableDuty, Pkp[2], Pki[2], Pkd[2], angle[2], false, false},
    {steerMotorNum[3], absEncNum[3], encoderType::abs, AMT22_CPR, steerCapableDuty, Pkp[3], Pki[3], Pkd[3], angle[3], false, false},
  };

  if (Stop) {
    for(int i = 0; i < 4; i++) {
      DC_motor::put(driveMotorNum[i], 0);
      DC_motor::put(steerMotorNum[i], 0);
    }
  }
  else {
    for(int i = 0; i < 4; i++) {
      drivePID[i].setGains(Vkp[i], Vki[i], Vkd[i]);
      drivePID[i].setTarget(angVel[i]);
      drivePID[i].compute();

      steerPID[i].setGains(Pkp[i], Pki[i], Pkd[i]);
      steerPID[i].setTarget(angle[i]);
      steerPID[i].compute();
      // DC_motor::put(steerMotorNum[i], 200);
    }
  }

  for(int i = 0; i < 4; i++) {
    enc_msg.data[i]   = Inc_enc::get(incEncNum[i]);
    enc_msg.data[i+4] = Abs_enc::get(absEncNum[i]);
    duty_msg.data[i] = DC_motor::get(driveMotorNum[i]);
    duty_msg.data[i+4] = DC_motor::get(steerMotorNum[i]);
    rad_msg.angVel[i] = encoderToAngle(Inc_enc::get_diff(incEncNum[i]), INC_CPR);
    rad_msg.angle[i]  = encoderToAngle(Abs_enc::get(absEncNum[i]), AMT22_CPR);
  }
  // enc_pub.publish(&enc_msg);
  // duty_pub.publish(&duty_msg);
  // rad_pub.publish(&rad_msg);

  Cubic::update();
}