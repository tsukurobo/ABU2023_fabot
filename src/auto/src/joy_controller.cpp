#include <ros/ros.h>
#include <math.h>
#include <msgs/FourWheelSteerRad.h>
#include <msgs/FourWheelSteerPIDGain.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <fabot_msgs/ArmMsg.h>
#include "FourWheelSteer.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "PurePursuit.h"
#include "visualization_msgs/MarkerArray.h"

#define ENABLE_BUTTON 5
#define AUTO_LOAD_BUTTON 4
#define AUTO_PICKUP_AXE 2
#define VY_AXE 0
#define VX_AXE 1
#define WX_AXE 3 //2
#define WY_AXE 4 //3
#define XVEHICLE_AXE 7 //5
#define YVEHICLE_AXE 6 //4
#define ROTATE_BUTTON_0 6 //10
#define ROTATE_BUTTON_1 7 //11

#define CLOSE_HAND_BUTTON 2 //Xボタン
#define OPEN_HAND_BUTTON 1 //Bボタン
#define UP_ARM_BUTTON 3 //Yボタン
#define DOWN_ARM_BUTTON 0 //Aボタン
#define CLOSE_HAND 2
#define OPEN_HAND 1
#define STOP_HAND 0
#define UP_ARM 1
#define DOWN_ARM 2
#define STOP_ARM 0
int hand_duty = 470; //Arduinoに指定するDuty
int arm_duty = 470;
fabot_msgs::ArmMsg arm_state_msg;

using namespace std;

msgs::FourWheelSteerRad target;
msgs::FourWheelSteerPIDGain pid_gain;

string mode = "XVEHICLE";

int freq = 1000;

// ステアのパラメータ
FourWheelSteer steer((80.0+0.92)*(21.0/49.0)*M_PI, 310.0, 350.0);
double v_max = 1.0, w_max = M_PI, TurnRadius_min = 0.8, TurnRadius_max = 1e6;

// PIDのパラメータ
double Vkp[4], Vki[4], Vkd[4];
double Pkp[4], Pki[4], Pkd[4];

// ステアの角速度と座標
double angVel[4], angle[4];
double x, y, theta;

// 自律走行のパラメータ
bool auto_flag = false;
double auto_vx = 0.3;
visualization_msgs::MarkerArray marker_array;

point pass0[] = {{-0.1, 0.0}, {4.0, 0.0}};
uint pass_num0 = 2;
double look_ahead_dist0 = 2.0;
PurePursuit pp0(pass0, pass_num0, look_ahead_dist0, 1.0);

point pass1[] = {{5.0, 0.0}, {0.0, 0.0}};
uint pass_num1 = 2;
double look_ahead_dist1 = 2.0;
PurePursuit pp1(pass1, pass_num1, look_ahead_dist1, 1.0);

// スキャンデータ
sensor_msgs::LaserScan front_scan, back_scan;

// リング自動回収のパラメータ
bool auto_pickup_flag = false;
double pickup_dist = 0.14, pickup_vel = 0.1;
double close_hand_time = 2.0, pickup_arm_time = 1.0;

// 自動装填のパラメータ
bool auto_load_flag = false;
// 平面を判定する際に見る両端からの要素数、トゲを判定する際に見る中心からの要素数
int flat_range = 5, needle_range = 4;
// 平面を判定する際の傾きの誤差の許容値、トゲを判定する際の傾きの差の閾値
double flat_threshold = 0.10, needle_threshold = 1;
double parallel_stop_time = 1.0;
// 正面を向いたと判定する傾きの許容値、装填を行う距離の閾値
double flat_angle_max = 0.05, load_dist = 0.10;
// 角度を合わせる角速度、距離を合わせる速度
double load_w = 0.03, load_vel = 0.03;
// 装填を行う際にアームを上げる時間、ハンドを開く時間
double load_arm_time = 1.0, open_hand_time = 1.0;

string autoPickup() {
    static string mode = "XVEHICLE";
    static ros::Time close_start_time, up_start_time;
    static ros::Time last_time = ros::Time::now();

    // しばらく呼び出されないときはXVEHICLEモードに戻す
    ros::Time now_time = ros::Time::now();
    if (now_time.toSec() - last_time.toSec() > 0.5) {
        mode = "XVEHICLE";
    }
    last_time = now_time;

    if (mode == "XVEHICLE") {
        // アームを下げる
        arm_state_msg.arm = DOWN_ARM;

        // 値の強度（信頼性）の平均を求める
        double intensities_mean = 0.0;
        for (int i = 0; i < front_scan.intensities.size(); i++) {
            intensities_mean += front_scan.intensities[i];
        }
        intensities_mean /= (double)front_scan.intensities.size();

        // 最小値を求める
        double min_dist = 1e6;
        int min_idx = 0;
        for (int i = 0; i < front_scan.ranges.size(); i++) {
            if (front_scan.ranges[i] < min_dist && front_scan.intensities[i] > intensities_mean) {
                min_dist = front_scan.ranges[i];
                min_idx = i;
            }
        }

        // リングとの方位誤差を計算し、その方向に速度を与える
        double angle = front_scan.angle_min + front_scan.angle_increment * min_idx;
        double w = 2*pickup_vel*sin(angle)/min_dist;

        target.stop = false;
        steer.xVehicle(pickup_vel, w);

        // リングが近づいたら止まってハンドを閉じる
        if (min_dist < pickup_dist) {
            mode = "CLOSE_HAND";
            target.stop = true;
            steer.stop();
            close_start_time = ros::Time::now();
            ROS_INFO_STREAM("CLOSE_HAND");
        }
    }
    else if (mode == "CLOSE_HAND") {
        arm_state_msg.hand = CLOSE_HAND;
        if (ros::Time::now().toSec() - close_start_time.toSec() > close_hand_time) {
            mode = "UP_ARM";
            arm_state_msg.hand = STOP_HAND;
            up_start_time = ros::Time::now();
            ROS_INFO_STREAM("UP_ARM");
        }
    }
    else if (mode == "UP_ARM") {
        arm_state_msg.arm = UP_ARM;
        if (ros::Time::now().toSec() - up_start_time.toSec() > pickup_arm_time) {
            mode = "FINISH";
            arm_state_msg.arm = STOP_ARM;
            ROS_INFO_STREAM("FINISH");
        }
    }
    
    return mode;
}

point polar_to_cartesian(double r, double theta) {
    point p;
    p.x = r * cos(theta);
    p.y = r * sin(theta);
    return p;
}

bool is_flat(uint start_num, uint end_num, double threshold, uint range = 5) {
    double r0 = back_scan.ranges[start_num];
    double r1 = back_scan.ranges[end_num];
    double theta0 = back_scan.angle_min + back_scan.angle_increment * start_num + M_PI_2; // LiDARの正面をy軸の正方向とする
    double theta1 = back_scan.angle_min + back_scan.angle_increment * end_num + M_PI_2;
    point p0 = polar_to_cartesian(r0, theta0);
    point p1 = polar_to_cartesian(r1, theta1);
    double _slope = (p1.y - p0.y) / (p1.x - p0.x);
    for (int i = 1; i < min((end_num - start_num) / 2, range); i++) {
        r0 = back_scan.ranges[start_num + i];
        r1 = back_scan.ranges[end_num - i];
        theta0 = back_scan.angle_min + back_scan.angle_increment * (start_num + i) + M_PI_2;
        theta1 = back_scan.angle_min + back_scan.angle_increment * (end_num - i) + M_PI_2;
        p0 = polar_to_cartesian(r0, theta0);
        p1 = polar_to_cartesian(r1, theta1);
        double slope = (p1.y - p0.y) / (p1.x - p0.x);
        if (fabs(slope - _slope) > threshold) {
            return false;
        }
    }
    return true;
}

bool is_needle(uint num, double threshold, uint range = 3) {
    double r0 = back_scan.ranges[num];
    double r1 = back_scan.ranges[num + range];
    double r2 = back_scan.ranges[num - range];
    double theta0 = back_scan.angle_min + back_scan.angle_increment * num + M_PI_2;
    double theta1 = back_scan.angle_min + back_scan.angle_increment * (num + range) + M_PI_2;
    double theta2 = back_scan.angle_min + back_scan.angle_increment * (num - range) + M_PI_2;
    point p0 = polar_to_cartesian(r0, theta0);
    point p1 = polar_to_cartesian(r1, theta1);
    point p2 = polar_to_cartesian(r2, theta2);
    double slope0 = (p1.y - p0.y) / (p1.x - p0.x);
    double slope1 = (p2.y - p0.y) / (p2.x - p0.x);
    if (fabs(slope0 - slope1) > threshold) {
        return true;
    }
    return false;
}

string autoLoad() {
    static string mode = "ROTATE";
    static ros::Time parallel_start_time, up_start_time, open_start_time;
    static ros::Time last_time = ros::Time::now();

    // しばらく呼び出されないときはROTATEモードに戻す
    ros::Time now_time = ros::Time::now();
    if (now_time.toSec() - last_time.toSec() > 0.5) {
        mode = "ROTATE";
    }
    last_time = now_time;

    const int middle = back_scan.ranges.size() / 2;

    if (mode == "ROTATE") {
        int flat_idx = -1;
        vec flat;

        // 平面の角度を計算
        for (int i = 0; i < middle; i++) {
            if (is_flat(i, back_scan.ranges.size() - i - 1, flat_threshold, flat_range)) {
                flat_idx = i;
                point p0 = polar_to_cartesian(back_scan.ranges[i], back_scan.angle_min + back_scan.angle_increment * i);
                point p1 = polar_to_cartesian(back_scan.ranges[back_scan.ranges.size() - i - 1], back_scan.angle_min + back_scan.angle_increment * (back_scan.ranges.size() - i - 1));
                double dist = sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2));
                flat.x = (p1.x - p0.x) / dist; flat.y = (p1.y - p0.y) / dist;
                ROS_INFO("FLAT: (%f, %f), (%f, %f)\n", p0.x, p0.y, p1.x, p1.y);
                break;
            }
        }
        if (flat_idx == -1) {
            target.stop = true;
            steer.rotate(0);
            ROS_INFO_STREAM("FLAT NOT FOUND");
            return "ERROR";
        }

        target.stop = false;
        if (flat.x < 0) steer.rotate(-load_w); // 符号は要検証
        else            steer.rotate(load_w);

        // 平面と機体の角度が合っている場合はPARALLELモードに移行
        if (abs(atan2(flat.x, flat.y)) < flat_angle_max) {
            mode = "PARALLEL";
            ROS_INFO_STREAM("PARALLEL");
            steer.stop();
            parallel_start_time = ros::Time::now();
        }
    }
    else if (mode == "PARALLEL") {
        // 目印となる平面上の突起を探す
        int needle_idx = -1;
        // for (int i = 0; i < middle - needle_range - 1; i++) {
        //     if (is_needle(middle + i, needle_threshold, needle_range)) {
        //         needle_idx = middle + i;
        //         break;
        //     }
        //     else if (is_needle(middle - i, needle_threshold, needle_range)) {
        //         needle_idx = middle - i;
        //         break;
        //     }
        // }

        // 最小値を求める
        double min_dist = 1e6;
        for (int i = 0; i < back_scan.ranges.size(); i++) {
            if (back_scan.ranges[i] < min_dist) {
                min_dist = back_scan.ranges[i];
                needle_idx = i;
            }
        }
        
        if (needle_idx == -1) {
            target.stop = true;
            steer.parallel(0, 0);
            ROS_INFO_STREAM("NEEDLE NOT FOUND");
            return "ERROR";
        }

        point needl = polar_to_cartesian(back_scan.ranges[needle_idx], back_scan.angle_min + back_scan.angle_increment * needle_idx);
        ROS_INFO("NEEDL: %f, %f\n", needl.x, needl.y);
        if (load_dist < 0.10) needl.x += 0.10 - load_dist;
        double dist = sqrt(pow(needl.x, 2) + pow(needl.y, 2));

        vec needl_vec = {needl.x / dist, needl.y / dist}; // 符号は要検証
        target.stop = false;
        steer.parallel(-load_vel*needl_vec.x, load_vel*needl_vec.y);

        if (ros::Time::now().toSec() - parallel_start_time.toSec() < parallel_stop_time) {
            steer.stop();
        }

        // 平面上の突起との距離が近い場合はUP_ARMモードに移行
        if (dist < load_dist) {
            mode = "UP_ARM";
            target.stop = true;
            steer.stop();
            up_start_time = ros::Time::now();
            ROS_INFO_STREAM("UP_ARM");
        }
    }
    else if (mode == "UP_ARM") {
        arm_state_msg.arm = UP_ARM;
        if (ros::Time::now().toSec() - up_start_time.toSec() > load_arm_time) {
            mode = "OPEN_HAND";
            arm_state_msg.arm = STOP_ARM;
            open_start_time = ros::Time::now();
            ROS_INFO_STREAM("OPEN_HAND");
        }
    }
    else if (mode == "OPEN_HAND") {
        arm_state_msg.hand = OPEN_HAND;
        if (ros::Time::now().toSec() - open_start_time.toSec() > open_hand_time) {
            mode = "FINISH";
            arm_state_msg.hand = STOP_HAND;
            ROS_INFO_STREAM("FINISH");
        }
    }

    return mode;
}

string auto0() {
    target.stop = false;
    double turn_radius = pp0.compute_turn_radius(x, y, theta);
    if (pp0.get_finish_flag()) {
        steer.stop();
        target.stop = true;
        ROS_INFO_STREAM("AUTO FINISH");
        return "FINISH";
    }
    double w = auto_vx / turn_radius;
    steer.xVehicle(auto_vx, w);
    marker_array = pp0.get_marker("odom");
    return "RUNNING";
}

string auto1() {
    target.stop = false;
    double turn_radius = pp1.compute_turn_radius(x, y, theta + M_PI);
    if (pp1.get_finish_flag()) {
        steer.stop();
        target.stop = true;
        ROS_INFO_STREAM("AUTO FINISH");
        return "FINISH";
    }
    double w = -auto_vx / turn_radius;
    steer.xVehicle(-auto_vx, w);
    marker_array = pp1.get_marker("odom");
    return "RUNNING";
}

void Auto() {
    static string mode = "auto0";
    if (mode == "auto0") {
        if(auto0() == "FINISH") {
            mode = "pickup";
        }
    }
    else if (mode == "pickup") {
        if(autoPickup() == "UP_ARM") {
            mode = "auto1";
        }
    }
    else if (mode == "auto1") {
        if(auto1() == "FINISH") {
            mode = "auto0";
        }
    }
}

void joyCb(const sensor_msgs::Joy &joy_msg) {
    double vx = 0.0, vy = 0.0, wx = 0.0, wy = 0.0;
    if (joy_msg.buttons[ENABLE_BUTTON]) {
        auto_flag = false;
        target.stop = false;
        vy =  joy_msg.axes[VY_AXE] * joy_msg.axes[VY_AXE] * copysign(v_max, joy_msg.axes[VY_AXE]);
        vx =  joy_msg.axes[VX_AXE] * joy_msg.axes[VX_AXE] * copysign(v_max, joy_msg.axes[VX_AXE]);
        wx  = joy_msg.axes[WX_AXE] * joy_msg.axes[WX_AXE] * copysign(w_max, joy_msg.axes[WX_AXE]);
        wy  = -joy_msg.axes[WY_AXE] * joy_msg.axes[WY_AXE] * copysign(w_max, joy_msg.axes[WY_AXE]);
        ROS_INFO_STREAM("MANUAL " + mode);

        if (mode == "XVEHICLE") {
            steer.xVehicle(vx, wx);
        }
        else if (mode == "YVEHICLE") {
            steer.yVehicle(vy, wy);
        }
        else if (mode == "PARALLEL") {
            steer.parallel(vx, vy);
        }
        else if (mode == "ROTATE") {
            steer.rotate(wx);
        }

        //両方押してるときは手は停止
        if(joy_msg.buttons[OPEN_HAND_BUTTON]==1 && joy_msg.buttons[CLOSE_HAND_BUTTON]==0){
            arm_state_msg.hand = OPEN_HAND;
        }else if(joy_msg.buttons[CLOSE_HAND_BUTTON]==1 && joy_msg.buttons[OPEN_HAND_BUTTON]==0){
            arm_state_msg.hand = CLOSE_HAND;
        }else{
            arm_state_msg.hand = STOP_HAND;
        }
        //両方押してるときは腕は停止
        if(joy_msg.buttons[UP_ARM_BUTTON]==1 && joy_msg.buttons[DOWN_ARM_BUTTON]==0){
            arm_state_msg.arm = UP_ARM;
        }else if(joy_msg.buttons[DOWN_ARM_BUTTON]==1 && joy_msg.buttons[UP_ARM_BUTTON]==0){
            arm_state_msg.arm = DOWN_ARM;
        }else{
            arm_state_msg.arm = STOP_ARM;
        }

        if (joy_msg.axes[AUTO_PICKUP_AXE] < -0.9) {
            auto_pickup_flag = true;
            ROS_INFO_STREAM("AUTO_PICKUP");
        }
        else {
            auto_pickup_flag = false;
        }

        if (joy_msg.buttons[AUTO_LOAD_BUTTON]) {
            auto_load_flag = true;
            ROS_INFO_STREAM("AUTO_LOAD");
        }
        else {
            auto_load_flag = false;
        }
    }
    // else if (joy_msg.buttons[AUTO_BUTTON]) {
    //     auto_flag = true;
    //     ROS_INFO_STREAM("AUTO");
    // }
    else {
        steer.stop();
        target.stop = true;
        auto_flag = false;
        auto_pickup_flag = false;

        arm_state_msg.hand = 0;
        arm_state_msg.arm = 0;

        ROS_INFO_STREAM("STOP");
    }

    if (joy_msg.axes[XVEHICLE_AXE] == 1 || joy_msg.axes[XVEHICLE_AXE] == -1) {
        mode = "XVEHICLE";
        ROS_INFO_STREAM("MODE CHANGE: XVEHCILE");
    }
    else if (joy_msg.axes[YVEHICLE_AXE] == 1 || joy_msg.axes[YVEHICLE_AXE] == -1) {
        mode = "YVEHICLE";
        ROS_INFO_STREAM("MODE CHANGE: YVEHCILE");
    }
    else if (joy_msg.buttons[ROTATE_BUTTON_0] == 1 || joy_msg.buttons[ROTATE_BUTTON_1] == 1) {
        mode = "ROTATE";
        ROS_INFO_STREAM("MODE CHANGE: ROTATE");
    }
}

void frontScanCb(const sensor_msgs::LaserScan &scan_msg) {
    front_scan = scan_msg;
}

void backScanCb(const sensor_msgs::LaserScan &scan_msg) {
    back_scan = scan_msg;
}

void getCoodinate() {
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        // ros::Duration(1.0).sleep();
        return;
    }
    x = transformStamped.transform.translation.x;
    y = transformStamped.transform.translation.y;

    tf2::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta);
}

void setTarget() {
    for (int i = 0; i < 4; i++) {
        target.angle[i] = steer.getAngle(i);
        target.angVel[i] = steer.getAngVel(i);
    }
}

void setGain() {
    for (int i = 0; i < 4; i++) {
        pid_gain.Vkp[i] = Vkp[i];
        pid_gain.Vki[i] = Vki[i];
        pid_gain.Vkd[i] = Vkd[i];
        pid_gain.Pkp[i] = Pkp[i];
        pid_gain.Pki[i] = Pki[i];
        pid_gain.Pkd[i] = Pkd[i];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");
    pnh.getParam("v_max", v_max);
    if(pnh.getParam("w_max", w_max)) w_max *= M_PI;
    pnh.getParam("TurnRadius_min", TurnRadius_min);
    pnh.getParam("TurnRadius_max", TurnRadius_max);
    pnh.getParam("Vkp0", Vkp[0]);
    pnh.getParam("Vki0", Vki[0]);
    pnh.getParam("Vkd0", Vkd[0]);
    pnh.getParam("Pkp0", Pkp[0]);
    pnh.getParam("Pki0", Pki[0]);
    pnh.getParam("Pkd0", Pkd[0]);
    pnh.getParam("Vkp1", Vkp[1]);
    pnh.getParam("Vki1", Vki[1]);
    pnh.getParam("Vkd1", Vkd[1]);
    pnh.getParam("Pkp1", Pkp[1]);
    pnh.getParam("Pki1", Pki[1]);
    pnh.getParam("Pkd1", Pkd[1]);
    pnh.getParam("Vkp2", Vkp[2]);
    pnh.getParam("Vki2", Vki[2]);
    pnh.getParam("Vkd2", Vkd[2]);
    pnh.getParam("Pkp2", Pkp[2]);
    pnh.getParam("Pki2", Pki[2]);
    pnh.getParam("Pkd2", Pkd[2]);
    pnh.getParam("Vkp3", Vkp[3]);
    pnh.getParam("Vki3", Vki[3]);
    pnh.getParam("Vkd3", Vkd[3]);
    pnh.getParam("Pkp3", Pkp[3]);
    pnh.getParam("Pki3", Pki[3]);
    pnh.getParam("Pkd3", Pkd[3]);
    pnh.getParam("freq", freq);
    // リング回収のパラメータ
    pnh.getParam("hand_duty", hand_duty);
    pnh.getParam("arm_duty", arm_duty);
    pnh.getParam("pickup_dist", pickup_dist);
    pnh.getParam("pickup_vel", pickup_vel);
    pnh.getParam("close_hand_time", close_hand_time);
    pnh.getParam("pickup_arm_time", pickup_arm_time);
    // 自律走行の速度
    pnh.getParam("auto_vx", auto_vx);
    // 装填のパラメータ
    pnh.getParam("flat_range", flat_range);
    pnh.getParam("flat_threshold", flat_threshold);
    pnh.getParam("flat_angle_max", flat_angle_max);
    if (pnh.getParam("load_w", load_w)) load_w *= M_PI;
    pnh.getParam("needle_range", needle_range);
    pnh.getParam("needle_threshold", needle_threshold);
    pnh.getParam("parallel_stop_time", parallel_stop_time);
    pnh.getParam("load_dist", load_dist);
    pnh.getParam("load_vel", load_vel);
    pnh.getParam("load_arm_time", load_arm_time);
    pnh.getParam("open_hand_time", open_hand_time);

    steer.setVMax(v_max);
    steer.setWMax(w_max);
    steer.setTurnRadiusMin(TurnRadius_min);
    steer.setTurnRadiusMax(TurnRadius_max);

    target.stop = true;

    ros::Subscriber joy_sub = nh.subscribe("joy", 1, joyCb);
    ros::Subscriber front_scan_sub = nh.subscribe("front_scan", 1, frontScanCb);
    ros::Subscriber back_scan_sub = nh.subscribe("back_scan", 1, backScanCb);
    ros::Publisher target_pub = nh.advertise<msgs::FourWheelSteerRad>("target", 1);
    ros::Publisher gain_pub = nh.advertise<msgs::FourWheelSteerPIDGain>("gain", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);

    ros::Publisher arm_pub = nh.advertise<fabot_msgs::ArmMsg>("hand_state", 1);
    arm_state_msg.hand_duty = hand_duty;
    arm_state_msg.arm_duty = arm_duty;

    sleep(5);
    setGain();
    gain_pub.publish(pid_gain);
    
    ros::Rate loop_rate(freq);
    while (ros::ok()) {
        ros::spinOnce();

        if (auto_pickup_flag) {
            autoPickup();
        }

        if (auto_load_flag) {
            autoLoad();
        }

        setTarget();
        target_pub.publish(target);

        // getCoodinate();

        // marker_pub.publish(marker_array);

        arm_pub.publish(arm_state_msg);

        loop_rate.sleep();
    }
    return 0;
}
