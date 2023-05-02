#ifndef FourWheelSteer_h
#define FourWheelSteer_h

#include <math.h>
#include <ros/ros.h>
#define ROOT3_2 0.86602540378443864676372317075294 // √3/2
#define ROOT2 1.4142135623730950488016887242097 // √2
#define M_PI_6 0.52359877559829887307710723054658 // π/6

#define FLT_ZERO 1e-3

#define ALLOWABLE_ANGVEL_ERROR 1.3

class FourWheelSteer {
private:
    // 中心とホイールの距離(m)、前後ホイールの距離(m)、左右ホイールの距離(m)、駆動輪エンコーダ1回転辺りの進む距離(m)
    const double DistWheelCenter, DistFBWheel, DistLRWheel, DistPerEnc;
    // 左前、左後、右後、右前ホイールの目標ステア角(-M_PI_2 ~ M_PI_2)
    double Angle[4] = {0.0};
    // 左前、左後、右後、右前ホイールの目標角速度(rad/s)
    double AngVel[4] = {0.0};

    // 座標
    double x = 0.0, y = 0.0, theta = 0.0;
    // 前回の座標更新時刻
    ros::Time prev_time; // = ros::Time::now();

    // 最大移動速度(m/s)、最大角速度(rad/s)、ただし正の値
    double v_max, w_max;
    // 最小旋回半径(m)、最大旋回半径(m)
    double TurnRadius_min, TurnRadius_max;
    
    // 移動速度を補正
    double vLimitter(double &vx, double &vy);
    // x軸方向の移動速度を補正
    void vxLimitter(double &vx);
    // 角速度を補正
    void wLimitter(double &w);
    // 旋回半径を補正
    bool TurnRadiusLimitter(double &TurnRadius);
    // -M_PI_2<(目標ステア角)<M_PI_2になるように目標ステア角と目標角速度を補正
    void AngleLimitter(double &Angle, double &AngVel);
public:
    // コンストラクタ
    // DistPerEnc: 駆動輪エンコーダ1回転辺りの進む距離(mm)、DistFBWheel: 前後ホイールの距離(mm)、DistLRWheel: 左右ホイールの距離(mm)
    FourWheelSteer(double DistPerEnc, double DistFBWheel, double DistLRWheel, double v_max = 1.0, double w_max = M_PI, double TurnRadius_min = 0.8, double TurnRadius_max = 100);
    // 平行移動をするための目標ステア角と目標RPSを計算
    void parallel(double vx, double vy);
    // 回転するための目標ステア角と目標RPSを計算
    void rotate(double w);
    // x方向に自動車と同じように走行するための目標ステア角と目標RPSを計算
    void xVehicle(double vx, double w);
    // y方向に自動車と同じように走行するための目標ステア角と目標RPSを計算
    void yVehicle(double vy, double w);
    // 停止
    void stop() {
        AngVel[0] = AngVel[1] = AngVel[2] = AngVel[3] = 0.0;
    }
    // 角速度の差が大きすぎないかを判定し、異常なら停止しtrueを返す。
    bool anomalyDetect(double angVel[4], double angle[4]);
    // オドメトリを計算
    void calcOdom(double angVel[4], double angle[4]);

    // 目標ステア角を取得
    double getAngle(int i) {
        if (i >= 0 && i < 4)
            return Angle[i];
        return 0.0;
    }
    // 目標角速度を取得
    double getAngVel(int i) {
        if (i >= 0 && i < 4)
            return AngVel[i];
        return 0.0;
    }
    double getX() {
        return x;
    }
    double getY() {
        return y;
    }
    double getTheta() {
        return theta;
    }
    void getOdom(double &x, double &y, double &theta) {
        x = this->x;
        y = this->y;
        theta = this->theta;
    }
    // 最大移動速度を設定
    void setVMax(double v_max) {
        this->v_max = abs(v_max);
    }
    // 最大角速度を設定
    void setWMax(double w_max) {
        this->w_max = abs(w_max);
    }
    // 最小旋回半径を設定
    void setTurnRadiusMin(double TurnRadius_min) {
        this->TurnRadius_min = abs(TurnRadius_min);
    }
    // 最大旋回半径を設定
    void setTurnRadiusMax(double TurnRadius_max) {
        this->TurnRadius_max = abs(TurnRadius_max);
    }
};

#endif