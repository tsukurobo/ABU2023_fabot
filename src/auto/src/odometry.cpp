#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "FourWheelSteer.h"
#include <msgs/FourWheelSteerRad.h>

int freq = 1000;

FourWheelSteer steer(34.286, 310.0, 350.0);

// tf2_ros::StaticTransformBroadcaster static_br;

void broadcast_odom_tf(double x, double y, double theta) {
    static tf2_ros::TransformBroadcaster dynamic_br;
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = x;
    odom_tf.transform.translation.y = y;
    odom_tf.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();
    dynamic_br.sendTransform(odom_tf);
}

void radCb(const msgs::FourWheelSteerRad::ConstPtr& rad_msg) {
    double angVel[4], angle[4];
    for (int i = 0; i < 4; i++) {
        angVel[i] = rad_msg->angVel[i];
        angle[i] = rad_msg->angle[i];
    }

    steer.calcOdom(angVel, angle);
    double x, y, theta;
    steer.getOdom(x, y, theta);

    broadcast_odom_tf(x, y, theta);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.getParam("freq", freq);

    ros::Subscriber rad_sub = nh.subscribe("rad", 1, radCb);

    ros::Rate loop_rate(freq);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}