#include<ros/ros.h>
#include<fabot_msgs/ArmMsg.h>
#include<sensor_msgs/Joy.h>

#define ENABLE_BUTTON 5
#define CLOSE_HAND_BUTTON 0 //Xボタン
#define OPEN_HAND_BUTTON 3 //Bボタン
#define UP_ARM_BUTTON 1 //Yボタン
#define DOWN_ARM_BUTTON 2 //Aボタン
#define HAND_DUTY 470 //Arduinoに指定するDuty
#define ARM_DUTY 470

fabot_msgs::ArmMsg arm_state_msg;

//ジョイコンのコールバック関数
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (joy->buttons[ENABLE_BUTTON]) {
        //両方押してるときは手は停止
        if(joy->buttons[OPEN_HAND_BUTTON]==1 && joy->buttons[CLOSE_HAND_BUTTON]==0){
            arm_state_msg.hand = 1;
        }else if(joy->buttons[CLOSE_HAND_BUTTON]==1 && joy->buttons[OPEN_HAND_BUTTON]==0){
            arm_state_msg.hand = 2;
        }else{
            arm_state_msg.hand = 0;
        }

        //両方押してるときは腕は停止
        if(joy->buttons[UP_ARM_BUTTON]==1 && joy->buttons[DOWN_ARM_BUTTON]==0){
            arm_state_msg.arm = 1;
        }else if(joy->buttons[DOWN_ARM_BUTTON]==1 && joy->buttons[UP_ARM_BUTTON]==0){
            arm_state_msg.arm = 2;
        }else{
            arm_state_msg.arm = 0;
        }
    }
    else {
        arm_state_msg.hand = 0;
        arm_state_msg.arm = 0;
    }
}

int main(int argc, char **argv)
{
    //ROSを初期化
    ros::init(argc, argv, "joy");

    //ROSノードハンドルを作成
    ros::NodeHandle nh;

    //ジョイコンのトピックを購読
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
    
    //Arduinoにメッセージを送信
    ros::Publisher pub = nh.advertise<fabot_msgs::ArmMsg>("hand_state", 1);
    
    //Dutyを設定
    arm_state_msg.hand_duty = HAND_DUTY;
    arm_state_msg.arm_duty = ARM_DUTY;

    //ROSのメインループを開始
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        
        // Arduinoにメッセージを送信
        pub.publish(arm_state_msg);

        loop_rate.sleep();
    }

    return 0;
}
