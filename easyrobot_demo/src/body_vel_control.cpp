#include <easyrobot.hpp>

int main(int argc, char** argv)
{
    //初始化
    ros::init(argc, argv, "body_vel_control");
    ros::NodeHandle n;
    Easyrobot uav(n,"");

    //无人机起飞
    uav.takeoff(1);
    //控制无人机以BODY坐标系下X轴0.5m/s的速度飞行
    uav.gotoVelBody(0.5, 0, 0);
    //等待3秒
    sleep(6);
    //控制无人机以BODY坐标系下X轴-0.5m/s的速度飞行以及航向转向1.57弧度即90度
    uav.gotoVelBody(-0.5, 0, 0, 1.57);
    //等待3秒
    sleep(6);
    //无人机降落
    uav.land();
    return 0;
}
