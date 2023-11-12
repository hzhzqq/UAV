#include <easyrobot.hpp>

int main(int argc, char** argv)
{
    //初始化
    ros::init(argc, argv, "body_xyvel_zpos_control");
    ros::NodeHandle n;
    Easyrobot uav(n,"");

    //无人机起飞
    uav.takeoff(1);
    //发布BODY坐标系下无人机(0,0,1)XY轴速度Z轴位置控制指令,无人机以X轴速度为0.5m/s,Z轴高度以无人机当前高度进行飞行
    uav.gotoVelxyPoszBody(0.5, 0, 0);
    //等待3秒
    sleep(6);
    //发布BODY坐标系下无人机(0,0,1)XY轴速度Z轴位置控制指令,无人机以X轴速度为0.5m/s,Z轴高度为以无人机当前高度进行飞行,无人机航向旋转到1.57弧度也就是90度
    uav.gotoVelxyPoszBody(-0.5, 0, 0, 1.57);
    sleep(6);
    //无人机降落
    uav.land();
    return 0;
}
