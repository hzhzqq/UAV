#include <easyrobot.hpp>

int main(int argc, char** argv)
{
    //初始化
    ros::init(argc, argv, "enu_pos_control");
    ros::NodeHandle n;
    Easyrobot uav(n,"");

    //无人机起飞
    uav.takeoff(1);
    //发布ENU坐标系下无人机(1,0,1)位置控制指令,第一个参数选为false,将不会进行阻塞,发布控制指令后将继续执行下面的代码
    uav.gotoPosEnu(false, 1, 0, 1);
    //等待3秒,等待无人机到达目标点
    sleep(6);
    //发布ENU坐标系下无人机(0,0,1)位置控制指令,第一个参数选为true,将进行阻塞,直到无人机到达目标点或任务超时
    uav.gotoPosEnu(true, 0, 0, 1);
    //发布ENU坐标系下无人机(0,0,1)位置控制指令,并让无人机航向旋转到1.57弧度,也就是90度,第一个参数选为true,将进行阻塞,直到无人机到达目标点或任务超时
    uav.gotoPosEnu(true, 0, 0, 1, 1.57);
    //无人机降落
    uav.land();
    return 0;
}
