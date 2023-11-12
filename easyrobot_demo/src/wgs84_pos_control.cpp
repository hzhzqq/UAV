#include <easyrobot.hpp>

int main(int argc, char** argv)
{
    //初始化
    ros::init(argc, argv, "wgs84_pos_control");
    ros::NodeHandle n;
    Easyrobot uav(n,"");

    //无人机起飞
    uav.takeoff();
    //获取无人机当前状态数据，获取无人机当前经纬度
    easyrobot_msgs::RobotState uav_state;
    uav_state = uav.getRobotState(false);
    //发布WGS84坐标系下无人机当前经纬度高度为3米的控制指令,第一个参数选为false,将不会进行阻塞,发布控制指令后将继续执行下面的代码
    uav.gotoPosGPS(false, uav_state.latitude, uav_state.longitude, 3);
    //等待3秒,等待无人机到达目标点
    sleep(3);
    //发布WGS84坐标系下无人机当前经纬度高度为5米的控制指令,第一个参数选为true,将进行阻塞,直到无人机到达目标点或任务超时
    uav.gotoPosGPS(true, uav_state.latitude, uav_state.longitude, 5);
    //发布WGS84坐标系下无人机当前经纬度高度为3米的控制指令,并让无人机航向旋转到1.57弧度,也就是90度,第一个参数选为true,将进行阻塞,直到无人机到达目标点或任务超时
    uav.gotoPosGPS(true, uav_state.latitude, uav_state.longitude, 3, 1.57);
    //无人机降落
    uav.land();
    return 0;
}