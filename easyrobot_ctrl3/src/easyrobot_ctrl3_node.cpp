/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


#include <offboard_control.hpp>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "easyrobot_ctrl3_node");
    ros::NodeHandle nh;

    OffboardControl offboard_control(nh);
    offboard_control.spin(argc, argv);

    // while(ros::ok()){
    //     /* 获取当前任务 1，视觉穿圆；2精准降落；3，仿线飞行；4，跟随小车，并降落在车上*/
	// 	if (action_mode == "circle")
    //         pass_2_circles();
	// 	else if (action_mode == "pland")
	// 		precesion_land_on_H();
	// 	else {
	// 		ROS_ERROR_NAMED("CTRL", "action: wrong/unexistant action_mode %s", action_mode.c_str());
	// 		return 0; 
	// 	}
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}
