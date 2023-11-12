#include <offboard_control.hpp>
#include <algorithm>
#include <cmath>
#include <ros/console.h>
// #include "mavlink/common/common.h"

using namespace std;
// using mavlink::common::POSITION_TARGET_TYPEMASK;

void 
OffboardControl::init() {
    
    if(_init_done) return;

    /**
        * 设置运行频率和是否使用pid进行速度控制
        */
    nh_sp.param("rate", rate, 100.0);
    nh_sp.param("use_pid", use_pid, false);


    if (use_pid) {
        /**
            * @note some of these are based on values defaulted @ https://bitbucket.org/enddl22/ardrone_side_project/
            * tweaks to them so to get a better velocity response are welcomed!
            */

        // Linear velocity PID gains and bound of integral windup
        nh_sp.param("linvel_p_gain", linvel_p_gain, 0.4);
        nh_sp.param("linvel_i_gain", linvel_i_gain, 0.05);
        nh_sp.param("linvel_d_gain", linvel_d_gain, 0.12);
        nh_sp.param("linvel_i_max", linvel_i_max, 0.1);
        nh_sp.param("linvel_i_min", linvel_i_min, -0.1);

        // Yaw rate PID gains and bound of integral windup
        nh_sp.param("yawrate_p_gain", yawrate_p_gain, 0.011);
        nh_sp.param("yawrate_i_gain", yawrate_i_gain, 0.00058);
        nh_sp.param("yawrate_d_gain", yawrate_d_gain, 0.12);
        nh_sp.param("yawrate_i_max", yawrate_i_max, 0.005);
        nh_sp.param("yawrate_i_min", yawrate_i_min, -0.005);

        // Setup of the PID controllers
        pid.setup_linvel_pid(linvel_p_gain, linvel_i_gain, linvel_d_gain, linvel_i_max, linvel_i_min, nh_sp);
        pid.setup_yawrate_pid(yawrate_p_gain, yawrate_i_gain, yawrate_d_gain, yawrate_i_max, yawrate_i_min, nh_sp);
    }

    /**
        * @brief Setpoint control mode selector
        *
        * Available modes:
        * - position
        * - velocity
        * - acceleration
        */
    std::string init_mode;
    nh_sp.param<std::string>("mode", init_mode, "position");

    if (init_mode == "position")
        mode = POSITION;
    else if (init_mode == "velocity")
        mode = VELOCITY;
    else if (init_mode == "acceleration")
        mode = ACCELERATION;
    else {
        ROS_ERROR_NAMED("sitl_test", "Control mode: wrong/unexistant control mode name %s", init_mode.c_str());
        return;
    }


    /**
        * @brief action_mode selector
        *
        * Available action_mode:
        *  circle_pass
        *  precise_land
        *  follow_line
        *  follow_vehicle
        */
    std::string init_action_mode;
    nh_sp.param<std::string>("action", init_action_mode, "circle_pass");

    if (init_action_mode == "circle_pass")
        action_mode = 	CIRCLE_PASS;
    else if (init_action_mode == "precise_land")
        action_mode = PRECISE_LAND;
    else {
        ROS_ERROR_NAMED("offboard_control", "Path action_mode: wrong/unexistant path action_mode name %s", init_action_mode.c_str());
        return;
    }
    _init_done = true;
}


void 
OffboardControl::do_offboard_takeoff(){
    ros::Rate loop_rate(rate);
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.8;

    //send a few setpoints before starting
    if(!finish_offboard_takeoff){
        for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_sp_pub.publish(pose);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok() && !finish_offboard_takeoff){

        /* 将模式设置为offboard,并解锁无人机 */
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0))){
            if( set_mode_client.call(offb_set_mode) &&  offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        /* 起飞到2m高度 */
        local_pos_sp_pub.publish(pose);

        Eigen::Vector3d dest;
        tf::pointMsgToEigen(pose.pose.position, dest);
        tf::pointMsgToEigen(localpos.pose.position, current);

        double distance = sqrt((dest - current).x() * (dest - current).x() +
                (dest - current).y() * (dest - current).y() +
                (dest - current).z() * (dest - current).z());

        if (distance <= 0.1)
            finish_offboard_takeoff = true;
        // ROS_INFO_STREAM_THROTTLE( 2, "INFO throttle message." );    
        // ROS_INFO("takeoff, dest - current distance %.2f , %.2f , %.2f , %.2f ", distance, dest.x(),dest.y(),dest.z() ); 
        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("SITL Test: Offboard control takeoff to (0,0,2).");

    return;
}



void 
OffboardControl::do_land(){
    ros::Rate loop_rate(rate);
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::Time last_request = ros::Time::now();

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "LAND";

    while(ros::ok() && !_finished_land ){

        if( current_state.mode != "LAND" && (ros::Time::now() - last_request > ros::Duration(3.0))){
            if( set_mode_client.call(offb_set_mode) &&  offb_set_mode.response.mode_sent){
                ROS_INFO("LAND enabled");
                _finished_land = true;
            }
            last_request = ros::Time::now();
        }

        loop_rate.sleep();
        if(!_finished_land) //结束后就不spin了
            ros::spinOnce();
    }

    return;
}


/* -*- main routine -*- */
void 
OffboardControl::spin(int argc, char *argv[]) {

    init(); // 初始化
    ros::Rate loop_rate(rate);

    // 进入offboard模式并解锁,起飞到设定的(0,0,2)位置
    //do_offboard_takeoff();

    if (mode == POSITION) {
        ROS_INFO("Position control mode selected.");
    }
    else if (mode == VELOCITY) {
        ROS_INFO("Velocity control mode selected.");
    }
    else if (mode == ACCELERATION) {
        ROS_INFO("Acceleration control mode selected.");
        ROS_ERROR_NAMED("sitl_test", "Control mode: acceleration control mode not supported in PX4 current Firmware.");
        /**
            * @todo: lacks firmware support, for now
            */
        return;
    }

    if (action_mode == CIRCLE_PASS) {
        ROS_INFO("Test option: CIRCLE_PASS path...");
        // circle_pass_path_motion(loop_rate, mode);
        one_circle_pass_path_motion(loop_rate, mode);
    }
    else if (action_mode == PRECISE_LAND) {
        ROS_INFO("Test option: PRECISE_LAND path...");
        precise_land_path_motion(loop_rate, mode);
    }
}

void 
OffboardControl::precise_land_path_motion(ros::Rate loop_rate, control_mode mode){
    ROS_INFO("precise_land_path_motion...");
    // double last_time = ros::Time::now().toSec();
    ros::Time last_time = ros::Time::now();
    while (ros::ok() && !_finished_land) {

        // if(ros::Time::now() - last_time > ros::Duration(wait_seconds) ) // TODO

        // 如果高度大于2m,下降速度为1m/s，如果高度小于2m,下降速度为0.5m/s。
        // 用经过的时间和速度需求来更新位置指令，此种方式优先
        // 如果APM支持TYPEMASK_Z_IGNORE，也可以用垂向速度指令
        // 水平上给位置指令
        // 当高度低于2m后，水平位置指令不再变化

        /* 等待2s,获取目标位置数据 circle_targets_1 ,得出_tar_pos1_when_wait */
        wait_on_location(2.0); 

        // 3. 根据圆心位置，修改目标位置为当前位置到圆心位置形成的射线，比圆心位置处延长一倍的位置。
        Vector3d targets;

        tf::pointMsgToEigen(_tar_pos1_when_wait.pose.position, targets);
        targets = longer_away_pos(targets );
        ROS_INFO("pass circle, go to targets %.2f, %.2f, %.2f ", targets.x(), targets.y(), targets.z());

        // Vector3d targets = longer_away_pos(circle_center_enu);

        /* 4. 把目标位置作为期望发送给无人机； */
        // void 	pointEigenToMsg (const Eigen::Vector3d &e, geometry_msgs::Point &m)
        circle_pass_stage = TAKEOFF_DONE;
        tf::pointEigenToMsg(targets, ps.pose.position);

        wait_and_move(ps);

        /* 5. 无人机到达位置后，悬停，发送降落指令； */
        circle_pass_stage = SECONDE_O_DONE;
        do_land();
        ROS_INFO("one circle pass DONE!");

        last_time = ros::Time::now();
        loop_rate.sleep();
        ros::spinOnce();        

    }
}


void 
OffboardControl::circle_pass_path_motion(ros::Rate loop_rate, control_mode mode){
    ROS_INFO("circle_pass_path_motion...");
    // double last_time = ros::Time::now().toSec();
    ros::Time last_time = ros::Time::now();
    while (ros::ok()) {

            
        // if(ros::Time::now() - last_time > ros::Duration(wait_seconds) ) // TODO


        /* 等待2s,获取目标位置数据 circle_targets_1 和 circle_targets_2 */
        wait_on_location(2.0);

        /* 2. 接收视觉判断的数据，计算出两个圆圈的local坐标系位置； */
        // circle_targets_cb 函数里做了

        if(_real_targets_num == 1){
            // 只有一个圆圈，就飞过圆圈1m后降落
            /* 3. 把较近一个圆圈的位置（x,y,z）和把xyz指向下一个圆圈的的yaw角作为期望发送给无人机； */

            circle_pass_stage = TAKEOFF_DONE;
            // wait_and_move(circle_targets_1);

        }else{
            /* 4. 无人机到达位置后，悬停2s； */
            circle_pass_stage = FIRST_O_DONE;
            // wait_on_location();

            /*   1. 2s内能得到下一个圆圈的位置信息的话，把当前位置指向圆圈中心的延长线1m后的位置作为位置指令发给无人机；*/
            /*   2. 如果得不到下一个圆圈的位置信息，把第2步中的第二个圆圈的位置作为2号圈的位置使用，把当前位置指向圆圈中心的延长线1m后的位置作为位置指令发给无人机； */
            // circle_pass_stage
            // wait_and_move(circle_targets_2);

        }
        

        /* 5. 无人机到达位置后，悬停，发送降落指令； */
        circle_pass_stage = SECONDE_O_DONE;

        last_time = ros::Time::now();
        loop_rate.sleep();
        ros::spinOnce();        

    }

}


void 
OffboardControl::one_circle_pass_path_motion(ros::Rate loop_rate, control_mode mode){
    ROS_INFO("one circle_pass_path_motion...");
    // double last_time = ros::Time::now().toSec();
    ros::Time last_time = ros::Time::now();
    while (ros::ok() && !_finished_land) {

        // if(ros::Time::now() - last_time > ros::Duration(wait_seconds) ) // TODO


        /* 等待2s,获取目标位置数据 circle_targets_1 ,得出_tar_pos1_when_wait */
        wait_on_location(2.0); 
        Vector3d targets;

        if(circle_targets_1.pz > 2.0){ //深度距离超过2m，飞到一个近的点，再悬停。
            tf::pointMsgToEigen(_tar_pos1_when_wait.pose.position, targets);
            targets = goto_close_pos(targets);
            tf::pointEigenToMsg(targets, ps.pose.position);
            wait_and_move(ps);
            wait_on_location(2.0); 

        }

        // 3. 根据圆心位置，修改目标位置为当前位置到圆心位置形成的射线，比圆心位置处延长一倍的位置。

        tf::pointMsgToEigen(_tar_pos1_when_wait.pose.position, targets);
        targets = longer_away_pos(targets );
        ROS_INFO("pass circle, go to targets %.2f, %.2f, %.2f ", targets.x(), targets.y(), targets.z());

        // Vector3d targets = longer_away_pos(circle_center_enu);

        /* 4. 把目标位置作为期望发送给无人机； */
        // void 	pointEigenToMsg (const Eigen::Vector3d &e, geometry_msgs::Point &m)
        circle_pass_stage = TAKEOFF_DONE;
        tf::pointEigenToMsg(targets, ps.pose.position);

        wait_and_move(ps,true);

        /* 5. 无人机到达位置后，悬停，发送降落指令； */
        circle_pass_stage = SECONDE_O_DONE;
        do_land();
        ROS_INFO("one circle pass DONE!");

        last_time = ros::Time::now();
        loop_rate.sleep();
        ros::spinOnce();        

    }

}


// 从当前位置再远一倍的位置
// 从当前位置再远dist米的位置
Vector3d
OffboardControl::longer_away_pos(Vector3d pos){
// OffboardControl::longer_away_pos(Vector3d pos, double dist){
    // current
    Vector3d tar_pos;
    tf::pointMsgToEigen(localpos.pose.position, current);
    tar_pos(0)=2.5*pos(0) - current(0);
    tar_pos(1)=2.5*pos(1) - current(1);
    tar_pos(2)=2.0*pos(2) - current(2);
    return tar_pos;
}

// 从当前位置到目标点的直线上取一点，此点到目标点的距离是2.5m.
Vector3d
OffboardControl::goto_close_pos(Vector3d pos,float dist){
// OffboardControl::longer_away_pos(Vector3d pos, double dist){
    // current
    Vector3d tar_pos;
    tf::pointMsgToEigen(localpos.pose.position, current);
    float distance = sqrt(pow(current(0)-pos(0),2.)+
                          pow(current(1)-pos(1),2.)+
                          pow(current(2)-pos(2),2.));
    if(dist < 2.0)
        return current;
    if(distance < dist ){
        return current;
    }else{
        return pos * (1- dist/distance)  + current * dist/distance;

    }


    return tar_pos;
}



/**
 * @brief 把相机坐标系中的位置转换为局部坐标系下enu的位置
 * 
 * @param vehicle_local_pos 当前无人机在enu坐标系下的位置
 * @param cam_pos 相机坐标系下的目标位置
 * @return Vector3d enu坐标系下的目标位置
 */
Vector3d
OffboardControl::cam_pos_trans_enu(Vector3d vehicle_local_pos, Vector3d cam_pos){
    Vector3d tar_pos_temp=cam_pos;
    if(0){
        Eigen::Vector3d ea( -0,to_radians(-_yaw_deg.data), 0);

        //3.1 欧拉角转换为旋转矩阵 绕y轴逆时针旋转 _yaw_deg 
        Eigen::Matrix3d rotation_matrix3;
        rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                        Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                        Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
        
        tar_pos_temp = rotation_matrix3 * cam_pos;

        // 绕x轴顺时针旋转90度，到东北天坐标系
        ea(0) = 0.0;
        ea(1) = 0.0;
        ea(2) = M_PI/2;
        rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                        Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
                        Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
        tar_pos_temp = rotation_matrix3 * tar_pos_temp;
        
    }else{

        // double yaw_rad = 1.9;
        double yaw_rad = to_radians(_yaw_deg.data);
        tar_pos_temp(0) =   cos(yaw_rad) * cam_pos(0) + sin(yaw_rad) * cam_pos(2);
        tar_pos_temp(2) = - sin(yaw_rad) * cam_pos(0) + cos(yaw_rad) * cam_pos(2);
        // ROS_INFO("guocheng1 %.2f, %.2f, %.2f ", tar_pos_temp(0), tar_pos_temp(1), tar_pos_temp(2));

        Vector3d tar_pos_temp2=tar_pos_temp ;
        // ROS_INFO("guocheng1 %.2f, %.2f, %.2f ", tar_pos_temp2(0), tar_pos_temp2(1), tar_pos_temp2(2));
        tar_pos_temp2(1) =   cos(M_PI_2) * tar_pos_temp(1) + sin(M_PI_2) * tar_pos_temp(2);
        tar_pos_temp2(2) = - sin(M_PI_2) * tar_pos_temp(1) + cos(M_PI_2) * tar_pos_temp(2);

        // ROS_INFO("guocheng2 %.2f, %.2f, %.2f ", tar_pos_temp2(0), tar_pos_temp2(1), tar_pos_temp2(2));

        tar_pos_temp = tar_pos_temp2;
        // ROS_INFO("guocheng3 %.2f, %.2f, %.2f ", tar_pos_temp(0), tar_pos_temp(1), tar_pos_temp(2));
    }    
    tar_pos_temp += vehicle_local_pos;
        
    return tar_pos_temp;
}



/**
    * @brief 原地悬停等待几秒
    */
void 
OffboardControl::wait_on_location(double wait_seconds){
    ros::Rate loop_rate(rate);
    ros::Time last_time = ros::Time::now();
    bool stop = false;
    ROS_INFO("circle_PASS wait_on_location %.2fs",wait_seconds);

    Eigen::Vector3d dest;
    // Eigen::Vector3d wait_location_v3d;
    geometry_msgs::PoseStamped wait_location;

    // 记录当前位置
    // tf::pointMsgToEigen(localpos.pose.position, wait_location_v3d);
    wait_location.pose.position = localpos.pose.position;
    // tf::pointEigenToMsg(Eigen::Vector3d(0.0f, 0.0f, 1.0f), ps.pose.position);
    static int cnti;
        // Eigen::Vector3d test_input1(0.0,0.0,0.0);
        // Eigen::Vector3d test_input2(0.08,-0.23,1.54);   
        // Eigen::Vector3d test_output; 
    // while (ros::ok() ) {
    while (ros::ok() && !stop) {
        if(ros::Time::now() - last_time > ros::Duration(wait_seconds) ){ // TODO
            stop = true;
        }
        local_pos_sp_pub.publish(wait_location);
        tf::pointMsgToEigen(localpos.pose.position, current);
        Vector3d circle_center_enu = cam_pos_trans_enu(current, 
            Vector3d(circle_targets_1.px, circle_targets_1.py, circle_targets_1.pz) );
        _tar_pos1_when_wait.pose.position.x = circle_center_enu(0);
        _tar_pos1_when_wait.pose.position.y = circle_center_enu(1);
        _tar_pos1_when_wait.pose.position.z = circle_center_enu(2);
        cnti++;
        if(cnti % 30 ==0)
            ROS_INFO("circle_center_enu: %.2f, %.2f, %.2f ", \
                circle_center_enu(0), circle_center_enu(1),circle_center_enu(2));


        // test_output = cam_pos_trans_enu(test_input1, test_input2 );
        // if(cnti % 31 ==0)
        //     ROS_INFO("enu test: %.2f, %.2f, %.2f ", \
        //         test_output(0), test_output(1),test_output(2));

        loop_rate.sleep();
        ros::spinOnce();
    }
}


bool 
OffboardControl::_compute_heading_from_2D_vector(float &heading, Vector2f v)
{
    float length = sqrtf(v(0) * v(0) + v(1) * v(1));
	if (isfinite(length) && length > SIGMA_NORM) {
		v.normalize();
		// To find yaw: take dot product of x = (1,0) and v
		// and multiply by the sign given of cross product of x and v.
		// Dot product: (x(0)*v(0)+(x(1)*v(1)) = v(0)
		// Cross product: x(0)*v(1) - v(0)*x(1) = v(1)
		heading =  mathpi::sign(v(1)) * mathpi::wrap_pi(acosf(v(0)));
		return true;
	}

	// heading unknown and therefore do not change heading
	return false;
}

/**
    * @brief Defines the accepted threshold to the destination/target position
    * before moving to the next setpoint.
    */
void 
OffboardControl::wait_and_move(geometry_msgs::PoseStamped target, bool set_yaw_flag){
    ros::Rate loop_rate(rate);
    ros::Time last_time = ros::Time::now();
    bool stop = false;
    
    Eigen::Vector3d dest;

    double distance;
    double err_th = threshold[rand() % threshold.size()];
    err_th = 0.5;

    mavros_msgs::PositionTarget local_pos_sp_raw;

    local_pos_sp_raw.type_mask = 0;
    local_pos_sp_raw.type_mask = \
                POSITION_TARGET_TYPEMASK_VX_IGNORE	|    \
                POSITION_TARGET_TYPEMASK_VY_IGNORE	|    \
                POSITION_TARGET_TYPEMASK_VZ_IGNORE	|    \
                POSITION_TARGET_TYPEMASK_AX_IGNORE	|    \
                POSITION_TARGET_TYPEMASK_AY_IGNORE	|    \
                POSITION_TARGET_TYPEMASK_AZ_IGNORE	|    \
                POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE ;
    
    local_pos_sp_raw.position.x = target.pose.position.x;
    local_pos_sp_raw.position.y = target.pose.position.y;
    local_pos_sp_raw.position.z = target.pose.position.z;
    local_pos_sp_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    _yaw_lock = false;

    // ROS_DEBUG("Next setpoint: accepted error threshold: %1.3f", err_th);
    // ROS_INFO("Next setpoint: accepted error threshold: %1.3f", err_th);

    while (ros::ok() && !stop) {
        tf::pointMsgToEigen(target.pose.position, dest);
        tf::pointMsgToEigen(localpos.pose.position, current);

        distance = sqrt((dest - current).x() * (dest - current).x() +
                (dest - current).y() * (dest - current).y() +
                (dest - current).z() * (dest - current).z());

        if (distance <= err_th)
            stop = true;

        Vector2f v = {(dest(0) - current(0)),(dest(1) - current(1))}; // Vector that points towards desired location
        // v = Vector2f(dest) - Vector2f(current);
        // 初始_yaw_lock为false，进去后由于距离较远，先用else中函数得出_yaw_setpoint，离得近了就_yaw_lock，下次就不走这一段了。
        if (!_yaw_lock) {
            if ( sqrtf(v(0) * v(0) + v(1) * v(1)) < 0.5) {
                _yaw_setpoint = _yaw_deg.data;
                _yaw_lock = true;

            } else {
                _compute_heading_from_2D_vector(_yaw_setpoint, v);
                // _yaw_lock = false;
            }
        }

        local_pos_sp_raw.yaw = _yaw_setpoint;


        if (mode == POSITION) {
            // /mavros/setpoint_raw/local
            target.header.stamp = ros::Time::now();
            target.header.seq++; 
            if(set_yaw_flag){
                local_pos_sp_raw_pub.publish(local_pos_sp_raw);
            }else{
                local_pos_sp_pub.publish(target);
            }
        }
        else if (mode == VELOCITY) {
            if (use_pid)
                tf::vectorEigenToMsg(pid.compute_linvel_effort(dest, current, last_time), vs.twist.linear);
            else
                tf::vectorEigenToMsg(dest - current, vs.twist.linear);
            vel_sp_pub.publish(vs);
        }
        else if (mode == ACCELERATION) {
            // TODO
            return;
        }

        // target.pose.position.x = 0.0;
        // target.pose.position.y = 1;
        // target.pose.position.z = 2;
        // local_pos_sp_pub.publish(target);


        last_time = ros::Time::now();
        loop_rate.sleep();
        ros::spinOnce();
    }
}




/* -*- callbacks -*- */

void 
OffboardControl::local_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg){
    localpos = *msg;
    static int one_cnt;
    one_cnt++;
    if(one_cnt % 30 == 0)
    ROS_INFO("local pos %.2f, %.2f, %.2f!", \
        localpos.pose.position.x, localpos.pose.position.y, localpos.pose.position.z);

}

void 
OffboardControl::LocalVelocityCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
    local_velocity = *msg;
}

void 
OffboardControl::yaw_deg_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
    tf2::Quaternion quat;
    tf2::fromMsg(orientation, quat);

    // 将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // 将弧度转换为度，并存储在 _yaw_deg 中
    _yaw_deg.data = (yaw * 180.0 / M_PI);


    if(_yaw_deg.data < 0){
        _yaw_deg.data += 360; 
    }
    //ROS_INFO("local yaw %.2f", _yaw_deg.data);
    
    
    
    
    //add yaw_msg
   // static int cnti;
       // cnti++:
       // if(cnti % 50 ==0)
           //ROS_INFO("local yaw %.2f", _yaw_deg.data);
           //ROS_INFO("local yaw %.2f", yaw_compass);
    
    
}




    //_yaw_deg = *msg;
    // ROS_INFO("local yaw %.2f", _yaw_deg.data);
/*
环境中只有一个圆环，识别一个圆环的callback
*/
void 
OffboardControl::one_circle_targets_cb(const spirecv_msgs::TargetsInFrameConstPtr& msg){
    circle_targets = *msg;
    const int targets_num = circle_targets.targets.size(); // TODO 怎么计数的？size对吗
    // circle_targets_1
    if(targets_num == 0) {
        _real_targets_num = 0;
        return;
    }

    double sum_xyz[3]={0.0};
    for(int j=0;j<targets_num;j++){ // 0~cnt 取平均
        sum_xyz[0] += circle_targets.targets[j].px;
        sum_xyz[1] += circle_targets.targets[j].py;
        sum_xyz[2] += circle_targets.targets[j].pz;
    }
    circle_targets_1 = circle_targets.targets[0];
    circle_targets_1.px = sum_xyz[0]/targets_num;
    circle_targets_1.py = sum_xyz[1]/targets_num;
    circle_targets_1.pz = sum_xyz[2]/targets_num;        
    static int onec_cnt;
    onec_cnt++;
    if(onec_cnt % 60 == 0)
        ROS_INFO("circle_targets_1 %.2f , %.2f , %.2f ", circle_targets_1.px,circle_targets_1.py,circle_targets_1.pz );

}


/*
环境中有2个圆环，识别2个圆环的callback，没验证，TODO
*/
void 
OffboardControl::two_circle_targets_cb(const spirecv_msgs::TargetsInFrameConstPtr& msg){
    circle_targets = *msg;
    const int targets_num = circle_targets.targets.size(); // TODO 怎么计数的？size对吗
    // circle_targets_1
    if(targets_num == 0) {
        _real_targets_num = 0;
        return;
    }

    /* 利用深度信息来区分所识别的两个圆圈 */
    double origin_depths[targets_num],depths[targets_num];
    
    for(int i=0;i<targets_num;i++){
        origin_depths[i] = circle_targets.targets[i].pz;
        depths[i] = circle_targets.targets[i].pz;
    }
    sort(depths,depths+targets_num); // 排序

    int map_index[targets_num];

    /*depths的第i个，映射到origin_depths的第j个*/
    for(int i=0;i<targets_num;i++){
        for(int j=0;j<targets_num;j++){
            if ( fabsf(origin_depths[j] - depths[i] ) < 1e-5 )
                map_index[i] = j;
        }
    }    
    /**
     * @brief 例如
     * 原始   5     0.1  0.2     0.3
     * 排序后 0.1   0.2   0.3    5
     * 则map_index=(1, 2, 3, 0)
     */


    int cnt=0,break_cnt=0; //0~i代表同一个圆圈
    for(cnt=0;cnt<targets_num-1;cnt++){
        if(fabsf(origin_depths[map_index[cnt+1]] - origin_depths[map_index[cnt]] ) > 2.5){
            break_cnt++; //不多于2个圆圈
        }
    }

    if(cnt == targets_num-1){ // 数到最后也没有差异明显的值
        _real_targets_num = 1;
    }else if(break_cnt ){    // 有个分界线
        _real_targets_num = 2;
    }


    if(_real_targets_num == 2 ){

        for(cnt=0;cnt<targets_num-1;cnt++){
            if(fabsf(origin_depths[map_index[cnt+1]] - origin_depths[map_index[cnt]] ) > 2.5){
                break; // 此时 0 ~ cnt 是第一个目标，其余是第二个目标
            }
        }

        double sum_xyz[3]={0.0};
        for(int j=0;j<cnt+1;j++){ // 0~cnt 取平均
            sum_xyz[0] += circle_targets.targets[j].px;
            sum_xyz[1] += circle_targets.targets[j].py;
            sum_xyz[2] += circle_targets.targets[j].pz;
        }
        circle_targets_1 = circle_targets.targets[map_index[0]];
        circle_targets_1.px = sum_xyz[0]/(cnt+1);
        circle_targets_1.py = sum_xyz[1]/(cnt+1);
        circle_targets_1.pz = sum_xyz[2]/(cnt+1);

        sum_xyz[0] = sum_xyz[1] = sum_xyz[2] = 0.0;
        for(int j=cnt+1;j<targets_num;j++){ // cnt+1 到 targets_num-1 取平均
            sum_xyz[0] += circle_targets.targets[j].px;
            sum_xyz[1] += circle_targets.targets[j].py;
            sum_xyz[2] += circle_targets.targets[j].pz;
        }
        circle_targets_2 = circle_targets.targets[map_index[cnt+1]];
        circle_targets_2.px = sum_xyz[0]/(targets_num-1 - (cnt+1) + 1);
        circle_targets_2.py = sum_xyz[1]/(targets_num-1 - (cnt+1) + 1);
        circle_targets_2.pz = sum_xyz[2]/(targets_num-1 - (cnt+1) + 1);

    }else if(_real_targets_num == 1){
        circle_targets_1 = circle_targets.targets[0];

    }

    distinguish_2_circles_pos();
}


void 
OffboardControl::distinguish_2_circles_pos(){
    /* 得出circle_targets_1 和 circle_targets_2*/ // circle_pass_path_motion 根据当前的stage

}

void 
OffboardControl::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


/**
 * @brief Square path motion routine
 */
void 
OffboardControl::square_path_motion(ros::Rate loop_rate, control_mode mode){
    uint8_t pos_target = 1;

    ROS_INFO("Testing...");

    while (ros::ok()) {
        wait_and_move(ps);

        // motion routine
        switch (pos_target) {
        case 1:
            tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
            break;
        case 2:
            tf::pointEigenToMsg(pos_setpoint(-1, 1, 1), ps.pose.position);
            break;
        case 3:
            tf::pointEigenToMsg(pos_setpoint(-1, -1, 1), ps.pose.position);
            break;
        case 4:
            tf::pointEigenToMsg(pos_setpoint(1, -1, 1), ps.pose.position);
            break;
        case 5:
            tf::pointEigenToMsg(pos_setpoint(1, 1, 1), ps.pose.position);
            break;
        default:
            break;
        }

        if (pos_target == 6) {
            ROS_INFO("Test complete!");
            ros::shutdown();
        }
        else
            ++pos_target;

        loop_rate.sleep();
        ros::spinOnce();
    }
}




/**
 * @brief Circle path motion routine
 */
void 
OffboardControl::circle_path_motion(ros::Rate loop_rate, control_mode mode){
    ROS_INFO("Testing...");
    ros::Time last_time = ros::Time::now();

    while (ros::ok()) {
        tf::pointMsgToEigen(localpos.pose.position, current);

        // starting point
        if (mode == POSITION) {
            tf::pointEigenToMsg(Eigen::Vector3d(5.0f, 0.0f, 1.0f), ps.pose.position);
            local_pos_sp_pub.publish(ps);
        }
        else if (mode == VELOCITY) {
            if (use_pid)
                tf::vectorEigenToMsg(pid.compute_linvel_effort(
                            Eigen::Vector3d(5.0f, 0.0f, 1.0f), current, last_time), vs.twist.linear);
            else
                tf::vectorEigenToMsg(Eigen::Vector3d(5.0f - current.x(), -current.y(), 1.0f - current.z()), vs.twist.linear);
            vel_sp_pub.publish(vs);
        }
        else if (mode == ACCELERATION) {
            // TODO
            return;
        }

        wait_and_move(ps);

        // motion routine
        for (int theta = 0; theta <= 360; theta++) {
            tf::pointMsgToEigen(localpos.pose.position, current);

            if (mode == POSITION) {
                tf::pointEigenToMsg(circle_shape(theta), ps.pose.position);
                local_pos_sp_pub.publish(ps);
            }
            else if (mode == VELOCITY) {
                if (use_pid)
                    tf::vectorEigenToMsg(pid.compute_linvel_effort(circle_shape(theta), current, last_time), vs.twist.linear);
                else
                    tf::vectorEigenToMsg(circle_shape(theta) - current, vs.twist.linear);
                vel_sp_pub.publish(vs);
            }
            else if (mode == ACCELERATION) {
                // TODO
                return;
            }
            if (theta == 360) {
                ROS_INFO("Test complete!");
                ros::shutdown();
            }
            last_time = ros::Time::now();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
}

/**
 * @brief Eight path motion routine
 */
void 
OffboardControl::eight_path_motion(ros::Rate loop_rate, control_mode mode){
    ROS_INFO("Testing...");
    ros::Time last_time = ros::Time::now();

    while (ros::ok()) {
        tf::pointMsgToEigen(localpos.pose.position, current);

        // starting point
        if (mode == POSITION) {
            tf::pointEigenToMsg(Eigen::Vector3d(0.0f, 0.0f, 1.0f), ps.pose.position);
            local_pos_sp_pub.publish(ps);
        }
        else if (mode == VELOCITY) {
            if (use_pid)
                tf::vectorEigenToMsg(pid.compute_linvel_effort(
                            Eigen::Vector3d(0.0f, 0.0f, 1.0f), current, last_time), vs.twist.linear);
            else
                tf::vectorEigenToMsg(Eigen::Vector3d(-current.x(), -current.y(), 1.0f - current.z()), vs.twist.linear);
            vel_sp_pub.publish(vs);
        }
        else if (mode == ACCELERATION) {
            // TODO
            return;
        }

        wait_and_move(ps);

        // motion routine
        for (int theta = -180; theta <= 180; theta++) {
            tf::pointMsgToEigen(localpos.pose.position, current);

            if (mode == POSITION) {
                tf::pointEigenToMsg(eight_shape(theta), ps.pose.position);
                local_pos_sp_pub.publish(ps);
            }
            else if (mode == VELOCITY) {
                if (use_pid)
                    tf::vectorEigenToMsg(pid.compute_linvel_effort(eight_shape(theta), current, last_time), vs.twist.linear);
                else
                    tf::vectorEigenToMsg(eight_shape(theta) - current, vs.twist.linear);
                vel_sp_pub.publish(vs);
            }
            else if (mode == ACCELERATION) {
                // TODO
                return;
            }
            if (theta == 180) {
                ROS_INFO("Test complete!");
                ros::shutdown();
            }
            last_time = ros::Time::now();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
}

/**
 * @brief Ellipse path motion routine
 */
void 
OffboardControl::ellipse_path_motion(ros::Rate loop_rate, control_mode mode){
    ROS_INFO("Testing...");
    ros::Time last_time = ros::Time::now();

    while (ros::ok()) {
        tf::pointMsgToEigen(localpos.pose.position, current);

        // starting point
        if (mode == POSITION) {
            tf::pointEigenToMsg(Eigen::Vector3d(0.0f, 0.0f, 2.5f), ps.pose.position);
            local_pos_sp_pub.publish(ps);
        }
        else if (mode == VELOCITY) {
            if (use_pid)
                tf::vectorEigenToMsg(pid.compute_linvel_effort(
                            Eigen::Vector3d(0.0f, 0.0f, 2.5f), current, last_time), vs.twist.linear);
            else
                tf::vectorEigenToMsg(Eigen::Vector3d(-current.x(), -current.y(), 2.5f - current.z()), vs.twist.linear);
            vel_sp_pub.publish(vs);
        }
        else if (mode == ACCELERATION) {
            // TODO
            return;
        }

        wait_and_move(ps);

        // motion routine
        for (int theta = 0; theta <= 360; theta++) {
            tf::pointMsgToEigen(localpos.pose.position, current);

            if (mode == POSITION) {
                tf::pointEigenToMsg(ellipse_shape(theta), ps.pose.position);
                local_pos_sp_pub.publish(ps);
            }
            else if (mode == VELOCITY) {
                if (use_pid)
                    tf::vectorEigenToMsg(pid.compute_linvel_effort(ellipse_shape(theta), current, last_time), vs.twist.linear);
                else
                    tf::vectorEigenToMsg(ellipse_shape(theta) - current, vs.twist.linear);
                vel_sp_pub.publish(vs);
            }
            else if (mode == ACCELERATION) {
                // TODO
                return;
            }
            if (theta == 360) {
                ROS_INFO("Test complete!");
                ros::shutdown();
            }
            last_time = ros::Time::now();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
}

