/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_local_pos;  //!< Current local position in ENU
geometry_msgs::PoseStamped current_leader_pos;  //!< Current leader position in ENU

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //current_local_pos = ftf::to_eigen(msg->pose.position);
    current_local_pos = *msg;
}

void leader_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //current_local_pos = ftf::to_eigen(msg->pose.position);
    current_leader_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber cur_leader_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_cb);
    ros::Subscriber cur_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("Robot_1/pose", 10, leader_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped last_pose;
    geometry_msgs::PoseStamped last_lead_pose;
    pose.pose.position.x = current_local_pos.pose.position.x;
    pose.pose.position.y = current_local_pos.pose.position.y;
    pose.pose.position.z = current_local_pos.pose.position.z;

    ROS_INFO("pos cmd: %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    double offb_time = 0;

    while(ros::ok()){
        if((current_state.mode == "OFFBOARD") && current_state.armed)
        {
            offb_time = (ros::Time::now() - last_request).toSec();
            double radio = 0.4;
            double rotation_freq = 0.5;
            double target_pos_offset_x = radio -radio * cos(2.0 * 3.1415 * rotation_freq * offb_time);
            double target_pos_offset_y = radio * sin(2.0 * 3.1415 * rotation_freq * offb_time);

            double leader_mov_y = 0.0;
            double leader_mov_x = 0.0;
           // leader_mov_x = current_leader_pos.pose.position.x - last_lead_pose.pose.position.x;
           // leader_mov_y = current_leader_pos.pose.position.y - last_lead_pose.pose.position.y;
            pose.pose.position.x = last_pose.pose.position.x + target_pos_offset_x + leader_mov_x;
            pose.pose.position.y = last_pose.pose.position.y + target_pos_offset_y + leader_mov_y;
            pose.pose.position.z = 1.0;

            ROS_INFO("Switch into OFFBOARD mode!");

            ROS_INFO("Continued Time: %f", offb_time);
        }
        else
        {
            pose.pose.position.x = current_local_pos.pose.position.x;
            pose.pose.position.y = current_local_pos.pose.position.y;
            pose.pose.position.z = current_local_pos.pose.position.z;

            //last_pose.pose.position.x = pose.pose.position.x;
            //last_pose.pose.position.y = pose.pose.position.y;
            //last_pose.pose.position.z = pose.pose.position.z;

            memcpy(&(last_pose.pose),&(pose.pose), sizeof(pose.pose));
            memcpy(&(last_lead_pose.pose),&(current_leader_pos.pose), sizeof(current_leader_pos.pose));

            last_request = ros::Time::now();
        }

        ROS_INFO("pos cmd: %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
