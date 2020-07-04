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
#include <iostream>
#include <fstream>

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

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber cur_leader_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/vision_pose/pose", 10, local_cb);
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
/*    int row = 3;
    int col = 200;
    float path = 1.1;
    ifstream infile;
    infile.open("path.txt");
    for (int i = 0; i < row; i++){
        for(int j = 0; j < col; j++){
            infile >> path;
             cout << path << endl; 
        }
    }
    infile.close();
    double y[200]={0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.01,0.01,0.02,0.03,0.04,0.05,0.07,0.09,0.11,0.13,0.16,0.19,0.23,0.27,0.31,0.35,0.39,0.44,0.49,0.54,0.59,0.64,0.68,0.73,0.78,0.82,0.86,0.89,0.92,0.95,0.97,0.99,1.00,1.00,1.00,0.99,0.98,0.96,0.94,0.91,0.87,0.83,0.79,0.74,0.68,0.62,0.56,0.49,0.42,0.35,0.28,0.20,0.12,0.04,-0.04,-0.12,-0.20,-0.27,-0.35,-0.42,-0.49,-0.56,-0.63,-0.69,-0.75,-0.80,-0.84,-0.89,-0.92,-0.95,-0.97,-0.99,-1.00,-1.00,-1.00,-0.99,-0.97,-0.95,-0.92,-0.88,-0.84,-0.80,-0.74,-0.69,-0.62,-0.56,-0.49,-0.42,-0.34,-0.27,-0.19,-0.11,-0.03,0.05,0.14,0.21,0.29,0.37,0.44,0.52,0.58,0.65,0.71,0.76,0.81,0.86,0.90,0.93,0.96,0.98,0.99,1.00,1.00,1.00,0.99,0.97,0.94,0.91,0.87,0.83,0.78,0.73,0.67,0.60,0.54,0.47,0.39,0.32,0.24,0.16,0.08,0.00,-0.08,-0.16,-0.24,-0.31,-0.39,-0.46,-0.52,-0.59,-0.65,-0.71,-0.76,-0.81,-0.85,-0.89,-0.92,-0.95,-0.97,-0.99,-1.00,-1.01,-1.00,-1.00,-0.98,-0.97,-0.94,-0.91,-0.88,-0.84,-0.80,-0.75,-0.71,-0.66,-0.61,-0.55,-0.50,-0.45,-0.40,-0.35,-0.31,-0.26,-0.22,-0.18,-0.14,-0.11,-0.08,-0.05,-0.03,-0.01,0.01,0.02,0.03,0.04,0.05,0.06,0.06,0.06,0.06,0.07,0.07,0.07,0.07};
    double x[200]={0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.01,0.01,0.02,0.02,0.03,0.05,0.06,0.07,0.09,0.11,0.13,0.16,0.19,0.22,0.25,0.29,0.32,0.36,0.40,0.44,0.49,0.53,0.58,0.62,0.67,0.72,0.76,0.81,0.85,0.89,0.93,0.98,1.01,1.04,1.08,1.11,1.15,1.18,1.21,1.23,1.26,1.28,1.31,1.33,1.34,1.36,1.37,1.39,1.39,1.40,1.41,1.41,1.41,1.41,1.40,1.40,1.39,1.38,1.36,1.35,1.33,1.31,1.29,1.27,1.24,1.21,1.19,1.16,1.12,1.09,1.05,1.02,0.99,0.95,0.91,0.87,0.82,0.78,0.73,0.68,0.63,0.58,0.53,0.48,0.43,0.37,0.32,0.26,0.21,0.15,0.09,0.04,-0.02,-0.08,-0.13,-0.19,-0.25,-0.30,-0.36,-0.41,-0.47,-0.52,-0.57,-0.62,-0.67,-0.72,-0.77,-0.82,-0.86,-0.90,-0.95,-0.99,-1.02,-1.06,-1.09,-1.13,-1.16,-1.19,-1.22,-1.25,-1.28,-1.30,-1.32,-1.34,-1.36,-1.38,-1.39,-1.41,-1.41,-1.42,-1.43,-1.43,-1.43,-1.43,-1.42,-1.41,-1.40,-1.39,-1.38,-1.36,-1.34,-1.32,-1.30,-1.27,-1.24,-1.21,-1.18,-1.15,-1.12,-1.08,-1.04,-1.01,-0.96,-0.92,-0.88,-0.83,-0.79,-0.74,-0.69,-0.64,-0.60,-0.55,-0.50,-0.45,-0.41,-0.37,-0.32,-0.28,-0.24,-0.21,-0.17,-0.14,-0.11,-0.09,-0.06,-0.04,-0.02,-0.00,0.01,0.02,0.03,0.04,0.05,0.05,0.06,0.06,0.06,0.06,0.06,0.07,0.07,0.07};
*/
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
    last_pose.pose.position.x = current_local_pos.pose.position.x;
    last_pose.pose.position.y = current_local_pos.pose.position.y;
    last_pose.pose.position.z = current_local_pos.pose.position.z;
    pose.pose.orientation.w = current_local_pos.pose.orientation.w;
    pose.pose.orientation.x = current_local_pos.pose.orientation.x;
    pose.pose.orientation.y = current_local_pos.pose.orientation.y;
    pose.pose.orientation.z = current_local_pos.pose.orientation.z;
    

    ROS_INFO("pos cmd: %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z); 

    //send a few setpoints before starting1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00
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
    
    int i = 0;
    while(ros::ok()){
        if((current_state.mode == "OFFBOARD") && current_state.armed)
        {
            i++;
            offb_time = (ros::Time::now() - last_request).toSec();
            double radio = 0.5;
            double rotation_freq = 0.1;
            //double target_pos_offset_x = radio * cos(2.0 * 3.1415 * rotation_freq * offb_time);
            //double target_pos_offset_y = radio * sin(2.0 * 3.1415 * rotation_freq * offb_time);
            double  target_pos_offset_y = 0.0; 
            double target_pos_offset_x = 0.3;
            double target_pos_offset_z = 1.0;
/*            if (i<=200)
            {
                target_pos_offset_y = y[i-1];
                target_pos_offset_x = x[i-1];
                ROS_INFO("running path");
            }
            else
            {
                i=0;
                target_pos_offset_y = y[199];
                target_pos_offset_x = x[199];
                ROS_INFO("<----PATH FINISHED---->");
            }
            ROS_INFO("I= %i", i);*/
            double leader_mov_y = 0.0;
            double leader_mov_x = 0.0;
            //leader_mov_x = current_leader_pos.pose.position.x - last_lead_pose.pose.position.x;
            //leader_mov_y = current_leader_pos.pose.position.y - last_lead_pose.pose.position.y;
            pose.pose.position.x = current_leader_pos.pose.position.x + target_pos_offset_x  + leader_mov_x;
            pose.pose.position.y = current_leader_pos.pose.position.y+ target_pos_offset_y + leader_mov_y;
            pose.pose.position.z = target_pos_offset_z;
            if((pose.pose.position.y-last_pose.pose.position.y) != 0 || (pose.pose.position.x-last_pose.pose.position.x) != 0)
            {
                pose.pose.orientation.w = cos(0.5*atan2(pose.pose.position.y-last_pose.pose.position.y , pose.pose.position.x-last_pose.pose.position.x));
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = sin(0.5*atan2(pose.pose.position.y-last_pose.pose.position.y , pose.pose.position.x-last_pose.pose.position.x));
                last_pose.pose.orientation.w = pose.pose.orientation.w;
                last_pose.pose.orientation.x = pose.pose.orientation.x;
                last_pose.pose.orientation.y = pose.pose.orientation.y;
                last_pose.pose.orientation.z = pose.pose.orientation.z;
            }
            else{
                pose.pose.orientation.w = last_pose.pose.orientation.w;
                pose.pose.orientation.x = last_pose.pose.orientation.x;
                pose.pose.orientation.y = last_pose.pose.orientation.y;
                pose.pose.orientation.z = last_pose.pose.orientation.z;
            }
            last_pose.pose.position.x = pose.pose.position.x;
            last_pose.pose.position.y = pose.pose.position.y;
            last_pose.pose.position.z = pose.pose.position.z;
            if(i%20==1)
            {
                ROS_INFO("Switch into OFFBOARD mode!");

                ROS_INFO("Continued Time: %f", offb_time);
            }
        }
        else
        {
            pose.pose.position.x = current_local_pos.pose.position.x;
            pose.pose.position.y = current_local_pos.pose.position.y;
            pose.pose.position.z = current_local_pos.pose.position.z;
            pose.pose.orientation.w = current_local_pos.pose.orientation.w;
            pose.pose.orientation.x = current_local_pos.pose.orientation.x;
            pose.pose.orientation.y = current_local_pos.pose.orientation.y;
            pose.pose.orientation.z = current_local_pos.pose.orientation.z;

            //last_pose.pose.position.x = pose.pose.position.x;
            //last_pose.pose.position.y = pose.pose.position.y;
            //last_pose.pose.position.z = pose.pose.position.z;

            memcpy(&(last_pose.pose),&(pose.pose), sizeof(pose.pose));
            memcpy(&(last_lead_pose.pose),&(current_leader_pos.pose), sizeof(current_leader_pos.pose));

            last_request = ros::Time::now();
        }
        if(i%2==1)
        {
            //ROS_INFO("pos cmd: %f %f %f", x[i], y[i], pose.pose.position.z);
            //ROS_INFO("I = : %i", i);
            //ROS_INFO("lead pos: %f %f %f", current_leader_pos.pose.position.x, current_leader_pos.pose.position.y, current_leader_pos.pose.position.z);
            //ROS_INFO("local pos: %f %f %f", current_local_pos.pose.position.x, current_local_pos.pose.position.y, current_local_pos.pose.position.z);
            //ROS_INFO("local orientation: %f %f %f %f", current_local_pos.pose.orientation.w, current_local_pos.pose.orientation.x, current_local_pos.pose.orientation.y, current_local_pos.pose.orientation.z);
            //ROS_INFO("TIME IS: %d", last_request);
        }
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
