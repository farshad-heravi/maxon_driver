/**
 * @file   maxon_bringup
 * @brief  
 * @author arwtyxouymz
 * @date   2019-05-24 17:49:41
 */

#include <ros/ros.h>
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "maxon_epos_driver/EposManager.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "maxon_bringup");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::vector<std::string> motor_names;
    if (!private_nh.getParam("motor_names", motor_names)) {
        ROS_FATAL("Failed to load motor_names");
        return 1;
    }

    ros::Rate sleep_rate(50);
    EposManager manager;
    if (!manager.init(nh, private_nh, motor_names))
    {
        ROS_FATAL("Failed to initialize EposManager");
        return 1;
    }
    ROS_INFO("Motors Initialized");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok()) {
        manager.read();
        sleep_rate.sleep();
            std_msgs::StringConstPtr m_control_mod = ros::topic::waitForMessage<std_msgs::String>("/maxon_bringup/set_mode",ros::Duration(0.15)); //this is nice
            if (m_control_mod) 
            {
                std::string new_mode = m_control_mod->data.c_str();
                std::cout<<new_mode<<" Msg received!"<<std::endl; // to remove after debug
                manager.modeSwitch(new_mode,nh, private_nh, motor_names);
            }
    }

    spinner.stop();

    return 0;
}
