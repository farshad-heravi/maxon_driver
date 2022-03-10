/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:08:23
 */

#ifndef _EposManager_HPP
#define _EposManager_HPP

#include <string>
#include <vector>
#include <ros/ros.h>
#include "maxon_epos_driver/EposMotor.hpp"
#include "maxon_epos_msgs/MotorStates.h"

class EposManager {

public:
    EposManager();
    virtual ~EposManager();

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh, const std::vector<std::string>& motor_names);

    void read(std::vector<double> &joint_positions, std::vector<double> &joint_velocities, std::vector<double> &joint_currents);

    void write(std::vector <double> &pos_command, std::vector <double> &vel_command, std::vector <double> &cur_command);

    void modeSwitch(std::vector<std::string> starting_controller_modes, std::vector<std::string> stoping_controller_modes, ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh);

private:
    std::vector<std::shared_ptr<EposMotor>> m_motors;
    ros::Publisher m_all_motor_publisher;
    ros::Subscriber m_all_motor_subscriber;
    ros::Subscriber m_active_controllers_subscriber;
    std::string cmd_mode;

    void get_active_controller_callback(const std_msgs::StringConstPtr& msg);
};

#endif // _EposManager_HPP
