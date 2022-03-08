/**
 * @file   EposCurrentMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:43:21
 */

#include "maxon_epos_driver/control/EposCurrentMode.hpp"


EposCurrentMode::~EposCurrentMode()
{}

void EposCurrentMode::init(ros::NodeHandle &motor_nh, NodeHandle &node_handle, const std::string &controller_name)
{
    ControlModeBase::init(motor_nh, node_handle, controller_name);
}

void EposCurrentMode::activate()
{}

void EposCurrentMode::read()
{}

void EposCurrentMode::write(const double position, const double velocity, const double current)
{}
