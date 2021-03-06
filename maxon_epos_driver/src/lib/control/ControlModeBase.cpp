/**
 * @file   ControlModeBase
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-05 15:28:40
 */

#include "maxon_epos_driver/control/ControlModeBase.hpp"
#include "maxon_epos_driver/Definitions.h"
#include <vector>

/**
 * @brief Destructor
 */
ControlModeBase::~ControlModeBase()
{}

void ControlModeBase::init(ros::NodeHandle &motor_nh, NodeHandle &node_handle,
    const std::string &controller_name, const int &m_max_qc)
{
    name = controller_name;
    m_epos_handle = node_handle;
    m_use_ros_unit = motor_nh.param("use_ros_unit", false);
    m_max_qc_ = m_max_qc;
    current_pos_ = 0;
    current_vel_ = 0;
}

void ControlModeBase::read(double &pos, double &vel, double &cur)
{
    int positionls = 0, velocityls = 0;
    short currentls = 0;
    try{
        VCS_NODE_COMMAND(GetPositionIs, m_epos_handle, &positionls);
    } catch (const EposException &e) {
        ROS_ERROR_STREAM("Epos GetPositionIs error!");
        ROS_ERROR_STREAM(e.what());
    }
    // VCS_NODE_COMMAND(GetVelocityIsAveraged, m_epos_handle, &velocityls);
    // VCS_NODE_COMMAND(GetCurrentIsAveraged, m_epos_handle, &currentls);       // UNCOMMENT later

    if(m_use_ros_unit){
        // quad-counts of the encoder -> rad
        pos = (positionls / static_cast<double>(m_max_qc_)) * 2. * M_PI;
        // rpm -> rad/s
        vel = velocityls * M_PI / 30.;
        // mA -> A
        cur = currentls / 1000.;
        return;
    }
    pos = positionls;
    vel = velocityls;
    cur = currentls;
    ROS_DEBUG_STREAM("INSIDE Controller read " << pos << ", " << vel << ", " << cur);
}
