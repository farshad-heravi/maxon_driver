/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:40:57
 */

#include "maxon_epos_driver/control/EposProfilePositionMode.hpp"

void EposProfilePositionMode::activate()
{
    ROS_INFO_STREAM("Activating profile_position controller.");
    VCS_NODE_COMMAND_NO_ARGS(ActivateProfilePositionMode, m_epos_handle);
}

void EposProfilePositionMode::write(double &pos, double &vel, double &cur)
{
    int quad_count;
    // ROS_INFO_STREAM("Received: [" << pos << ", " << vel << ", " << cur << "] at controller");
    ROS_DEBUG_STREAM("Target Position: " << pos);
    ROS_DEBUG_STREAM("Encoder Resolution: " << m_max_qc_);
    if (m_use_ros_unit) {
        quad_count = static_cast<int>((pos / (2 * M_PI)) * m_max_qc_);
    } else {
        quad_count = static_cast<int>(pos);
    }
    ROS_DEBUG_STREAM("Send Quad Count: " << quad_count);
    VCS_NODE_COMMAND(MoveToPosition, m_epos_handle, quad_count, /* absolute */true, /* overwrite */true);
}
