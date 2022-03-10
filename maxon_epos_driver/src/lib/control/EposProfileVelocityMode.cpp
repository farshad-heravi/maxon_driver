/**
 * @file   EposProfileVelocityMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:42:41
 */

#include "maxon_epos_driver/control/EposProfileVelocityMode.hpp"

void EposProfileVelocityMode::activate()
{   
    VCS_NODE_COMMAND_NO_ARGS(ActivateProfileVelocityMode, m_epos_handle);
}

void EposProfileVelocityMode::write(double &pos, double &vel, double &cur)
{
    int quad_count;
    // ! implement ros unit conversion // TODO 
    ROS_DEBUG_STREAM("Target velocity: " << vel);
    ROS_DEBUG_STREAM("Send Velocity: " << vel);
    VCS_NODE_COMMAND(MoveWithVelocity, m_epos_handle, vel);
}
