/**
 * @file   EposProfileVelocityMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:07:38
 */

#ifndef _EposProfileVelocityMode_HPP
#define _EposProfileVelocityMode_HPP

#include "maxon_epos_driver/control/ControlModeBase.hpp"
#include "maxon_epos_driver/Device.hpp"
#include <ros/ros.h>


class EposProfileVelocityMode : public ControlModeBase {
public:
    virtual void activate();
    virtual void write(double &pos, double &vel, double &cur);

private:
    double m_position_cmd;
};

#endif // _EposProfileVelocityMode_HPP
