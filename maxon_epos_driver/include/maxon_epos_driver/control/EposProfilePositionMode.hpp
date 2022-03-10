/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:04:39
 */

#ifndef _EposProfilePositionMode_HPP
#define _EposProfilePositionMode_HPP

#include "maxon_epos_driver/control/ControlModeBase.hpp"
#include "maxon_epos_driver/Device.hpp"
#include <ros/ros.h>


class EposProfilePositionMode : public ControlModeBase {
public:
    virtual void activate();
    virtual void write(double &pos, double &vel, double &cur);

private:
    double m_position_cmd;
};

#endif // _EposProfilePositionMode_HPP
