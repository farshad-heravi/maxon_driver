/**
 * @file   EposCurrentMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:08:10
 */

#ifndef _EposCurrentMode_HPP
#define _EposCurrentMode_HPP

#include "maxon_epos_driver/control/ControlModeBase.hpp"
#include "maxon_epos_driver/Device.hpp"
#include <ros/ros.h>


class EposCurrentMode : public ControlModeBase {
    public:
        virtual void activate();
        virtual void write(double &pos, double &vel, double &cur);

};

#endif // _EposCurrentMode_HPP
