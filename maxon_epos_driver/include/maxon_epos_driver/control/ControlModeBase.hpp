/**
 * @file   ModeBase
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 11:31:01
 */

#ifndef _ModeBase_HPP
#define _ModeBase_HPP

#include <string>
#include <vector>
#include <ros/ros.h>
#include "maxon_epos_driver/Device.hpp"

class ControlModeBase {
    public:
        std::string name;
        virtual ~ControlModeBase();

        virtual void init(ros::NodeHandle &motor_nh, NodeHandle &node_handle, const std::string &controller_name, const int &m_max_qc);

        // activate operation mode
        virtual void activate() = 0;

        // read something required for operation mode
        virtual void read(double &pos, double &vel, double &cur);

        // write commands of operation mode
        virtual void write(double &pos, double &vel, double &cur) = 0;

    protected:
        bool m_use_ros_unit;
        int m_max_qc_;
        NodeHandle m_epos_handle;
        int current_pos_, current_vel_;
};

#endif // _ModeBase_HPP
