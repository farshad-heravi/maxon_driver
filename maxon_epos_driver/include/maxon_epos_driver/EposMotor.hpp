/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:12:59
 */

#ifndef _EposMotor_HPP
#define _EposMotor_HPP

#include <string>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "std_msgs/String.h"
#include "maxon_epos_driver/Device.hpp"
#include "maxon_epos_driver/control/ControlModeBase.hpp"
#include "maxon_epos_msgs/MotorState.h"


class EposMotor {
public:
    std::string m_motor_name;
    int m_zero;
    EposMotor();
    virtual ~EposMotor();

    void init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh, const std::string &motor_name);

    void read(double &joint_position, double &joint_velocity, double &joint_current);
    void write(double &pos, double &vel, double &cur);
    void changeControlMode(const std::string &cmd_mode, ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh);
    std::string get_active_controller();
    int rad2ticks(const double &position);
    double ticks2rad(const int &ticks);
    double ticks2rad(const double &ticks);
    void setZero();
    void Start();
    void Stop();

private:
    void initEposDeviceHandle(ros::NodeHandle &motor_nh);
    void initDeviceError();
    void initProtocolStackChanges(ros::NodeHandle &motor_nh);
    void initControlMode(ros::NodeHandle &motor_nh);
    void initEncoderParams(ros::NodeHandle &motor_nh);
    void initProfilePosition(ros::NodeHandle &motor_nh);
    void initProfileVelocity(ros::NodeHandle &motor_nh);
    void initProfileCurrent(ros::NodeHandle &motor_nh);
    void initMiscParams(ros::NodeHandle &motor_nh);

    void writeCallback(const maxon_epos_msgs::MotorState::ConstPtr &msg);

    static void WriteThread(EposMotor * pModule);
    static void ReadThread(EposMotor * pModule);
    void WriteLoop();
    void ReadLoop();
    void setTargetPoint(const double &pos);
    void setUpdateInterval(int milis);
    int getUpdateInterval();
    void setDesiredVelocity(const double &vel);
    int getDesiredVelocity();

    //! Flag indicating if the write loop should keep on spinning.
    std::atomic_bool m_bIsRunning;
    std::atomic_bool m_readLoop;
    //! Motor target position in ticks
    int m_target_pos;
    //! Pointer to the write thread. Will be nullptr if the module is stopped.
    std::thread* m_pWriteThread; 
    std::thread* m_pReadThread; 
    int m_desired_velocity; 

private:
    typedef std::shared_ptr<ControlModeBase> ControlModePtr;
    typedef std::map<std::string, ControlModePtr> ControlModeMap;


    NodeHandle m_epos_handle;
    ControlModePtr m_control_mode;
    ControlModeMap m_control_mode_map;
    ros::NodeHandle motor_nh_;

    double m_position;
    double m_velocity;
    double m_effort;
    double m_current;
    int m_update_interval_;  // in ms

    ros::Publisher m_state_publisher;
    ros::Subscriber m_state_subscriber;
    ros::Subscriber m_mode_subscriber;

    int m_max_qc;
    bool m_use_ros_unit;
};

#endif // _EposMotor_HPP
