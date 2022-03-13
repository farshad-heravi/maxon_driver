/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:15:22
 */

#include <ros/ros.h>
#include "maxon_epos_driver/Definitions.h"
#include "maxon_epos_driver/EposMotor.hpp"
#include "maxon_epos_driver/utils/Macros.hpp"
#include "maxon_epos_driver/utils/EposException.hpp"
#include "maxon_epos_driver/control/EposProfilePositionMode.hpp"
#include "maxon_epos_driver/control/EposProfileVelocityMode.hpp"
#include "maxon_epos_driver/control/EposCurrentMode.hpp"

#include "maxon_epos_msgs/MotorState.h"

/**
 * @brief Constructor
 */
EposMotor::EposMotor()
    : m_position(0), m_velocity(0), m_effort(0), m_current(0)
{}

/**
 * @brief Destructor
 *        change the device state to "disable"
 */
EposMotor::~EposMotor()
{
    try {
        VCS_NODE_COMMAND_NO_ARGS(SetDisableState, m_epos_handle);
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
    }
}


void EposMotor::init(ros::NodeHandle& root_nh, ros::NodeHandle& motor_nh, const std::string& motor_name)
{
    m_motor_name = motor_name;
    // create motor handle
    initEposDeviceHandle(motor_nh);

    // disable the motor for initialization
    VCS_NODE_COMMAND_NO_ARGS(SetDisableState, m_epos_handle);
    // get and clear existed errors
    initDeviceError();
    initProtocolStackChanges(motor_nh);
    // initialize motor default controller
    initControlMode(motor_nh);
    // Set encoder properties
    initEncoderParams(motor_nh);
    // intialize position, velocity and current profiles
    initProfilePosition(motor_nh);
    initProfileVelocity(motor_nh);
    initProfileCurrent(motor_nh);
    // set other settings
    initMiscParams(motor_nh);

    // enable the motor
    VCS_NODE_COMMAND_NO_ARGS(SetEnableState, m_epos_handle);
}

void EposMotor::read(double &joint_position, double &joint_velocity, double &joint_current)
{
    m_control_mode->read(joint_position, joint_velocity, joint_current);
    // ROS_INFO_STREAM("INSIDE MOTOR" << joint_position << ", " << joint_velocity << ", " << joint_current);
}

void EposMotor::write(double &pos, double &vel, double &cur)
{
    try {
        if (m_control_mode) {
            // ROS_INFO_STREAM("Received: [" << pos << ", " << vel << ", " << cur << "] at " << m_motor_name);
            m_control_mode->write(pos, vel, cur);
        }
    } catch (const EposException &e) {
        ROS_ERROR_STREAM(e.what());
    }
}

/**
 * @brief Initialize Epos Node Handle
 *
 * @param motor_nh ros NodeHandle of motor
 */
void EposMotor::writeCallback(const maxon_epos_msgs::MotorState::ConstPtr &msg)
{
    // FIXME needed?!
    // std::vector<double> data = {msg->position, msg->velocity, msg->current};
    // write(data);
}

void EposMotor::initEposDeviceHandle(ros::NodeHandle &motor_nh)
{
    const DeviceInfo device_info(motor_nh.param<std::string>("device", "EPOS4"),
                                 motor_nh.param<std::string>("protocol_stack", "MAXON SERIAL V2"),
                                 motor_nh.param<std::string>("interface", "USB"),
                                 motor_nh.param<std::string>("port", "USB0"));
    const unsigned short node_id(motor_nh.param("node_id", 1));

    // create epos handle
    m_epos_handle = HandleManager::CreateEposHandle(device_info, node_id);
}

/**
 * @brief Clear initial errors
 */
void EposMotor::initDeviceError()
{
    unsigned char num_of_device_errors;
    // Get Current Error nums
    VCS_NODE_COMMAND(GetNbOfDeviceError, m_epos_handle, &num_of_device_errors);
    // Print existed errors to ROS_WARN_STREAM
    for (int i = 1; i <= num_of_device_errors; i++) {
        unsigned int device_error_code;
        VCS_NODE_COMMAND(GetDeviceErrorCode, m_epos_handle, i, &device_error_code);
        ROS_WARN_STREAM("EPOS Device Error: 0x" << std::hex << device_error_code);
    }

    // Clear Errors
    VCS_NODE_COMMAND_NO_ARGS(ClearFault, m_epos_handle);
    // Get Current Error nums again
    VCS_NODE_COMMAND(GetNbOfDeviceError, m_epos_handle, &num_of_device_errors);
    if (num_of_device_errors > 0) {
        throw EposException(m_motor_name + ": " + std::to_string(num_of_device_errors) + " faults uncleared");
    }
}

/**
 * @brief Set Protocol Stack for Epos Node Handle
 *
 * @param motor_nh ros NodeHandle of motor
 */
void EposMotor::initProtocolStackChanges(ros::NodeHandle &motor_nh)
{
    // load values from ros parameter server
    const unsigned int baudrate(motor_nh.param("baudrate", 0));
    const unsigned int timeout(motor_nh.param("timeout", 0));
    if (baudrate == 0 && timeout == 0) {
        return;
    }

    if (baudrate > 0 && timeout > 0) {
        VCS_COMMAND(SetProtocolStackSettings, m_epos_handle.ptr.get(), baudrate, timeout);
    } else {
        unsigned int current_baudrate, current_timeout;
        VCS_COMMAND(GetProtocolStackSettings, m_epos_handle.ptr.get(), &current_baudrate, &current_timeout);
        VCS_COMMAND(SetProtocolStackSettings, m_epos_handle.ptr.get(),
                baudrate > 0 ? baudrate : current_baudrate,
                timeout > 0 ? timeout : current_timeout);
    }
}

/**
 * @brief Initialize Control Mode
 *
 * @param motor_nh NodeHandle of motor
 */
void EposMotor::initControlMode(ros::NodeHandle &motor_nh){
    // get default control mode from rosparam
    const std::string control_mode(motor_nh.param<std::string>("control_mode", "profile_position"));
    ROS_INFO_STREAM("default control mode for " << m_motor_name << " is " << control_mode);

    if (control_mode == "profile_position") {
        m_control_mode.reset(new EposProfilePositionMode());
    } else if (control_mode == "profile_velocity") {
        m_control_mode.reset(new EposProfileVelocityMode());
    } else if (control_mode == "profile_current") {
        m_control_mode.reset(new EposCurrentMode());
    } else {
        throw EposException("Unsupported control mode (" + control_mode + ")");
    }

    m_control_mode->init(motor_nh, m_epos_handle, control_mode, m_max_qc);
    m_control_mode->activate();
}

void EposMotor::changeControlMode(const std::string &cmd_mode, ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh)
{
    VCS_NODE_COMMAND_NO_ARGS(SetDisableState, m_epos_handle);
    initDeviceError();
    initProtocolStackChanges(motor_nh);
    std::string current_active_controller = m_control_mode->name;
    if (current_active_controller == cmd_mode){
        ROS_INFO_STREAM("The " << cmd_mode << " controller has already started and is running.");
        return;
    }
    if (cmd_mode == "profile_position") {
        std::cout<< "Changing to Position Control!" <<std::endl; // to remove after debug
        m_control_mode.reset(new EposProfilePositionMode());
        VCS_NODE_COMMAND_NO_ARGS(ActivateProfilePositionMode, m_epos_handle);
        initProfilePosition(motor_nh);
    } else if (cmd_mode == "profile_velocity") {
        std::cout<<"Changing to Velocity Control"<<std::endl; // to remove after debug
        m_control_mode.reset(new EposProfileVelocityMode());
        VCS_NODE_COMMAND_NO_ARGS(ActivateProfileVelocityMode, m_epos_handle);
        initProfileVelocity(motor_nh);
    } else if (cmd_mode == "profile_current") {
        m_control_mode.reset(new EposCurrentMode());
    } else {
        throw EposException("Unsupported control mode (" + cmd_mode + ")");
    }
    m_control_mode->init(motor_nh, m_epos_handle, cmd_mode, m_max_qc);
    
    VCS_NODE_COMMAND_NO_ARGS(SetEnableState, m_epos_handle);
}

/**
 * @brief Initialize EPOS sensor(Encoder) parameters
 *
 * @param motor_nh NodeHandle of motor
 */
void EposMotor::initEncoderParams(ros::NodeHandle &motor_nh)
{   
    // create encoder ros node handler
    ros::NodeHandle encoder_nh(motor_nh, "encoder");
    // get /[robot_name]/<motor_name>/encoder/type from rosparam
    const int type(encoder_nh.param("type", 0));
    
    // set sensor type
    VCS_NODE_COMMAND(SetSensorType, m_epos_handle, type);

    if (type == 1 || type == 2) {   // Incremental Encoder
        const int resolution(encoder_nh.param("resolution", 0));
        const int gear_ratio(encoder_nh.param("gear_ratio", 0));
        if (resolution == 0 || gear_ratio == 0) {
            throw EposException("Please set both 'resolution' and 'gear_ratio' parameters");
        }
        const bool inverted_polarity(encoder_nh.param("inverted_polarity", false));
        if (inverted_polarity) {
            ROS_INFO_STREAM(m_motor_name << ": Inverted polarity is True");
        }
        VCS_NODE_COMMAND(SetIncEncoderParameter, m_epos_handle, resolution, inverted_polarity);

        m_max_qc = 4 * resolution * gear_ratio;
        ROS_INFO_STREAM(m_motor_name << "\'s max qc:" << m_max_qc);
    } else if (type == 4 || type == 5) {    // SSI Abs Encoder
        int data_rate, number_of_multiturn_bits, number_of_singleturn_bits;
        const bool inverted_polarity(encoder_nh.param("inverted_polarity", false));
        if (encoder_nh.hasParam("data_rate") && encoder_nh.hasParam("number_of_singleturn_bits") && encoder_nh.hasParam("number_of_multiturn_bits")) {
            encoder_nh.getParam("data_rate", data_rate);
            encoder_nh.getParam("number_of_multiturn_bits", number_of_multiturn_bits);
            encoder_nh.getParam("number_of_singleturn_bits", number_of_singleturn_bits);
        } else {
            ROS_ERROR("Please set 'data_rate', 'number_of_singleturn_bits', and 'number_of_multiturn_bits'");
        }
        VCS_NODE_COMMAND(SetSsiAbsEncoderParameter, m_epos_handle, data_rate, number_of_multiturn_bits, number_of_singleturn_bits, inverted_polarity);
        m_max_qc = inverted_polarity ? -(1 << number_of_singleturn_bits) : (1 << number_of_singleturn_bits);
    } else {
        // Invalid Encoder
        throw EposException("Invalid Encoder Type: " + std::to_string(type));
    }
}

void EposMotor::initProfilePosition(ros::NodeHandle &motor_nh)
{
    ros::NodeHandle profile_position_nh(motor_nh, "profile_position");
    if (profile_position_nh.hasParam("velocity")
        && profile_position_nh.hasParam("acceleration")
        && profile_position_nh.hasParam("deceleration")) {
            int velocity, acceleration, deceleration;
            profile_position_nh.getParam("velocity", velocity);
            profile_position_nh.getParam("acceleration", acceleration);
            profile_position_nh.getParam("deceleration", deceleration);
            VCS_NODE_COMMAND(SetPositionProfile, m_epos_handle, velocity, acceleration, deceleration);
    } else {
        ROS_ERROR_STREAM("Do set all parameters of ProfilePosition.");
    }
}

void EposMotor::initProfileVelocity(ros::NodeHandle &motor_nh)
{
    ros::NodeHandle profile_velocity_nh(motor_nh, "profile_velocity");
    if (profile_velocity_nh.hasParam("acceleration") && profile_velocity_nh.hasParam("deceleration")) {
        int acceleration, deceleration;
        profile_velocity_nh.getParam("acceleration", acceleration);
        profile_velocity_nh.getParam("deceleration", deceleration);
        VCS_NODE_COMMAND(SetVelocityProfile, m_epos_handle, acceleration, deceleration);
    } else {
        ROS_ERROR_STREAM("Do set all parameters of ProfileVelocity.");
    }
}

void EposMotor::initProfileCurrent(ros::NodeHandle &motor_nh)
{
    // TODO
    // ros::NodeHandle profile_current_nh(motor_nh, "profile_current");
    // if (profile_current_nh.hasParam("current_musst")) {
    //     int must;
    //     must.getParam("current_musst", must);
    //     VCS_NODE_COMMAND(SetCurrentMust, m_epos_handle, acceleration, deceleration);
    // } else {
    //     ROS_ERROR_STREAM("Do set all parameters of ProfileVelocity.");
    // }
}

/**
 * @brief Initialize other parameters
 *
 * @param motor_nh NodeHandle of motor
 */
void EposMotor::initMiscParams(ros::NodeHandle &motor_nh)
{
    // use ros unit or default epos unit
    motor_nh.param("use_ros_unit", m_use_ros_unit, false);
}