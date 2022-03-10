/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:18:31
 */

#include <vector>
#include "maxon_epos_driver/EposManager.hpp"
#include "maxon_epos_msgs/MotorState.h"
#include "maxon_epos_msgs/MotorStates.h"

#include <boost/foreach.hpp>

/**
 * @brief Constructor
 */
EposManager::EposManager() = default;

/**
 * @brief Destructor
 */
EposManager::~EposManager() = default;


/**
 * @brief Initialize function
 *
 * @param root_nh 
 * @param motors_nh
 * @param motor_names
 *
 * @return 
 */
bool EposManager::init(ros::NodeHandle& root_nh, 
    ros::NodeHandle& robot_nh, const std::vector<std::string>& motor_names)
{
    BOOST_FOREACH (const std::string& motor_name, motor_names)
    {
        ROS_INFO_STREAM("Loading Epos: " << motor_name);
        ros::NodeHandle motor_nh(root_nh, motor_name);  // Change root_nh to robot_nh if you want the parameters to be in /robotName/...
        // create motor shared_ptr object
        std::shared_ptr<EposMotor> motor(new EposMotor());
        motor->init(root_nh, motor_nh, motor_name);
        m_motors.push_back(motor);
    }
    
    // m_all_motor_publisher = robot_nh.advertise<maxon_epos_msgs::MotorStates>("get_all_states", 100);
    // m_all_motor_subscriber = robot_nh.subscribe("set_all_states", 100, &EposManager::write, this);

    // m_active_controllers_subscriber = robot_nh.subscribe("GetActiveControllers", 100, &EposManager::get_active_controller_callback, this);
    
    return true;
}

void EposManager::modeSwitch(std::vector<std::string> starting_controller_modes, std::vector<std::string> stoping_controller_modes, ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh)
{
    int i = 0;
    BOOST_FOREACH (std::shared_ptr<EposMotor> &motor, m_motors)
    {
        ros::NodeHandle motor_nh(motors_nh, motor->m_motor_name);
        m_motors[i]->changeControlMode(starting_controller_modes[i], root_nh, motor_nh);
        i++;
    }
    // m_all_motor_publisher = motors_nh.advertise<maxon_epos_msgs::MotorStates>("get_all_states", 100);
    // m_all_motor_subscriber = motors_nh.subscribe("set_all_states", 100, &EposManager::write, this);
}

void EposManager::get_active_controller_callback(const std_msgs::StringConstPtr& msg){
    // TODO ??? needed?!
    std::vector<std::string> active_controllers;
    BOOST_FOREACH (const std::shared_ptr<EposMotor> &motor, m_motors)
    {
        std::string cmd_mode = "123"; // motor->get_active_controller();
        std::cout << "iterate..." << std::endl;
    }
    std::cout << active_controllers.size() << std::endl;
     
}

void EposManager::read(std::vector<double> &joint_positions, std::vector<double> &joint_velocities, std::vector<double> &joint_currents)
{
    int i=0;
    BOOST_FOREACH (const std::shared_ptr<EposMotor> &motor, m_motors)
    {
        motor->read(joint_positions[i], joint_velocities[i], joint_currents[i]);
        // ROS_INFO_STREAM("INSIDE EPOS" << joint_positions[i] << ", " << joint_velocities[i] << ", " <<  joint_currents[i]);
        i++;
    }
}

void EposManager::write(std::vector <double> &pos_command, std::vector <double> &vel_command, std::vector <double> &cur_command)
{
    int i=0;
    BOOST_FOREACH(const std::shared_ptr<EposMotor> &motor, m_motors )
    {
        // ROS_INFO_STREAM("Send: [" << pos_command[i] << ", " << vel_command[i] << ", " << cur_command[i] << "] to " << motor->m_motor_name);
        motor->write(pos_command[i], vel_command[i], cur_command[i]);
    }
}
