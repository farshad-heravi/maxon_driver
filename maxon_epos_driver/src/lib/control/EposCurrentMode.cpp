/**
 * @file   EposCurrentMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:43:21
 */

#include "maxon_epos_driver/control/EposCurrentMode.hpp"

void EposCurrentMode::activate()
{
    // TODO test it!
    VCS_NODE_COMMAND_NO_ARGS(ActivateCurrentMode, m_epos_handle);
}

void EposCurrentMode::write(double &pos, double &vel, double &cur)
{
    //  TODO
    
}
