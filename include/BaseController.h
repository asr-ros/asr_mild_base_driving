#ifndef __BASE_CONTROLLER__
#define __BASE_CONTROLLER__

#include "RobotState.h"
#include <boost/thread/mutex.hpp>
#include "geometry_msgs/Twist.h"
#include "CanListener.h"

/**
Get drive/velocity commends from ros. Transfrom and send them to CAN.
*/
class BaseController
{
protected:
    //Current RoboterState
    RobotState *state;
    CanListener *canListener;
    boost::mutex mutex;
    //Messages from Ros
    geometry_msgs::Twist cmd;

public:
    /**
    This function is triggered when a Twist message is received.
    */
    void setTargetVelocity(const geometry_msgs::Twist &twist);
    BaseController(RobotState *state, CanListener *canListener1):state(state)
    {
        canListener = canListener1;
    };
    /**
    This function handles all the mild_base_driving bus/motor/relais board related stuff.
    Gets the messages from ros and transform them to can-commands.
    */
    void run();

};

#endif
