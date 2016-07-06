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
 private:
    //Current RoboterState
    RobotState *state;
    CanListener *canListener;
    boost::mutex mutex;
    //Messages from Ros
    geometry_msgs::Twist cmd;


    int initAX10420();
    void writeDataToCan(float vleft, float vright,  float max_speed);
    bool enableMotor(int ax10420);
    float calculateVelocity(bool left, float speed, float velocity_old);

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

    /**
    Calculate the adaptation, which needed if required velocity and real velocity is different.
    @param required_velocity calculate velocity from ROS and adjusted for CAN. [cm/s]
    @param real_velocity velocity from the CAN (CanListener). [cm/s]
    @param adapter adaptation value which is updated.
    */
    double calculateAdapter(double required_velocity, double real_velocity, double adapter);

};

#endif
