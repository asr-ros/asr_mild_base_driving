#ifndef __CAN_LISTENER__
#define __CAN_LISTENER__

#include "RobotState.h"

/**
Gets velocity information from Can and publish them on ROS as odom information.
*/
class CanListener
{
 private:
    //Current RoboterState.
    RobotState *state;

    geometry_msgs::TransformStamped getOdomTF(ros::Time current_time);
    nav_msgs::Odometry getOdomMsg(ros::Time current_time);
    bool gettingData();
    int overflowDetection(int ticks, int ticks_old);

public:
    CanListener(RobotState *state):state(state) {};
    /**
    Get the velocity from Can, transform them and publish on ROS.
    */
    void run();

    /**
    Return the velocity of the wheels, by an average (average_size).
    */
    double get_velocity_right();
    double get_velocity_left();

    struct can_frame frame;
};

#endif
