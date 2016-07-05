#ifndef __CAN_LISTENER__
#define __CAN_LISTENER__

#include "RobotState.h"

/**
Gets velocity information from Can and publish them on ROS as odom information.
*/
class CanListener
{
 private:

    //Variables
    //Current RoboterState.
    RobotState *state;
    //d=drived distance, d_left = drived distance wheel left, t = changing of the orientation.
    double d, d_left, d_right, t;
    // velocity of left/right wheel
    double velocity_left, velocity_right;
    //Means 1:144 gearing.
    double impulses_per_mm_left;
    double impulses_per_mm_right;
    //distance between left and right wheel
    double wheel_distance;
    int ticks_left, ticks_right, ticks_left_old, ticks_right_old;

    bool first;

    ros::Time current_time, last_time, start_time;

    //Average over X succressive measures.
    int average_size;
    double left_velocity_average[];
    double right_velocity_average[];

    int counter;

    double left_average;
    double right_average;

    //Methodes
    void initialize();
    geometry_msgs::TransformStamped getOdomTF(ros::Time current_time);
    nav_msgs::Odometry getOdomMsg(ros::Time current_time);
    bool gettingData();
    int overflowDetection(int ticks, int ticks_old);
    void movingForward(double d, double d_time);
    void turningInPlace(double t, double d_time);
    void otherMovement(double d, double t, double d_time);

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
