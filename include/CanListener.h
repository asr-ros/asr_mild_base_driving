/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Dehmani Souheil, Marek Felix, Meißner Pascal, Reckling Reno

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

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
    double impulses_per_mm_left;
    double impulses_per_mm_right;
    double wheel_distance;
    double left_average;
    double right_average;
    //Average over X succressive measures.
    int average_size;
    double left_velocity_average[50];
    double right_velocity_average[50];
    int counter;

    /**
    Initalize global paramter.
    */
    void initalize();
    /**
    Get the how the actual odom positin is transformed to origin.
    */
    geometry_msgs::TransformStamped getOdomTF(ros::Time current_time);
    /**
    Create the odom message, which get published.
    */
    nav_msgs::Odometry getOdomMsg(ros::Time current_time);
    /**
    Return true if data from Can is received.
    */
    bool gettingData();
    /**
    Detect overflow. If wheel rotate 360°.
    */
    int overflowDetection(int ticks, int ticks_old);
    /**
    Calculate position + orientation if moving forward.
    */
    void movingForward(double d, double d_time);
    /**
    Calculate position + orientation if rotate.
    */
    void turningInPlace(double t, double d_time);
    /**
    Calculate position + orientation if combined movment of rotation and translation.
    */
    void otherMovement(double d, double t, double d_time);
    /**
    Calculate average velocity over the last average_size velocities.
    */
    double calculateAverage(double velocity_average[], double velocity);

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
