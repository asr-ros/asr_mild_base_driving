#ifndef __ROBOT_STATE__
#define __ROBOT_STATE__

#include <boost/thread/mutex.hpp>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
/**

All the information which must be used to estimate the pose
of the robot in odom frame are stored here and initialized to zero.


*/
class RobotState
{

protected:
    //Position at x-axis
    double x;
    //Position at y-axis
    double y;
    //Alignment in radien
    double th;
    //Velocity x-axis [m/s]
    double vx;
    //Velocity y-axis [m/s]
    double vy;
    //Velocity z-axis [rad]
    double vth;
    boost::mutex mutex;
    //Connection with can-bus
    int socket;

public:
    ros::Publisher odom_pub;
    ros::NodeHandle *n;
    tf::TransformBroadcaster odom_broadcaster;

    RobotState(ros::NodeHandle *n, int socket):socket(socket),n(n)
    {
        odom_pub = n->advertise<nav_msgs::Odometry>("odom", 50);
        x = 0.0;
        y = 0.0;
        th = 0.0;
        vx = 0.0;
        vy = 0.0;
        vth = 0.0;
    };

    double getX()
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        return x;
    };

    double getY()
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        return y;
    };

    double getTh()
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        return th;
    };

    double getVX()
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        return vx;
    };

    double getVY()
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        return vy;
    };

    double getVTh()
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        return vth;
    };

    int getSocket()
    {
        return socket;
    }

    void setX(double v)
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        x = v;
    };

    void setY(double v)
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        y = v;
    };

    void setTh(double v)
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        th = v;
    };

    void setVX(double v)
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        vx = v;
    };

    void setVY(double v)
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        vy = v;
    };

    void setVTh(double v)
    {
        boost::mutex::scoped_lock scoped_lock(mutex);
        vth = v;
    };
};

#endif

