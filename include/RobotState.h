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
  tf::TransformBroadcaster odom_broadcaster;

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
 ros::NodeHandle* n;
 RobotState(ros::NodeHandle* n, int socket) : socket(socket), n(n)
    {
      odom_pub = n->advertise<nav_msgs::Odometry>("odom", 50);
      x = 0.0;
      y = 0.0;
      th = 0.0;
      vx = 0.0;
      vy = 0.0;
      vth = 0.0;
    };

  void sendTransformOdomTF(geometry_msgs::TransformStamped odom_trans) { odom_broadcaster.sendTransform(odom_trans); }

  void publishOdomMsg(nav_msgs::Odometry odom) { odom_pub.publish(odom); }

  ros::NodeHandle* getNodeHandle() {return n;}

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

  void setX(double x)
  {
    boost::mutex::scoped_lock scoped_lock(mutex);
    this->x = x;
  };

  void setY(double y)
  {
    boost::mutex::scoped_lock scoped_lock(mutex);
    this->y = y;
  };

  void setTh(double th)
  {
    boost::mutex::scoped_lock scoped_lock(mutex);
    this->th = th;
  };

  void setVX(double vx)
  {
    boost::mutex::scoped_lock scoped_lock(mutex);
    this->vx = vx;
  };

  void setVY(double vy)
  {
    boost::mutex::scoped_lock scoped_lock(mutex);
    this->vy = vy;
  };

  void setVTh(double vth)
  {
    boost::mutex::scoped_lock scoped_lock(mutex);
    this->vth = vth;
  };

};

#endif

