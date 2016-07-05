#include <sys/socket.h>
#include <linux/sockios.h>
#include <linux/can.h>
#include <linux/if.h>
#include "CanListener.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "OwnMath.h"
#include <stdlib.h>


double left_average = 0;
double right_average = 0;

geometry_msgs::TransformStamped CanListener::getOdomTF(ros::Time current_time)
{

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state->getTh());

    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = state->getX()/1000.0;
    odom_trans.transform.translation.y = state->getY()/1000.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;

}

nav_msgs::Odometry CanListener::getOdomMsg(ros::Time current_time)
{

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state->getTh());

    nav_msgs::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //Set the position.
    odom.pose.pose.position.x = state->getX()/1000.0;
    odom.pose.pose.position.y = state->getY()/1000.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //Set the velocity.
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = state->getVX()/1000.0;
    odom.twist.twist.linear.y = state->getVY()/1000.0;
    odom.twist.twist.angular.z = state->getVTh();

    return odom;

}
bool CanListener::gettingData()
{
    ssize_t nbytes;
    nbytes = recv(state->getSocket(), &frame, sizeof(struct can_frame), MSG_DONTWAIT);
    if (nbytes < 0)
    {
        if (errno != EAGAIN)
        {
            ROS_ERROR("CanListener: mild_base_driving raw socket read, status %zu (%i)", nbytes, errno);
            exit(1);
        }
    }
    else if (nbytes < (int)sizeof(struct can_frame))
    {
        ROS_ERROR("CanListener: read: incomplete CAN frame of size %zu",nbytes);
        exit(1);
    }
    return true;
}

int CanListener::overflowDetection(int ticks, int ticks_old)
{
    int max_encoder = 0xffff;
    if (fabs(ticks - ticks_old) > 0.5*max_encoder)
    {
        //Overflow detected left.
        ROS_DEBUG("CanListener: Overflow left. Old ticks_old = %i", ticks_old);
        if (ticks > ticks_old)
        {
            ticks_old = ticks_old + max_encoder;
        }
        else ticks_old = ticks_old - max_encoder;
        ROS_DEBUG("CanListener: Overflow left. New ticks_old = %i", ticks_old);
    }
    return ticks_old;
}
void CanListener::run()
{

    //********************************************************************************//
    // Initialisation.
    //********************************************************************************//
    //d=drived distance, d_left = drived distance wheel left, t = changing of the orientation.
    double d = 0, d_left = 0, d_right = 0, t =0 ;
    // velocity of left/right wheel
    double velocity_left = 0, velocity_right = 0;
    // angular_velocitiy, velocity = velocity of whole robot, radius = radius of driven circle
    double angular_velocity = 0, velocity = 0, radius = 0;
    //Means 1:144 gearing.
    double impulses_per_mm_left = -152.8;
    double impulses_per_mm_right = 152.8;
    //distance between left and right wheel
    double wheel_distance = 663.0;
    int ticks_left, ticks_right, ticks_left_old, ticks_right_old;
    ticks_left = ticks_right = ticks_left_old = ticks_right_old = 0;
    bool first = true;

    ros::Rate rate(300);
    ros::Time current_time, last_time, start_time;
    ros::Duration delta_time;
    current_time = last_time = start_time = ros::Time::now();

    //Average over X succressive measures.
    int average_size = 50;
    double left_velocity_average[50] = {};
    double right_velocity_average[50] = {};

    int counter = 0;

    //Loop until node is stopped.
    while( ros::ok() )
    {

        current_time = ros::Time::now();
        delta_time = current_time - last_time;
        //Time between two loops.
        double d_time = delta_time.toNSec();
        //********************************************************************************//
        // Receiving data from CAN-Bus
        //********************************************************************************//
        if(gettingData())
        {

            ticks_left = (frame.data[3]<<8)+frame.data[2];
            ticks_right = (frame.data[1]<<8)+frame.data[0];

            ROS_DEBUG("ticks_lef: %i, ticks_right: %i", ticks_left,ticks_right);
            if(first)
            {
                ROS_INFO("CanListener: Started successfully.\n");
                ticks_left_old = ticks_left;
                ticks_right_old = ticks_right;
                first = false;
            }


            //********************************************************************************//
            // Overflow detection.
            //********************************************************************************//
            ticks_left_old = overflowDetection(ticks_left, ticks_left_old);
            ticks_right_old = overflowDetection(ticks_right, ticks_right_old);
            //End overflow detection.

            //********************************************************************************//
            // Calculation the estimated velocities from the ticks
            //********************************************************************************//
            if (delta_time > ros::Duration(0,0))
            {
                d_left  =  (ticks_left  - ticks_left_old) / impulses_per_mm_left ;
                d_right =  (ticks_right - ticks_right_old) / impulses_per_mm_right ;

                velocity_left = d_left / d_time;
                velocity_right = d_right / d_time;


            }
            ROS_DEBUG_STREAM("CanListener: D left: " << d_left  << ", D right: " << d_right);
            ticks_left_old = ticks_left;
            ticks_right_old = ticks_right;

            ROS_DEBUG("CanListener: velocity_left: %f, velocity_right: %f", velocity_left*100000000,velocity_right*100000000);

            //Calculate average velocity
            left_velocity_average[counter]= velocity_left;
            right_velocity_average[counter]= velocity_right;

            counter++;
            if(counter >= average_size)
            {
                counter = 0;
            }

            double left_sum = 0;
            double right_sum = 0;

            for(int i = 0; i < average_size; i++)
            {
                left_sum += left_velocity_average[i];
                right_sum += right_velocity_average[i];
            }

            left_average = left_sum/average_size;
            right_average = right_sum/average_size;

            ROS_DEBUG("CanListener: velocity_left: %f, velocity_right: %f", left_average*10000000,right_average*10000000);

            d = ( d_left + d_right ) / 2 ;
            ROS_DEBUG("CanListener: Driven dinstance =  %f", d);

            t = ( d_right - d_left ) / wheel_distance;
            ROS_DEBUG("CanListener: Orientation change =  %f", t);

            //********************************************************************************//
            // Calculating the Odometry.
            //********************************************************************************//

            //Moving forward.
            if(velocity_left == velocity_right)
            {
                velocity = d / d_time;
                double tx = state->getX()+velocity*cos(state->getTh())*d_time;
                double ty = state->getY()+velocity*sin(state->getTh())*d_time;

                state->setVX((tx-state->getX())/d_time);
                state->setVY((ty-state->getY())/d_time);
                state->setVTh(0);
                state->setX(tx);
                state->setY(ty);

                ROS_DEBUG("CanListener: Moving forward. velocity = %f", velocity);

            }
            else
            {
                angular_velocity = t / d_time;
                radius = d / t;

                //Turning in place.
                if(velocity_left == -velocity_right)
                {
                    state->setVX(0);
                    state->setVY(0);
                    double tth = state->getTh() + t ;
                    state->setVTh((tth-state->getTh())/d_time);
                    state->setTh(tth);

                    ROS_DEBUG("CanListener: Turning in place. angular_velocity = %f , radius = %f", angular_velocity, radius);
                    //Other Movements.
                }
                else
                {
                    double iccx = state->getX() - radius*sin(state->getTh());
                    double iccy = state->getY() + radius*cos(state->getTh());

                    double tx =
                        cos(t)*(state->getX()-iccx)
                        -sin(t)*(state->getY()-iccy) + iccx;
                    double ty =
                        sin(t)*(state->getX()-iccx)
                        +cos(t)*(state->getY()-iccy) + iccy;
                    double tth =
                        state->getTh() + t;

                    state->setVX((tx-state->getX())/d_time);
                    state->setVY((ty-state->getY())/d_time);
                    state->setVTh((tth-state->getTh())/d_time);
                    state->setX(tx);
                    state->setY(ty);
                    state->setTh(tth);

                    ROS_DEBUG("CanListener: Other movement. angular_velocity = %f , radius = %f", angular_velocity, radius);
                }
            }
        }

        //********************************************************************************//
        // Publishing.
        //********************************************************************************//

        //Since all odometry is 6DOF we'll need a quaternion created from yaw.
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state->getTh());

        //First, we'll publish the transform over tf.
        state->sendTransformOdomTF(getOdomTF(current_time));

        //Next, we'll publish the odometry message over ROS.
        state->odom_pub.publish(getOdomMsg(current_time));

        last_time = current_time;

        rate.sleep();
    }
}

double CanListener::get_velocity_right()
{
    return right_average*10000000;
}
double CanListener::get_velocity_left()
{
    return left_average*10000000;
}
