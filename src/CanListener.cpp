#include <sys/socket.h>
#include <linux/sockios.h>
#include <linux/can.h>
#include <linux/if.h>
#include "CanListener.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "OwnMath.h"
#include <stdlib.h>

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
    int max_encoder = 0xffff;
    double impulses_per_mm_left = -152.8;
    double impulses_per_mm_right = 152.8;
    //distance between left and right wheel
    double wheel_distance = 663.0;
    int ticks_left, ticks_right, ticks_left_old, ticks_right_old;
    ticks_left = ticks_right = ticks_left_old = ticks_right_old = 0;
    bool first = true;
    ssize_t nbytes;
    struct can_frame frame;

    ros::Rate rate(300);
    ros::Time current_time, last_time, start_time;
    ros::Duration delta_time;
    current_time = last_time = start_time = ros::Time::now();

    //Loop until node is stopped.
    while( ros::ok() )
    {

        nav_msgs::Odometry odom;
        current_time = ros::Time::now();
        delta_time = current_time - last_time;
        //Time between two loops.
        double d_time = delta_time.toNSec();
        //********************************************************************************//
        // Receiving data from CAN-Bus
        //********************************************************************************//
        nbytes = recv(state->getSocket(), &frame, sizeof(struct can_frame), MSG_DONTWAIT);
        if (nbytes < 0)
        {
            if (errno != EAGAIN)
            {
                ROS_ERROR("CanListener: mild_base_driving raw socket read, status %i (%i)", nbytes, errno);
                exit(1);
            }
        }
        else if (nbytes < (int)sizeof(struct can_frame))
        {
            ROS_ERROR("CanListener: read: incomplete CAN frame of size %i",nbytes);
            exit(1);
        }
        else
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
            if (fabs(ticks_left - ticks_left_old) > 0.5*max_encoder)
            {
                //Overflow detected left.
                ROS_DEBUG("CanListener: Overflow left. Old ticks_left_old = %i", ticks_left_old);
                if (ticks_left > ticks_left_old)
                {
                    ticks_left_old = ticks_left_old + max_encoder;
                }
                else ticks_left_old = ticks_left_old - max_encoder;
                ROS_DEBUG("CanListener: Overflow left. New ticks_left_old = %i", ticks_left_old);
            }
            if (fabs(ticks_right - ticks_right_old) > 0.5*max_encoder)
            {
                //Overflow detected right.
                ROS_DEBUG("CanListener: Overflow right. Old ticks_right_old = %i", ticks_right_old);
                if (ticks_right > ticks_right_old)
                {
                    ticks_right_old = ticks_right_old + max_encoder;
                }
                else ticks_right_old = ticks_right_old - max_encoder;
                ROS_DEBUG("CanListener: Overflow right. Old ticks_right_old = %i", ticks_right_old);
            }
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

            ROS_DEBUG("CanListener: velocity_left: %f, velocity_right: %f", velocity_left,velocity_right);

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

                ROS_DEBUG("CanListener: Moving forward. velocity = %d", velocity);

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

                    ROS_DEBUG("CanListener: Turning in place. angular_velocity = %d , radius = %d", angular_velocity, radius);
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

                    ROS_DEBUG("CanListener: Other movement. angular_velocity = %i , radius = %i", angular_velocity, radius);
                }
            }
        }


        //********************************************************************************//
        // Publishing.
        //********************************************************************************//


        //Since all odometry is 6DOF we'll need a quaternion created from yaw.
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state->getTh());

        //First, we'll publish the transform over tf.
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = state->getX()/1000.0;
        odom_trans.transform.translation.y = state->getY()/1000.0;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //Send the transform.
        state->odom_broadcaster.sendTransform(odom_trans);


        //Next, we'll publish the odometry message over ROS.
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

        //Publish the message.
        state->odom_pub.publish(odom);

        last_time = current_time;

        rate.sleep();
    }
}
