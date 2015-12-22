#include <sys/socket.h>
#include <linux/sockios.h>
#include <linux/can.h>
#include <linux/if.h>
#include "CanListener.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "OwnMath.h"
#include <stdlib.h>


void CanListener::run() {

    //Initialisation
    double d = 0, d_left = 0, d_right = 0, t =0 ;
    double velocity_left = 0, velocity_right = 0;
    double angular_velocity = 0, velocity = 0, radius = 0;
    int max_encoder = 0xffff;
    double impulses_per_mm_left = -152.8;
    double impulses_per_mm_right = 152.8;
    double wheel_distance = 663.0;
    int ticks_left, ticks_right, ticks_left_old, ticks_right_old, frame_size;
    ticks_left = ticks_right = ticks_left_old = ticks_right_old = 0;
    bool first = true;
    ssize_t nbytes;
    struct can_frame frame;

    ros::Rate rate(300);
    ros::Time current_time, last_time, start_time;
    ros::Duration delta_time;
    current_time = last_time = start_time = ros::Time::now();


    while( ros::ok() ) {

        nav_msgs::Odometry odom;
        current_time = ros::Time::now();
        delta_time = current_time - last_time;
        double d_time = delta_time.toNSec();
        //********************************************************************************//
        // Receiving data from CAN-Bus
        //********************************************************************************//
        nbytes = recv(state->getSocket(), &frame, sizeof(struct can_frame), MSG_DONTWAIT);
        if (nbytes < 0) {
            if (errno != EAGAIN) {
                frame_size = nbytes;
                ROS_ERROR("CAN raw socket read, status %i (%i)", frame_size, errno);
                exit(1);
            }
        } else if (nbytes < (int)sizeof(struct can_frame)) {
            frame_size = nbytes;
            ROS_ERROR("read: incomplete CAN frame of size %i",frame_size);
            exit(1);
        } else {

            ticks_left = (frame.data[3]<<8)+frame.data[2];
            ticks_right = (frame.data[1]<<8)+frame.data[0];

            if(first) {
                ROS_INFO("Received first encoder data on the CAN bus");
                ticks_left_old = ticks_left;
                ticks_right_old = ticks_right;
                first = false;
            }
	ROS_DEBUG_STREAM("Ticks left: " << ticks_left << " ticks right:  " << ticks_right);

        //********************************************************************************//
        // Overflow detection
        //********************************************************************************//
            if (fabs(ticks_left - ticks_left_old) > 0.5*max_encoder)
            { // overflow detected left
                if (ticks_left > ticks_left_old)
                {
                    ticks_left_old = ticks_left_old + max_encoder;
                }
                else ticks_left_old = ticks_left_old - max_encoder;
            }
            if (fabs(ticks_right - ticks_right_old) > 0.5*max_encoder)
            { // overflow detected right
                if (ticks_right > ticks_right_old)
                {
                    ticks_right_old = ticks_right_old + max_encoder;
                }
                else ticks_right_old = ticks_right_old - max_encoder;
            }
            // end overflow detection

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

            ticks_left_old = ticks_left;
            ticks_right_old = ticks_right;

            d = ( d_left + d_right ) / 2 ;
             t = ( d_right - d_left ) / wheel_distance;


            //********************************************************************************//
            // Calculating the Odometry
            //********************************************************************************//

                    //Moving forward
                    if(velocity_left == velocity_right) {
                    velocity = d / d_time;
                    double tx = state->getX()+velocity*cos(state->getTh())*d_time;
                    double ty = state->getY()+velocity*sin(state->getTh())*d_time;

                    state->setVX((tx-state->getX())/d_time);
                    state->setVY((ty-state->getY())/d_time);
                    state->setVTh(0);
                    state->setX(tx);
                    state->setY(ty);

    

                  } else {
                        angular_velocity = t / d_time;
                        radius = d / t;

                        //Turning in place
                        if(velocity_left == -velocity_right) {
                          state->setVX(0);
                          state->setVY(0);
                          double tth = state->getTh() + t ;
                          state->setVTh((tth-state->getTh())/d_time);
                          state->setTh(tth);


                        //Other Movements
                } else {
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
                }
            }
        }


        //********************************************************************************//
        // Publishing
        //********************************************************************************//


        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state->getTh());

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = state->getX()/1000.0;
        odom_trans.transform.translation.y = state->getY()/1000.0;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        state->odom_broadcaster.sendTransform(odom_trans);


        //next, we'll publish the odometry message over ROS
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = state->getX()/1000.0;
        odom.pose.pose.position.y = state->getY()/1000.0;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = state->getVX()/1000.0;
        odom.twist.twist.linear.y = state->getVY()/1000.0;
        odom.twist.twist.angular.z = state->getVTh();

        //publish the message
        state->odom_pub.publish(odom);

        last_time = current_time;

        rate.sleep();
    }
}
