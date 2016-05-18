#include <stdlib.h>
#include <sys/socket.h>
#include <linux/sockios.h>
#include <linux/can.h>
#include <linux/if.h>
#include "BaseController.h"
#include "geometry_msgs/Twist.h"
#include "UseAX10420.h"
#include "AX10420_types.h"
#include <algorithm>

/* AX10420 foo */
// Out Bits
#define MOTOR_ENABLE_BIT 2
#define EMERGENCY_STOP_BIT 1
#define INDICATOR_BIT 0x80
// In Bits
#define EMERGENCY_STOP_SCANNER_BIT 1
#define KEY_SWITCH_STATUS_BIT 2
#define EMERGENCY_STOP_BUTTON_BIT 4
#define SCANNER_WEAK_BIT 8
#define EXTRA_BUTTON_BIT 32
#define SCANNER_OSSD2_BIT 0x10

//This function handles all the mild_base_driving bus/motor/relais board related stuff.
//The AX10420 is the controller for the relais board.
void BaseController::run()
{



    //subscribing to the velocity commands
    ros::Subscriber sub = state->n->subscribe("cmd_vel", 1,
                          &BaseController::setTargetVelocity,
                          this);

    int ax10420;
    float speed;
    state->n->param("velocity", speed, 1.0f);

    //We use only one cart (nr.0) and use only the first group (eG1).
    //Port A is set to Out (0).
    //Port B is set to In (1).
    //Port C upper and lower is set to In (1).
    ax10420 = AX10420_OpenDevice("/dev/ax104200");
    int ret = AX10420_Init(ax10420, eG1, 0, 1, 1, 1);
    if (ret!=0)
    {
        ROS_ERROR("BaseController: AX10420_Init() failed, error code %d", ret);
    }


    //********************************************************************************//
    // Initialisation
    //********************************************************************************//
    unsigned char outbyte = 0;
    float vleft2 = 0;
    float vright2 = 0;
    float max_speed = 612;
    unsigned short outputleft;
    unsigned short outputright;
    bool motorEnabled = false;

    ros::Rate rate(50);

    struct can_frame frame;
    frame.can_id = 0xb;
    frame.can_dlc = 8;

    bool first = true;

    //Driving loop until node is stopped. Processing velocity commands each time being passed.
    while ( ros::ok() )
    {
        outbyte = 0;

        float vleft = 0;
        float vright = 0;
        float nextleft = 0;
        float nextright = 0;

        {
            //********************************************************************************//
            //Transforming the velovity commands into differential drive velocities
            //********************************************************************************//
            boost::mutex::scoped_lock scoped_lock(mutex);

            // 0.3315 = wheel_distance/2, in meter, multiply with speedfaktor.
            nextleft =  100 * ( cmd.linear.x -  (cmd.angular.z*0.3315))*speed;
            nextright =   100 * ( cmd.linear.x +  (cmd.angular.z*0.3315))*speed;
        }

        //Smoothing the moves. With weighting 4/6.
        vleft = vleft2 * 0.40 + nextleft * 0.60;
        vright = vright2 * 0.40 + nextright * 0.60;

        ROS_INFO("BaseController: 1. vleft: %f, vright: %f", vleft, vright);

        vleft2 = vleft;
        vright2 = vright;

        ROS_INFO("BaseController: velocity_left: %f, velocity_right: %f", canListener->get_velocity_left(),canListener->get_velocity_right());

        if(vleft > canListener->get_velocity_left()*100){
            vleft += vleft-canListener->get_velocity_left()*100;
        }else{
            vleft -= canListener->get_velocity_left()*100-vleft;
        }

        if(vright > canListener->get_velocity_right()*100){
            vright += vright-canListener->get_velocity_right()*100;
        }else{
            vright -= canListener->get_velocity_right()*100-vright;
        }
        ROS_INFO("BaseController: 2. vleft: %f, vright: %f", vleft, vright);
        //We got an effective driving command.
        if ((vleft != 0) || (vright != 0))
        {

            //Enable motor.
            outbyte |= MOTOR_ENABLE_BIT;
            motorEnabled = true;
            ROS_DEBUG("BaseController: Motor enable.");
            //Disable emergency stop.
            outbyte |= EMERGENCY_STOP_BIT;
            outbyte |= INDICATOR_BIT;
            ROS_DEBUG("BaseController: Disable emergency stop.");

            //Now put value to port A of Group 1.
            ret = AX10420_SetOutput(ax10420, eG1, ePA, outbyte);
            if (ret!=0)
            {
                ROS_ERROR("BaseController: AX10420_SetOutput() failed, error code %d", ret);
            }

            vleft = std::min(vleft, max_speed);
            vleft = std::max(vleft, -max_speed);
            vright = std::min(vright, max_speed);
            vright = std::max(vright, -max_speed);

            ROS_DEBUG("vleft: %f, vright %f", vleft,vright);
            //We need inverse speeds because of the iaim wiring.
            vleft = -vleft;

            //Left.
            outputleft=(unsigned short)(vleft/max_speed*0x7f+0x7f);
            ROS_DEBUG("BaseController: outputleft = %i", outputleft);

            //Right.
            outputright=(unsigned short)(vright/max_speed*0x7f+0x7f);
            ROS_DEBUG("BaseController: outputright = %i", outputright);

            //********************************************************************************//
            //Writing the Velocities in the CAN-Frame
            //********************************************************************************//
            frame.data[0] = outputright & 0xff;
            frame.data[1] = outputright >> 8;
            frame.data[2] = outputleft & 0xff;
            frame.data[3] = outputleft >> 8;

            //Writing on the CAN-Bus.
            write(state->getSocket(), &frame, sizeof(struct can_frame));
        }
        else if (motorEnabled)
        {
            ROS_DEBUG("BaseController: Motor disabled");
            //Disable motor.
            motorEnabled = false;
            AX10420_SetOutput(ax10420, eG1, ePA, 0);
        }
        if(first)
        {
            ROS_INFO("BaseController: Started successfully. With speedup x%f\n", speed);
            first = false;
        }

        rate.sleep();
    }

    //Disable motor and close breaks, because when platform stands with zero velocity commands and open breaks, platform may drift either real or just concerning its sensors values.
    AX10420_SetOutput(ax10420, eG1, ePA, 0);
}

//This function is triggered when a Twist message is received.
void BaseController::setTargetVelocity(const geometry_msgs::Twist &twist)
{
    boost::mutex::scoped_lock scoped_lock(mutex);
    cmd = twist;
}
