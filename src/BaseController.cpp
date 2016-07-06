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


int BaseController::initAX10420()
{
    //We use only one cart (nr.0) and use only the first group (eG1).
    //Port A is set to Out (0).
    //Port B is set to In (1).
    //Port C upper and lower is set to In (1).
    int ax10420 = AX10420_OpenDevice("/dev/ax104200");
    int ret = AX10420_Init(ax10420, eG1, 0, 1, 1, 1);
    if (ret!=0)
    {
        ROS_ERROR("BaseController: AX10420_Init() failed, error code %d", ret);
    }
    return ax10420;
}

void BaseController::writeDataToCan(float vleft, float vright,  float max_speed)
{
    unsigned short outputleft;
    unsigned short outputright;
    struct can_frame frame;
    frame.can_id = 0xb;
    frame.can_dlc = 8;
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

bool BaseController::enableMotor(int ax10420)
{
    unsigned char outbyte = 0;
    //Enable motor.
    outbyte |= MOTOR_ENABLE_BIT;
    ROS_DEBUG("BaseController: Motor enable.");
    //Disable emergency stop.
    outbyte |= EMERGENCY_STOP_BIT;
    outbyte |= INDICATOR_BIT;
    ROS_DEBUG("BaseController: Disable emergency stop.");

    //Now put value to port A of Group 1.
    int ret = AX10420_SetOutput(ax10420, eG1, ePA, outbyte);
    if (ret!=0)
    {
        ROS_ERROR("BaseController: AX10420_SetOutput() failed, error code %d", ret);
    }
    return true;
}

float BaseController::calculateVelocity(float speed, float velocity_old)
{
    // 0.3315 = wheel_distance/2, in meter, multiply with speedfaktor.
    float next =  100 * ( cmd.linear.x - (cmd.angular.z*0.3315))*speed;

    //Smoothing the moves. With weighting 4/6.
    return velocity_old * 0.40 + next * 0.60;
}
//This function handles all the mild_base_driving bus/motor/relais board related stuff.
//The AX10420 is the controller for the relais board.
void BaseController::run()
{
    //subscribing to the velocity commands
    ros::Subscriber sub = state->n->subscribe("cmd_vel", 1,
                          &BaseController::setTargetVelocity,
                          this);

    int ax10420 = initAX10420();
    float speed;
    state->n->param("velocity", speed, 1.0f);

    //********************************************************************************//
    // Initialisation
    //********************************************************************************//
    float vleft2 = 0;
    float vright2 = 0;
    float max_speed = 612;
    bool motorEnabled = false;

    ros::Rate rate(50);

    bool first = true;

    //Adaptiation for right/left velocity.
    double right_adapter = 0;
    double left_adapter = 0;

    //Driving loop until node is stopped. Processing velocity commands each time being passed.
    while ( ros::ok() )
    {
        float vleft = 0;
        float vright = 0;

        boost::mutex::scoped_lock scoped_lock(mutex);

        vleft = calculateVelocity(speed, vleft2);
        vright = calculateVelocity(speed, vright2);

        ROS_DEBUG("BaseController: 1. vleft: %f, vright: %f", vleft, vright);

        vleft2 = vleft;
        vright2 = vright;

        if(vright == 0)
        {
            right_adapter = 0;
        }
        if(vleft == 0)
        {
            left_adapter = 0;
        }

        //We got an effective driving command.
        if ((vleft != 0) || (vright != 0))
        {
            //Adapt the velocity linear to the required velocity, if the real velocity is different to the required.
            right_adapter = calculateAdapter(vright, canListener->get_velocity_right(), right_adapter);
            vright += right_adapter;

            left_adapter = calculateAdapter(vleft, canListener->get_velocity_left(), left_adapter);
            vleft += left_adapter;

            ROS_DEBUG("BaseController: 2. vleft: %f, vright: %f", vleft, vright);

            motorEnabled = enableMotor(ax10420);

            vleft = std::min(vleft, max_speed);
            vleft = std::max(vleft, -max_speed);
            vright = std::min(vright, max_speed);
            vright = std::max(vright, -max_speed);

            //We need inverse speeds because of the iaim wiring.
            vleft = -vleft;

            writeDataToCan(vleft, vright, max_speed);
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

double BaseController::calculateAdapter(double required_velocity, double real_velocity, double adapter)
{
    double adapter_value = 0;
    double difference = std::abs(std::abs(required_velocity) - std::abs(real_velocity*100.f));
    if(required_velocity > 0)
    {
        adapter_value = difference/10.f;
    }
    else
    {
        adapter_value = -difference/10.f;
    }
    ROS_DEBUG("BaseController: difference %f, adapter %f", difference, adapter);

    if(std::abs(required_velocity) > std::abs(real_velocity*100.f))
    {
        adapter += adapter_value;
    }
    else
    {
        adapter -= adapter_value;
    }
    if(std::abs(adapter) > std::abs(required_velocity/2))
    {
        adapter = required_velocity/2;
    }
    return adapter;
}

//This function is triggered when a Twist message is received.
void BaseController::setTargetVelocity(const geometry_msgs::Twist &twist)
{
    boost::mutex::scoped_lock scoped_lock(mutex);
    cmd = twist;
}
