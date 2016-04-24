#include <sys/socket.h>
#include <linux/sockios.h>
#include <linux/can.h>
#include <linux/if.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <errno.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <boost/thread.hpp>
#include "CanListener.h"
#include "BaseController.h"
#include "RobotState.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Can");
    ros::NodeHandle n;

    //Creating the socket for the CAN-Bus
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, "can0" );
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    int bufsize = 256;
    if(!setsockopt(s, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize))){
        ROS_INFO("Can: Socket set successfully.");
    }else{
        ROS_ERROR("Can: Socket set error.");
    }


    //Creating objects for the other files
    RobotState state(&n, s);
    CanListener canlist(&state);
    BaseController controller(&state);

    boost::thread list(boost::bind(&CanListener::run, &canlist));
    boost::thread contr(boost::bind(&BaseController::run, &controller));

    ros::spin();

    ROS_INFO("Can: Init CanListener and BaseController.");

    list.join();
    contr.join();

    close(s);
    return 0;
}
