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
        ROS_INFO("Can: Socket set successfully.\n");
    }else{
        ROS_ERROR("Can: Socket set error.");
    }


    //Creating objects for the other files
    RobotState state(&n, s);
    CanListener canlist(&state);
    BaseController controller(&state, &canlist);

    boost::thread list(boost::bind(&CanListener::run, &canlist));
    boost::thread contr(boost::bind(&BaseController::run, &controller));

    ros::spin();

    list.join();
    contr.join();

    close(s);
    return 0;
}
