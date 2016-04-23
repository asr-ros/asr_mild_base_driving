#ifndef _hardware_UseAX10420_h_
#define _hardware_UseAX10420_h_

#include "AX10420_types.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

/**
Creates an open file description that refers to a the device.
Which has read and write access.

@param *device_name path to device file
*/
inline int AX10420_OpenDevice(const char *device_name)
{
    int fd;
    fd = open(device_name, O_RDWR);
    if (fd < 0)
    {
        ROS_ERROR("error opening the ax10420 device: %s\n",strerror(errno));
        return -1;
    }

    return fd;
};

/**
Init the ports of the device.

@param fd shows if device open. 1 = device open.
@param groub control groups.
@param port ports of device.
*/
inline int AX10420_Init(int fd, tAXGroup group,
                        tAXIOConfigure port_a, tAXIOConfigure port_b,
                        tAXIOConfigure port_c_upper, tAXIOConfigure port_c_lower)
{
    int len;
    if (fd < 0)
    {
        ROS_ERROR("AX10420_Init invalid fd:%x\n",fd);
        return -1;
    }

    AX10420_msg_init init;
    init.group = group;
    init.port_a = port_a;
    init.port_b = port_a;
    init.port_c_upper = port_c_upper;
    init.port_c_lower = port_c_lower;

    AX10420_msg msg;
    msg.type = tInit;
    msg.msginit = init;

    len = write(fd, &msg, sizeof(AX10420_msg));

    if (len>0)
    {
        return 0;
    }
    else
    {
        ROS_ERROR("AX10420_Init write failed: %s\n",strerror(errno));
        return -errno;
    }

    return -1;
}

/**
Get Input from spezific group and port.

@param fd shows if device open. 1 = device open.
@param groub control groups.
@param port ports of device.
*/
inline int AX10420_GetInput(int fd, tAXGroup group, tAXPort port)
{
    int len;
    if (fd < 0)
    {
        ROS_ERROR("AX10420_GetInput invalid fd:%x\n",fd);
        return -1;
    }

    AX10420_msg_state state;
    len = read(fd, &state, sizeof(AX10420_msg_state));

    if (len > 0)
    {
        switch (group)
        {
        case eG1:
            switch (port)
            {
            case ePA:
                return state.port1_a;
                break;
            case ePB:
                return state.port1_b;
                break;
            case ePC:
                return state.port1_c;
                break;
            default:
                return -1;
                break;
            };
            break;
        case eG2:
            switch (port)
            {
            case ePA:
                return state.port2_a;
                break;
            case ePB:
                return state.port2_b;
                break;
            case ePC:
                return state.port2_c;
                break;
            default:
                return -1;
                break;
            };
            break;
        default:
            return -1;
            break;
        }
    }
    else
    {
        ROS_ERROR("AX10420_GetInput read failed: %s\n",strerror(errno));
        return -errno;
    }

    return -1;
}

/**
Set an Output to a port and group, with a spezific value.

@param fd shows if device open. 1 = device open.
@param groub control groups.
@param port ports of device.
@param value that you want send.
*/
inline int AX10420_SetOutput(int fd, tAXGroup group, tAXPort port, unsigned value)
{
    int len;
    if (fd < 0)
    {
        ROS_ERROR("AX10420_SetOutput invalid fd:%x\n",fd);
        return -1;
    }

    AX10420_msg_setOutput set;
    set.group = group;
    set.port = port;
    set.value = value;

    AX10420_msg msg;
    msg.type = tSetOutput;
    msg.msgsetoutput = set;

    len = write(fd, &msg, sizeof(AX10420_msg));

    if (len>0)
    {
        return 0;
    }
    else
    {
        ROS_ERROR("AX10420_SetOutput write failed: %s\n",strerror(errno));
        return -errno;
    }

    return -1;

}

#endif /* _hardware_UseAX10420_h_ */
