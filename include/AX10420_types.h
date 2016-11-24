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

#ifndef _ax10420_types_h_
#define _ax10420_types_h_

/**
 * Data types for character device communication
 */

typedef unsigned tAXMsgType;
typedef unsigned tAXGroup;
typedef unsigned tAXPort;
typedef unsigned tAXIOConfigure;

/* control message types */
enum  {tInit, tSetOutput};
/* control groups */
enum  {eG1,eG2};
/* control ports */
enum  {ePA,ePB,ePC};
/* io directions */
enum  {eOut,eIn};

/**
 * This message represents a new device initialisation.
 * It is used to set the I/O modes of the device.
 */
typedef struct
{
    tAXGroup group;
    tAXIOConfigure port_a;
    tAXIOConfigure port_b;
    tAXIOConfigure port_c_upper;
    tAXIOConfigure port_c_lower;
} AX10420_msg_init;

/**
 * This message is used to set the output value of a port.
 */
typedef struct
{
    tAXGroup group;
    tAXPort port;
    unsigned value;
} AX10420_msg_setOutput;

/**
 * This message is used to represent the state of all groups and ports.
 */
typedef struct
{
    tAXIOConfigure port1_a;
    tAXIOConfigure port1_b;
    tAXIOConfigure port1_c;
    tAXIOConfigure port2_a;
    tAXIOConfigure port2_b;
    tAXIOConfigure port2_c;
} AX10420_msg_state;

/**
 * This message is used from the userland to send commands to the driver.
 */
typedef struct
{
    tAXMsgType type;
    union
    {
        AX10420_msg_init msginit;
        AX10420_msg_setOutput msgsetoutput;
    };
} AX10420_msg;



#endif /* _ax10420_types_h_ */
