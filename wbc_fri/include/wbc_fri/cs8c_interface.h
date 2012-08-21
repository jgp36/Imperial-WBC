/**@file cs8c_interface.h
 *
 * File defining the 'remote control' UDP interface for the Staubli CS8C/TX90.
 *
 * Once the CS8C has been booted, it will broadcast either cartesian or joint
 * feedback packets, depending on whether the user has set either cartesian or
 * joint control using the CS8C telnet interface. The feedback packets will be
 * sent to the client IP and port addresses defined within this file.
 *
 * The robot can be controlled by responding to the feedback packets with a
 * command packet of either joint velocity, joint position, cartesian velocity
 * or cartesian position. The CS8C telnet interface must be used to tell the
 * CS8C which type of commands it should accept, and it will ignore any packet
 * which is not the same.
 *
 * Once the CS8C receives a command packet, checks will be performed to ensure
 * safe operation of the robot. The strictness of these checks can be modified
 * using the CS8C telnet interface.
 *
 * NOTE:
 *  - All cartesian values refer to the distal coordinate frame at the centre of
 *     the spherical wrist.
 *  - Zero position for joints are standard for Puma 560 (this is NOT the
 *     Staubli convention).
 *  - The joint limits will NOT completely prevent the robot from colliding with
 *     the walls or floor, the user must avoid these themselves. This is
 *     particularly important with regards to the biomechanics fixtures in the
 *     centre of the workspace.
 *
 * @author Stuart Bowyer
 * @date 19 June 2012
 * @version 3.0
 *
 * *****************************************************************************
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * *****************************************************************************
 */

#ifndef _CS8C_INTERFACE_H_
#define _CS8C_INTERFACE_H_


// Accomodate different locations for int32_t and definitions of inline
#if defined( WIN32 )
 typedef __int32 int32_t;
 typedef unsigned __int32 uint32_t;
 #define INLINE __inline
#elif defined( VXWORKS )
 #include <vxworks.h>
 #define INLINE inline
#else
 #include <stdint.h>
 #define INLINE inline
#endif


#define CS8CI_VERSION "3.0"


// Lengths for packet vectors
#define CART_POS_LEN 7
#define CART_VEL_LEN 6
#define JNT_POS_LEN  6
#define JNT_VEL_LEN  6 
#define FT_LEN       6


// Packet ID numbers (pktID) for communication structures
static const int32_t CART_VEL_CMD_ID = 14928453;
static const int32_t JNT_VEL_CMD_ID =  29458644;
static const int32_t CART_POS_CMD_ID = 30698473;
static const int32_t JNT_POS_CMD_ID =  40593850;
static const int32_t CART_FBK_ID =     51002373;
static const int32_t JNT_FBK_ID =      68645641;


// Network addressing
// IP and port to which the the client should send commands
static const char*            CS8C_IPADDR =     "155.198.64.28";
static const unsigned short   CS8C_PORT =       57271;
// IP and port to which the CS8C will send feedback
static const char*            CLIENT_IPADDR =   "172.31.0.123";
static const unsigned short   CLIENT_PORT =     57272;


////////////////////////////////////////////////////////////////////////////////
// SAFETY PROPERTIES ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Error numbers for the robot feedback packets (OR together for complete ERRNO)
static const int32_t ERRNO_OK =           0;    //< Operating as commanded
static const int32_t ERRNO_INVALID_CMD =  1;    //< No valid commmands recvd
static const int32_t ERRNO_JOINT_LIMS =   2;    //< Joints outside limit
static const int32_t ERRNO_JOINT_VELS =   4;    //< Joints too fast
static const int32_t ERRNO_JOINT_ACCEL =  8;    //< Joints acceleration to high
static const int32_t ERRNO_JOINT_TORUES = 16;   //< Joint loads too high


// Joint position limits (radians) [J3 keeps Robot as elbow-up]
// Enforced when in LIMITED/FULL safety checks
static const double JNT_POS_MAX[6] = { 0.61, 0.0, 3.97, 4.55, 2.33, 4.55 };
static const double JNT_POS_MIN[6] = { -0.61,-1.66, 1.66,-4.55,-1.95,-4.55 };

// Joint velocity limit (radians/second)
// Enforced when in LIMITED/FULL safety checks
static const double JNT_VEL_MAX[6] = { 3.0, 3.0, 3.0, 6.0, 6.0, 6.0 };

// Joint acceleration limit (radians/second/second)
// Enforced when in FULL safety checks
static const double JNT_ACCEL_MAX[6] = { 8.0, 8.0, 8.0, 8.0, 8.0, 8.0 };

// The maximum age (in seconds) permitted for a command to be applied
// to the robot (i.e. the robot will stop if the mirror number
// identifies that the command is based on feedback issued too long
// ago). This value also encapsulates a limit on the number of packets
// which can be lost (i.e. if no new command is received, the robot
// will repeat the last command, up until the command is too old, then
// it will stop).
static const double CMD_AGE_LIMIT = 0.2;


////////////////////////////////////////////////////////////////////////////////
// COMMAND PACKETS /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Standard header of information used in command packet structures
#define CMD_PKT_HDR \
   int32_t pktID; \
   int32_t pktNo; \
   int32_t pktMirror;

// CARTESIAN VELOCITY COMMAND structure
// (i.e. cartesian velocity control commands to the CS8C)
typedef struct _CS8CCartVelCmd {
   CMD_PKT_HDR
   int32_t cartVel[CART_VEL_LEN];
} CS8CCartVelCmd;


// JOINT VELOCITY COMMAND structure
// (i.e. joint velocity control commands to the CS8C)
typedef struct _CS8CJntVelCmd {
   CMD_PKT_HDR
   int32_t jntVel[JNT_VEL_LEN];
} CS8CJntVelCmd;

// CARTESIAN POSITION COMMAND structure
// (i.e. cartesian position control commands to the CS8C)
typedef struct _CS8CCartPosCmd {
   CMD_PKT_HDR
   int32_t cartPos[CART_POS_LEN];
} CS8CCartPosCmd;


// JOINT POSITION COMMAND structure
// (i.e. joint position control commands to the CS8C)
typedef struct _CS8CJntPosCmd {
   CMD_PKT_HDR
   int32_t jntPos[JNT_POS_LEN];
} CS8CJntPosCmd;


// Abstract command packet for easier manipulation within the controller
typedef union _CS8CAbsCmd {
   CS8CCartVelCmd    cartVel;
   CS8CJntVelCmd     jntVel;
   CS8CCartPosCmd    cartPos;
   CS8CJntPosCmd     jntPos;
} CS8CAbsCmd;



////////////////////////////////////////////////////////////////////////////////
// FEEDBACK PACKETS ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Standard header of information used in feedback packet structures
#define FBK_PKT_HDR \
   CMD_PKT_HDR \
   int32_t cs8cErrno;

// CARTESIAN FEEDBACK structure
// (i.e. cartesian feedback from the CS8C)
typedef struct _CS8CCartFbk {
   FBK_PKT_HDR
   int32_t cartPos[CART_POS_LEN];
   int32_t cartVel[CART_VEL_LEN];
   int32_t ft[FT_LEN];
} CS8CCartFbk;


// JOINT FEEDBACK structure
// (i.e. joint control feedback from the CS8C)
typedef struct _CS8CJntFbk {
   FBK_PKT_HDR
   int32_t jntPos[JNT_POS_LEN];
   int32_t jntVel[JNT_VEL_LEN];
   int32_t ft[FT_LEN];
} CS8CJntFbk;



////////////////////////////////////////////////////////////////////////////////
// CONVERSION FUNCTIONS ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static const double PKT2UNIT = 1e-6;   //< packet to unit
static const double PKT2MUNIT = 1e-3;  //< packet to milli-unit
static const double UNIT2PKT = 1e6;    //< unit to packet
static const double MUNIT2PKT = 1e3;   //< mili-unit to packet

/** Convert packet arrays (micro) into unit decimals. */
static INLINE void pkt2Unit( int32_t* pkt_int, double* unit_dbl, int len ) {
   int i;
   for ( i = 0; i < len; ++i ) unit_dbl[i] = (double)pkt_int[i] * PKT2UNIT;
}

/** Convert packet arrays (micro) into milli-unit decimals. */
static INLINE void pkt2MilliUnit( int32_t* pkt_int, double* mili_dbl, int len ) {
   int i;
   for ( i = 0; i < len; ++i ) mili_dbl[i] = (double)pkt_int[i] * PKT2MUNIT;
}

/** Convert unit decimals into packet arrays (micro). */
static INLINE void unit2Pkt( double* unit_dbl, int32_t* pkt_int, int len ) {
   int i;
   for ( i = 0; i < len; ++i ) pkt_int[i] = (int32_t)(unit_dbl[i] * UNIT2PKT);
}

/** Convert milli-unit decimals into packet arrays (micro). */
static INLINE void milliUnit2Pkt( double* mili_dbl, int32_t* pkt_int, int len ) {
   int i;
   for ( i = 0; i < len; ++i ) pkt_int[i] = (int32_t)(mili_dbl[i] * MUNIT2PKT);
}


////////////////////////////////////////////////////////////////////////////////
// OTHER USEFUL BITS ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Transformation from the tip of the pointer tool to the centre of the robot's
// spherical wrist
//
//    ===|XXX|===========O
//      |||||||    <-----+
//      |||T|||    Z     |
//  Z ^ |||X|||          |
//    | |||9|||          v  Y 
//    | |||0|||   
//    +----->
//          Y
//
static const double TOOL_HTM[4][4] = {
      { -1.0,  0.0,  0.0, 0.0 },
      {  0.0,  0.0, -1.0, 193.0 },
      {  0.0, -1.0,  0.0, 121.0 },
      {  0.0,  0.0,  0.0, 1.0 } };


#endif // _CS8C_INTERFACE_H_


