#include <ros/ros.h>
#include <err.h>
#include <wbc_stb/udp_osi.h>
#include <wbc_stb/cs8c_interface.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "wbc_robottest");
  ros::NodeHandle node("~");

  char local_port [10];
  sprintf(local_port, "%d", CLIENT_PORT);
  char robot_port [10];
  sprintf(robot_port, "%d", CS8C_PORT);
  UdpOSI robotUDP( local_port, const_cast<char*>(CS8C_IPADDR), robot_port, 0);
  //UdpOSI robotUDP( local_port, "127.0.0.1", robot_port, 0);


  //Initial message
  CS8CJntVelCmd robotCmd;
  for (size_t ii(0); ii < JNT_VEL_LEN; ++ii) {
    robotCmd.jntVel[ii] = 0;
  }
  robotCmd.pktID = JNT_VEL_CMD_ID;
  int bytes = robotUDP.sendPacket((char*)&robotCmd, sizeof(robotCmd));
  if (bytes != 0) {
    fprintf(stderr, "Failed to send robot UDP");
    return 0;
  }
  std::cout <<"Sent initial command packet\n";

  //Wait for a response
  CS8CJntFbk robotFb;
  bytes = 0;
  while (bytes <1){
    bytes = robotUDP.recvPacket((char*)&robotFb, sizeof(robotFb));
  }
  if (robotFb.cs8cErrno != 0) {
    fprintf(stderr, "Error with feedback packet: %d",robotFb.cs8cErrno);
    //return 0;
  }
  if (robotFb.pktID != JNT_FBK_ID) {
    fprintf(stderr, "Error: wrong feedback packet. Switch to joint feedback");
    return 0;
  }
  for (size_t ii(0); ii < JNT_POS_LEN; ++ii) {
    std::cout << "J" << ii << " position: " << robotFb.jntPos[ii] << "\n";    
    std::cout << "J" << ii << " velocity: " << robotFb.jntVel[ii] << "\n";
  }
  while(1) {

   //UDP robot out
    robotCmd.pktMirror = robotFb.pktNo;
    bytes = robotUDP.sendPacket((char*)&robotCmd, sizeof(robotCmd));
    if (bytes != 0) {
      fprintf(stderr, "Failed to send robot UDP");
      return 0;
    }

    //UDP robot in
    bytes = robotUDP.recvPacket((char*)&robotFb, sizeof(robotFb));
    if (bytes <1) {
      fprintf(stderr,"Error: no robot states recieved");
      return 0;
    }
    if (robotFb.cs8cErrno != 0) {
      fprintf(stderr, "Error with feedback packet: %d",robotFb.cs8cErrno);
      //return 0;
    }
    if (robotFb.pktID != JNT_FBK_ID) {
      fprintf(stderr, "Error: wrong feedback packet. Switch to joint feedback");
      return 0;
    }
    for (size_t ii(0); ii < JNT_POS_LEN; ++ii) {
      std::cout << "J" << ii << " position: " << robotFb.jntPos[ii] << "\n";    
      std::cout << "J" << ii << " velocity: " << robotFb.jntVel[ii] << "\n";
    }

  }
}
