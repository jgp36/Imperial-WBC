#include <ros/ros.h>
#include <err.h>
#include <wbc_stb/udp_osi.h>
#include <wbc_stb/cs8c_interface.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "wbc_fakerobot");
  ros::NodeHandle node("~");

  char local_port [10];
  char control_port [10];
  sprintf(control_port, "%d", CLIENT_PORT);
  sprintf(local_port, "%d", CS8C_PORT);
  UdpOSI robotUDP( local_port, "127.0.0.1", control_port, 0);

  //Wait for an initial message
  CS8CJntVelCmd command;
  int bytes = 0;
  while (bytes < 1) {
    bytes = robotUDP.recvPacket((char*)&command, sizeof(command));
    if (command.pktID != JNT_VEL_CMD_ID) {
      fprintf(stderr, "Error: wrong command packet. Switch to joint commands");
      return 0;
    }
    for (size_t ii(0); ii < JNT_VEL_LEN; ++ii) {
      std::cout << "Command" << ii << ": " << command.jntVel[ii] << "\n";   
    }
  }

  CS8CJntFbk states;
  states.pktID = JNT_FBK_ID;
  for (size_t ii(0); ii< JNT_POS_LEN; ++ii) {
    states.jntPos[ii] = ii*0.1*UNIT2PKT;
    states.jntVel[ii] = 0;
  }

  //UDP robot out
  bytes = robotUDP.sendPacket((char*)&states, sizeof(states));
  if (bytes != 0) {
    fprintf(stderr, "Failed to send robot UDP");
    return 0;
  }

  float dt = 0.0001;

  while(ros::ok()) {

    //UDP robot out
    bytes = robotUDP.sendPacket((char*)&states, sizeof(states));
    if (bytes != 0) {
      fprintf(stderr, "Failed to send robot UDP");
      return 0;
    }

    //UDP robot in    
    bytes = robotUDP.recvPacket((char*)&command, sizeof(command));
    if (command.pktID != JNT_VEL_CMD_ID) {
      fprintf(stderr, "Error: wrong command packet. Switch to joint commands");
      return 0;
    }
    for (size_t ii(0); ii < JNT_VEL_LEN; ++ii) {
      states.jntPos[ii] = states.jntPos[ii] + command.jntVel[ii]*dt;
      states.jntVel[ii] = command.jntVel[ii];
      std::cout << "Command" << ii << ": " << command.jntVel[ii]*PKT2UNIT << "\n";   
    }
  }
}
