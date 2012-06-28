#include <ros/ros.h>
#include <err.h>
#include <iostream>
#include <wbc_stb/udp_osi.h>
#include <wbc_stb/cs8c_interface.h>
#include <wbc_stb/ndtypes.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "wbc_fakecamera");
  ros::NodeHandle node("~");

  char local_port [10];
  char cam_port [10];
  sprintf(local_port, "%d", CS8C_PORT+2);
  sprintf(cam_port, "%d", CLIENT_PORT+2);
  UdpOSI camUDP( local_port, "127.0.0.1", cam_port, 0);

  int datapts = 1;
  Position3d p3dData[datapts];
  for (size_t ii(0); ii< datapts; ++ii) {
    p3dData[ii].x = ii+10;
    p3dData[ii].y = ii+20;    
    p3dData[ii].z = ii+30;
  }
  int bytes = 0;
  while(1) {
    
    bytes = camUDP.sendPacket((char*)p3dData, sizeof(p3dData));
    if (bytes != 0 ) {
      fprintf(stderr,"Failed to send cam data");
      return 0;
    }

  }
}
