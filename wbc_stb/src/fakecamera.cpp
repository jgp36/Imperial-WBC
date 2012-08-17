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

  int datapts = 5;
  Position3d p3dData[datapts];
  /*for (size_t ii(0); ii< datapts; ++ii) {
    p3dData[ii].x = 47.5;
    p3dData[ii].y = 5;    
    p3dData[ii].z = 43.5;
  }*/
    p3dData[0].x = 47.5;
    p3dData[0].y = 5;    
    p3dData[0].z = 55;
    p3dData[1].x = 52.5;
    p3dData[1].y = 5;    
    p3dData[1].z = 55;
    p3dData[2].x = 47.5;
    p3dData[2].y = 10;    
    p3dData[2].z = 55;
    p3dData[3].x = 42.5;
    p3dData[3].y = 5;    
    p3dData[3].z = 55;
    p3dData[4].x = 47.5;
    p3dData[4].y = 0;    
    p3dData[4].z = 55;
  int bytes = 0;
  int count = 1;
  while(ros::ok()) {
    
    bytes = camUDP.sendPacket((char*)p3dData, sizeof(p3dData));
    if (bytes != 0 ) {
      fprintf(stderr,"Failed to send cam data");
      return 0;
    }
  }
}
