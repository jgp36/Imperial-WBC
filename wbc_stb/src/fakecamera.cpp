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

  int datapts = 56;
  Position3d p3dData[datapts];
  /*for (size_t ii(0); ii< datapts; ++ii) {
    p3dData[ii].x = 47.5;
    p3dData[ii].y = 5;    
    p3dData[ii].z = 43.5;
  }*/

    for (size_t ii(0); ii<8; ++ii) {
        for (size_t jj(0); jj<7; ++jj) {
           p3dData[ii*7+jj].x = (0.35+0.05*ii)*1e2;
           p3dData[ii*7+jj].y = (-0.15+0.05*jj)*1e2;    
           p3dData[ii*7+jj].z = (0.24+pow(0.35+0.05*ii-0.40,2))*1e2;
       }
    }


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
