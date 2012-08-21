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

  int datapts = 9;
  Position3d p3dData[datapts];
    p3dData[0].x = 0.375*1e2;
    p3dData[0].y = 0.0*1e2;    
    p3dData[0].z = 1.0*1e2;
    p3dData[1].x = 0.425*1e2;
    p3dData[1].y = 0.0*1e2;    
    p3dData[1].z = 1.0*1e2;
    p3dData[2].x = 0.375*1e2;
    p3dData[2].y = 0.05*1e2;    
    p3dData[2].z = 1.0*1e2;
    p3dData[3].x = 0.325*1e2;
    p3dData[3].y = 0.05*1e2;    
    p3dData[3].z = 1.00*1e2;
    p3dData[4].x = 0.425*1e2;
    p3dData[4].y = 0.05*1e2;    
    p3dData[4].z = 1.00*1e2;
    p3dData[5].x = 0.325*1e2;
    p3dData[5].y = 0.0*1e2;    
    p3dData[5].z = 1.00*1e2;
    p3dData[6].x = 0.325*1e2;
    p3dData[6].y = -0.05*1e2;    
    p3dData[6].z = 1.00*1e2;
    p3dData[7].x = 0.425*1e2;
    p3dData[7].y = -0.05*1e2;    
    p3dData[7].z = 1.00*1e2;
    p3dData[8].x = 0.375*1e2;
    p3dData[8].y = -0.05*1e2;    
    p3dData[8].z = 1.00*1e2;
  int bytes = 0;
  double begin = ros::Time::now().toSec();
  while(ros::ok()) {
    
    bytes = camUDP.sendPacket((char*)p3dData, sizeof(p3dData));
    if (bytes != 0 ) {
      fprintf(stderr,"Failed to send cam data");
      return 0;
    }

    double amp = 0.05*1e2;
    double osc = sin((ros::Time::now().toSec()-begin)/4);
    p3dData[0].x = 0.375*1e2;
    p3dData[0].y = 0.0*1e2;
    p3dData[0].z = 1.0*1e2+amp*osc;
    p3dData[1].x = 0.425*1e2;
    p3dData[1].y = 0.0*1e2;   
    p3dData[1].z = 1.0*1e2+amp*osc;
    p3dData[2].x = 0.375*1e2;
    p3dData[2].y = 0.05*1e2;    
    p3dData[2].z = 1.0*1e2+amp*osc;
    p3dData[3].x = 0.325*1e2;
    p3dData[3].y = 0.05*1e2;    
    p3dData[3].z = 1.00*1e2+amp*osc;
    p3dData[4].x = 0.425*1e2;
    p3dData[4].y = 0.05*1e2;    
    p3dData[4].z = 1.00*1e2+amp*osc;
    p3dData[5].x = 0.325*1e2;
    p3dData[5].y = 0.0*1e2;   
    p3dData[5].z = 1.00*1e2+amp*osc;
    p3dData[6].x = 0.325*1e2;
    p3dData[6].y = -0.05*1e2;
    p3dData[6].z = 1.00*1e2+amp*osc;
    p3dData[7].x = 0.425*1e2;
    p3dData[7].y = -0.05*1e2;
    p3dData[7].z = 1.00*1e2+amp*osc;
    p3dData[8].x = 0.375*1e2;
    p3dData[8].y = -0.05*1e2;
    p3dData[8].z = 1.00*1e2+amp*osc;

  }
}
