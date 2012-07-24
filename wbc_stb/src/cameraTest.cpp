#include <wbc_stb/udp_osi.h>
#include <wbc_stb/cs8c_interface.h>
#include <wbc_stb/ndtypes.h>
#include <wbc_stb/vis_interface.h>
#include <err.h>
#include <cstdio>

using namespace std;

int main(int argc, char** argv) {

  Position3d rawCamData[10];
  robotinfo visinfo;
  int bytes;
  int count=1;;
  
  //EDIT ME to actual camera address
  char local_cam_port [10];
  sprintf(local_cam_port, "%d", CLIENT_PORT+2);
  char cam_port [10];
  sprintf(cam_port, "%d", CS8C_PORT+2);
  UdpOSI cameraUDP(local_cam_port, "127.0.0.1", cam_port, 0);

  bytes = cameraUDP.recvPacket((char*)rawCamData, sizeof(rawCamData));
  if (bytes<1) {
    fprintf(stderr, "Error receiving data from camera");
    return 0;
  }
  for (size_t ii(0); ii < bytes/sizeof(Position3d); ++ii) {
    cout << "Marker "<< ii << ": " << rawCamData[ii].x  << ", " << rawCamData[ii].y << ", " << rawCamData[ii].z << endl;
  }
  cout <<"Recieved initial camera data\n";

  while(1) {
    bytes = cameraUDP.recvPacket((char*)rawCamData, sizeof(rawCamData));
    if (bytes <1) {
      fprintf(stderr,"Error: no camera data recieved");
      return 0;
    }  
    
    if (count > 100) {
      for (size_t ii(0); ii < bytes/sizeof(Position3d); ++ii) {
	cout << "Marker "<< ii << ": " << rawCamData[ii].x  << ", " << rawCamData[ii].y << ", " << rawCamData[ii].z << endl;
      }
      count = 0;
    }
    
    //Camera out
    for (size_t ii(0); ii<6 ; ++ii) {
      visinfo.position[ii] = ii;
    }
    visinfo.R = 0.1;
    visinfo.T = 0.4;
    bytes = cameraUDP.sendPacket((char*)&visinfo, sizeof(visinfo));
    count++;
  }
  
}
