




#include "udp_osi.h"
#include "ndtypes.h"
#include <iostream>
#include <stdio.h>
extern "C" {
	#include "sleep.h"
}

void main( int argc, unsigned char *argv[] )
{
	char* TARGET_IP = "127.0.0.1";						//TARGET IP FOR UDP
	unsigned short TARGET_PORT = 57860;					//TARGET PORT FOR UDP
	unsigned short LOCAL_PORT = 50000;
	Position3d p3dData[50];
	

	UdpOSI UDPConnection = UdpOSI( LOCAL_PORT, TARGET_IP, TARGET_PORT, 0 );		//CREATE UDP CONNECTION

	int count = 0;

	while (1) {
		int bytes = UDPConnection.recvPacket((char*)p3dData, sizeof(p3dData));			//int recvPacket( char* buffer, int buffer_len );	len = 96
		if (bytes>1) {
			count++;
			std::cout<<"Package recieved "<<count<<"\t number of bytes "<<bytes<<std::endl;
		}

		int numMarkers = bytes/sizeof(Position3d);

		for (int nCurMarker = 0; nCurMarker < numMarkers; nCurMarker++ ){
				
			fprintf (stdout,"Marker %d : %f %f %f \n",nCurMarker,p3dData[nCurMarker].x,p3dData[nCurMarker].y,p3dData[nCurMarker].z );

		}

		//sleep(1);
	}
	UDPConnection.~UdpOSI();

	while(1) { }
	exit( 0 );
}