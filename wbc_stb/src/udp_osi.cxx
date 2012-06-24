/**@file udp_osi.cxx
 *
 * Function implementations for operating system independent UDP.
 *
 * @author Stuart Bowyer
 * @date 20 March 2012
 */

#include "udp_osi.h"


UdpOSI::UdpOSI( unsigned short local_port, char* target_ip, unsigned short target_port, int blocking )
{
   m_socket = 0;

   std::cout << "Creating UDP communications class..." <<
      "\n\tLocal Port: " << local_port <<
      "\n\tTarget IP: " << target_ip <<
      "\n\tTarget Port: " << target_port << "\n" << std::endl;

   // Create socket
   int create_status = createSocket();
   if ( create_status != 0 ) return;


   // Bind socket to required local port number
   int bind_status = bindSocket( local_port );
   if ( bind_status != 0 ) { closeSocket(); return; }

   
   // Set socket as non-blocking if required
   if ( !blocking )
   {
      int nb_status = nonBlocking();
      if ( nb_status != 0 ) { closeSocket(); return; }
   }


   // Connect to target (though UDP is connectionless, this prevents entering
   // the address each time).
   int connect_status = connectSocket( target_ip, target_port );
   if ( connect_status != 0 ) { closeSocket(); return; }
}


int UdpOSI::recvPurgePacket( char* buffer, int buffer_len )
{
   int last_bytes;
   int bytes_recv = 0;

   do
   {
      last_bytes = bytes_recv;
      bytes_recv = recvPacket( buffer, buffer_len );
   } while ( bytes_recv > 0 );

   return last_bytes;
}


