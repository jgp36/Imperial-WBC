/**@file udp_osi.cxx
 *
 * Function implementations for operating system independent UDP.
 *
 * @author Stuart Bowyer
 * @date 20 March 2012

 * modified by Josh Petersen
 * for linux use in WBC

 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <signal.h>
#include <wbc_fri/udp_osi.h>


UdpOSI::UdpOSI( char* local_port, char* target_ip, char* target_port, int blocking )
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



int UdpOSI::sendPacket( char* data, int data_len )
{
   int bytes_sent = send( m_socket, data, data_len, 0 );
   if ( bytes_sent != data_len )
   {
      std::cout << "Bytes sent not equal to packet length" << std::endl;
      return 1;
   }
   return 0;
}


int UdpOSI::recvPacket( char* buffer, int buffer_len )
{
   int bytes_recv = recv( m_socket, buffer, buffer_len, 0 );

   if ( bytes_recv < 0 )   return 0;
   else                    return bytes_recv;
}


int UdpOSI::createSocket()
{
   m_socket = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );

   if ( m_socket == -1 )
   {
      m_socket = 0;
      std::cout << "ERROR: Socket creation failed" << std::endl;
      return 1;
   }

   return 0;
}


int UdpOSI::bindSocket( char* local_port )
{
   // Create local address structure
  struct addrinfo hints;
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE;

  getaddrinfo(NULL, local_port, &hints, &m_local_sockaddr);

   // Bind
  int bind_status = bind( m_socket, m_local_sockaddr->ai_addr, m_local_sockaddr->ai_addrlen );

   if ( bind_status != 0 )
   {
      std::cout << "ERROR: Socket bind failed"  << std::endl;
      return 1;
   }
   return 0;
}


int UdpOSI::nonBlocking()
{
  /*
   u_long nb_on = 1;
   int nb_status = ioctlsocket( m_socket, FIONBIO, &nb_on );

   if ( nb_status != 0 )
   {
      std::cout << "ERROR: Socket non-blocking set failed" << std::endl;
      return 1;
      }*/
   return 0;
}


int UdpOSI::connectSocket( char* target_ip, char* target_port )
{
   // Create target address structure  
  struct addrinfo hints;
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;

  getaddrinfo(target_ip, target_port, &hints, &m_target_sockaddr);

   // Connect
  int connect_status = connect( m_socket, m_target_sockaddr->ai_addr, m_target_sockaddr->ai_addrlen);

   if ( connect_status != 0 )
   {
      std::cout << "ERROR: Socket connect failed" << std::endl;
      return 1;
   }
   return 0;
}


int UdpOSI::closeSocket()
{
   if ( m_socket == 0 ) return 0;

   int close_status = close(m_socket);

   if ( close_status != 0 )
   {
      std::cout << "ERROR: Socket close failed" << std::endl;
      return 1;
   }
   m_socket = 0;
   return 0;
}
