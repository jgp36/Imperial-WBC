/**@file udp_osi.h
 *
 * An operating system abstract class which operated using UDP.
 *
 * @author Stuart Bowyer
 * @date 20 March 2012
 */

#include <iostream>

#pragma once

class UdpOSI
{
public:

   /**
    * Constructor for a UDP communication channel.
    * @param local_port Port number to communicate through
    * @param target_ip IP address to communicate with
    * @param target_port Port number to communicate with
    * @param blocking Should the port be blocking ( 1=blocking, 0=nonblocking )
    */
   UdpOSI( unsigned short local_port, char* target_ip,
      unsigned short target_port, int blocking );

   /** Destructor which closes the socket etc. */
   ~UdpOSI() { closeSocket(); }

   /**
    * Send data packet.
    * @param data Data array to send to the target
    * @param data_len Number of bytes to send
    * @return Success of the operation (0=success else=fail)
    */
   int sendPacket( char* data, int data_len );

   /**
    * Receive data packet.
    * @param buffer Data array to buffer recieved bytes in
    * @param buffer_len Size of the buffer
    * @return Number of bytes recieved
    */
   int recvPacket( char* buffer, int buffer_len );

   /**
    * Purge all data from the socket and receive the last data packet.
    * @note This should NEVER be used with BLOCKING sockets as it will go
    * on forever.
    * @param buffer Data array to buffer recieved bytes in the last packet
    * @param buffer_len Size of the buffer in the last packet
    * @return Number of bytes recieved in the last packet
    */
   int recvPurgePacket( char* buffer, int buffer_len );

private:

   /**
    * Create the socket which will be used to send data packets.
    * @return Success of the operation (0=success else=fail)
    */
   int createSocket();

   /**
    * Bind the socket to the specified port address on the pre-assigned ip.
    * @param local_port Port number to bind socket to.
    * @return Success of the operation (0=success else=fail)
    */
   int bindSocket( unsigned short local_port );

   /**
    * Set the socket as non-blocking so that recv calls will return if no data
    * is waiting.
    * @return Success of the operation (0=success else=fail)
    */
   int nonBlocking();

   /**
    * Connect to socket to a target.
    * @return Success of the operation (0=success else=fail)
    */
   int connectSocket( char* target_ip, unsigned short target_port );

   /**
    * Close the socket preventing any more packets being sent or received.
    * @return Success of the operation (0=success else=fail)
    */
   int closeSocket();


   // Member variables
   int   m_socket;
   void* m_target_sockaddr;
};


