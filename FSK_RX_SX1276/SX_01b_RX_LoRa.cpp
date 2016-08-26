/*  
 *  LoRa 868 / 915MHz SX1272 LoRa module
 *  
 *  Copyright (C) Libelium Comunicaciones Distribuidas S.L. 
 *  http://www.libelium.com 
 *  
 *  This program is free software: you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License as published by 
 *  the Free Software Foundation, either version 3 of the License, or 
 *  (at your option) any later version. 
 *  
 *  This program is distributed in the hope that it will be useful, 
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License 
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *  
 *  Version:           1.2
 *  Design:            David Gascón 
 *  Implementation:    Covadonga Albiñana, Victor Boria, Ruben Martin
 */
 
// Include the SX1272 and SPI library: 
#include "arduPiLoRa.h"

int e;
char my_packet[100];
int sender2 =0;
int sender1 =0;

int count=0;

void setup()
{
  // Print a start message
  printf("SX1272 module and Raspberry Pi: receive packets without ACK\n");
  
  // Power ON the module
  e = sx1272.ON();
  printf("Setting power ON: state %d\n", e);
  
  // Set transmission mode
  
  
  // Set header
  
  
  // Select frequency channel
  e |= sx1272.setChannel(CH_10_868);  // this funcction will override the values in the initialisation registers 
  printf("Setting Channel: state %d\n", e);
  
  // Set CRC
  e |= sx1272.setCRC_ON();
  printf("Setting CRC ON: state %d\n", e);
  
  // Select output power (Max, High or Low)
  e |= sx1272.setPower('M');
  printf("Setting Power: state %d\n", e);
  
  // Set the node address
  e |= sx1272.setNodeAddress(44);
  printf("Setting Node address: state %d\n", e);
  //sx1272.getPayloadLength();
  // Print a success message
  if (e == 0)
    printf("SX1272 successfully configured\n");
  else
    printf("SX1272 initialization failed\n");

  delay(1000);
}

void loop(void)
{
  // Receive message
   printf("====================================\n");
   printf("Start main program                 ||\n");
   printf("====================================\n");
   printf("Please wait 10 seconds, we are searching to hear something during this time ... \n");
  //sx1272.getChannel();
  //sx1272.getPreambleLength();
  //sx1272.getPayloadLength();
  e = sx1272.receivePacketTimeout(10000);
  
  if ( e == 0 )
  {
    printf("Receive packet, state %d\n",e);

    //for (unsigned int i = 0; i < sx1272.packet_received.length; i++)
   // {
     // my_packet[i] = (char)sx1272.packet_received.data[i];
      count++;
   // }
    //printf("Message: %s\n", my_packet);
		
		   
  }
  else {
	  
	     if(count > 0)
	     {
			 printf("             ***************************************************\n");
			 printf("             *  Number of packets received till now: %d       *\n",count);
			 printf("             ***************************************************\n");
			 
		 }
		 else
		 {
			 printf("Receive packet, state %d : Nothing found yet\n",e); 
		 }
	  
    
    
  }
}

int main (){
	setup();
	while(1){
		loop();
	}
	return (0);
}
