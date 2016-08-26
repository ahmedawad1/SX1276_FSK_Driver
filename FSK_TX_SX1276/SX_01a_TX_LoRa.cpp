
 
// Include the SX1272 and SPI library: 
#include "arduPiLoRa.h"

int e;
int count = 0;


void setup()
{
  // Print a start message
  printf("SX1272 module and Raspberry Pi: send packets without ACK\n");
  
  // Power ON the module
  e = sx1272.ON();
  printf("Setting power ON: state %d\n", e);
  
  
  // Select frequency channel
  e |= sx1272.setChannel(CH_10_868);
  printf("Setting Channel: state %d\n", e);
  
  // Set CRC
  // CRC is activated as it is configured in RegPacketConfig1 to be on
  
  // Select output power (Max, High or Low)
  e |= sx1272.setPower('M');
  printf("Setting Power: state %d\n", e);
  
  // Set the node address
  e |= sx1272.setNodeAddress(7);
  printf("Setting Node address: state %d\n", e);
  
  // Print a success message
  if (e == 0)
    printf("SX1272 successfully configured\n");
  else
    printf("SX1272 initialization failed\n");

  delay(1000);
}

void loop(void)
{
	 printf("//////////////////////////Starting new message transmission /////////////////////////\n");
	// Send message1 and print the result
	char message1 [] = "AL"; // message must be less than 251 bytes (Ahmed: max Grenzen gefunden) test 9.8.2016
	//char number[]="";
	
	//sprintf(number,"%d",count);
	count ++;
	//strcat(message1,number);
 
    sx1272.getChannel();
    sx1272.getPreambleLength();
    sx1272.getPayloadLength();
    sx1272.getCRC();
    
    e = sx1272.sendPacketTimeout(44, message1);
    
    printf("Packet sent, state %d\n",e);
    
    delay(1000);
 
 	
}

int main (){
	setup();
	while(1){
		loop();
	if (count==100)
    break;	
	}
	return (0);
}
