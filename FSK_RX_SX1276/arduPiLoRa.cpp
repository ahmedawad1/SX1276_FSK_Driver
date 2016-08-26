
//**********************************************************************
// Includes
//**********************************************************************
#include "arduPiLoRa.h"

#include "arduPiClasses.h"

//**********************************************************************
// Public functions.
//**********************************************************************

/*
 Function: Sets the module ON.
 Returns: uint8_t setLORA state
*/
uint8_t SX1272::ON()
{

  uint8_t state = 2;
 
  #if (SX1272_debug_mode > 1)
	  printf("\n");
	  printf("Starting 'ON'\n");
  #endif

  // Inital Reset Sequence

  // 1.- power ON embebed socket
  Utils.socketON();

  // 2.- reset pulse for LoRa module initialization
 // pinMode(LORA_RESET_PIN, OUTPUT);
  //digitalWrite(LORA_RESET_PIN, HIGH);
  delay(100);

  //digitalWrite(LORA_RESET_PIN, LOW);
  delay(100);

  // 3.- SPI chip select
  pinMode(SX1272_SS,OUTPUT);
  digitalWrite(SX1272_SS,HIGH);
  delayMicroseconds(100);
 
  //Configure the MISO, MOSI, CS, SPCR.
  SPI.begin();
  //Set Most significant bit first
  SPI.setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  //Divide the clock frequency
  SPI.setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
  //Set data mode
  SPI.setDataMode(BCM2835_SPI_MODE0);
  delayMicroseconds(100);
  setMaxCurrent(0x1B);
  #if (SX1272_debug_mode > 1)
	  printf("## Setting ON with maximum current supply ##\n");
	  printf("\n");
  #endif

  // set FSK mode
  state = setFSK();

	//Set initialization values
	writeRegister(0x0,0x0);
	// registers for common settings 
	writeRegister(0x1,0x03); // Modulation Type FSK, No shaping , FS Rx mode
	writeRegister(0x2,0x0D); //  Bit Rate MSB  // 9.6 kbps
	writeRegister(0x3,0x05); //  Bit Rate LSB
	writeRegister(0x4,0x01);  //  MSB of the frequency deviation
	writeRegister(0x5,0x99); //  LSB of the frequency deviation
	writeRegister(0x6,0xD8); // MSB of the RF carrier frequency
	writeRegister(0x7,0x4C); // MSB of the RF carrier frequency
	writeRegister(0x8,0xCC); // LSB of the RF carrier frequency 
	// Registers for the transmitter  // Not so important as this RPI 0 is the receiver 
	writeRegister(0x9,0x4F); // PA output
	writeRegister(0xA,0x29); // RegPaRamp 
	writeRegister(0xB,0x1B); // RegOcp
	 //Registers for the receiver 
	writeRegister(0xC,0x23); // LnaGain (G1 + improved sensitivity for the RX)
	writeRegister(0xD,0x1E); // RegRxConfig 
	writeRegister(0xE,0x02); // RegRssiConfig
	writeRegister(0xF,0x0A); // RegRssiCollision
	writeRegister(0x10,0xFF); // RegRssiThresh
	writeRegister(0x11,0x0);  // RegRssiValue
	writeRegister(0x12,0x0B);  // RegRxBw = 50Khz Rx filter bw
	writeRegister(0x13,0x12);  // RegAfcBw = 83.3 KHz
	writeRegister(0x14,0x28); // RegOokPeak (Bit Synchronizer is enabled as packet mode is chosen --> see packet handling registers)
	writeRegister(0x15,0x0C);  // OoKFixedThreshold
	writeRegister(0x16,0x12);  //  RegOokAvg
	writeRegister(0x17,0x32);// reserved
	writeRegister(0x18,0x32);// reserved
	writeRegister(0x19,0x32); // reserved
	writeRegister(0x1A,0x0);  // 
	writeRegister(0x1B,0x0); //
	writeRegister(0x1C,0x0); //
	writeRegister(0x1D,0x0); //
	writeRegister(0x1E,0x0); //
	writeRegister(0x1F,0xAA); // RegPreambleDetect
	writeRegister(0x20,0x0); // 
	writeRegister(0x21,0x0);
	writeRegister(0x22,0x0); // 
	writeRegister(0x23,0x0);
	 //RC Oscillator registers 
	writeRegister(0x24,0x00); 
	// Packet handling registers 
	writeRegister(0x25,0x00);    //Size of MSB preamble (bytes)
	writeRegister(0x26,0x05);    //  LSB preamble 
	writeRegister(0x27,0x34);   // RegSynConfig 
	writeRegister(0x28,0x69); // RegSynchValue1 
	writeRegister(0x29,0x81); // RegSynchValue2 
	writeRegister(0x2A,0x7E); // RegSynchValue3 
	writeRegister(0x2B,0x96); // RegSynchValue4 
	writeRegister(0x2C,0x01); // RegSynchValue5 
	writeRegister(0x2D,0x01); // RegSynchValue6 
	writeRegister(0x2E,0x01); // RegSynchValue7 
	writeRegister(0x2F,0x01); // RegSynchValue8
	writeRegister(0x30,0x50);       // RegPacketConfig1 
	writeRegister(0x31,0x40);       // RegPacketConfig2 // Packet mode
	writeRegister(0x32,0x8); // PayloadLength // it is laways changed in RX as function setPacketLength re-write its value
	writeRegister(0x33,0x00); // RegNodeAdrs
	writeRegister(0x34,0x00); // RegBroadcastAdrs
	writeRegister(0x35,0x84); // RegFifoThresh
	 //sequencer registers 
	writeRegister(0x36,0x0);
	writeRegister(0x37,0x0);
	writeRegister(0x38,0x0);
	writeRegister(0x39,0xF5);
	writeRegister(0x3A,0x20);
	// service registers 
	writeRegister(0x3B,0x02);
	writeRegister(0x3C,0x0); // measured temperature
	writeRegister(0x3D,0x02);  
	// status registers 
	writeRegister(0x3E,0x0);
	writeRegister(0x3F,0x0);
	writeRegister(0x40,0x0);
	writeRegister(0x41,0x0);
	// version register 
	writeRegister(0x42,0x22);
	
  return state;
}

/*
 Function: Reads the indicated register.
 Returns: The content of the register
 Parameters:
   address: address register to read from
*/
byte SX1272::readRegister(byte address)
{
	digitalWrite(SX1272_SS,LOW);
    bitClear(address, 7);		// Bit 7 cleared to write in registers
    //SPI.transfer(address);
    //value = SPI.transfer(0x00);
    txbuf[0] = address;
	txbuf[1] = 0x00;
	maxWrite16();
	digitalWrite(SX1272_SS,HIGH);

    #if (SX1272_debug_mode > 1)
        printf("## Reading:  ##\tRegister ");
		printf("%X", address);
		printf(":  ");
		printf("%X", rxbuf[1]);
		printf("\n");
	#endif

    return rxbuf[1];
}

/*
 Function: Writes on the indicated register.
 Returns: Nothing
 Parameters:
   address: address register to write in
   data : value to write in the register
*/
void SX1272::writeRegister(byte address, byte data)
{
	digitalWrite(SX1272_SS,LOW);
	delayMicroseconds(1);
    bitSet(address, 7);			// Bit 7 set to read from registers
    //SPI.transfer(address);
    //SPI.transfer(data);
    txbuf[0] = address;
	txbuf[1] = data;
	maxWrite16();
	//digitalWrite(SX1272_SS,HIGH);

    #if (SX1272_debug_mode > 1)
        printf("## Writing:  ##\tRegister ");
		bitClear(address, 7);
		printf("%X", address);
		printf(":  ");
		printf("%X", data);
		printf("\n");
	#endif

}

/*
 Function: It gets the temperature from the measurement block module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
void SX1272::maxWrite16()
{
	digitalWrite(SX1272_SS,LOW);
	SPI.transfernb(txbuf, rxbuf, 2);
	digitalWrite(SX1272_SS,HIGH);
}

/*
 Function: Clears the interruption flags
 Returns: Nothing
*/
void SX1272::clearFlags()
{
    byte st0;

	st0 = readRegister(REG_OP_MODE);		// Save the previous status

	if( _modem == LORA )
	{ // LoRa mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby mode to write in registers
		writeRegister(REG_IRQ_FLAGS, 0xFF);	// LoRa mode flags register
		writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
		#if (SX1272_debug_mode > 1)
			printf("## LoRa flags cleared ##\n");
		#endif
	}
	else
	{ // FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby mode to write in registers
		writeRegister(REG_IRQ_FLAGS1, 0xFF); // FSK mode flags1 register
		writeRegister(REG_IRQ_FLAGS2, 0xFF); // FSK mode flags2 register
		writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
		#if (SX1272_debug_mode > 1)
			printf("## FSK flags cleared ##\n");
		#endif
	}
}

/*
 Function: Sets the module in FSK mode.
 Returns:   Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setFSK()
{
	
	uint8_t state = 2;
    byte st0;
    byte config1;

	#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("Starting 'setFSK'\n");
	#endif

	if(	_modem == LORA )
	{
	
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
		writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);
	}
	writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);	// Sleep mode (mandatory to change mode)
	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// FSK standby mode
	config1 = readRegister(REG_PACKET_CONFIG1);
	config1 = config1 & 0B01111101;		// clears bits 8 and 1 from REG_PACKET_CONFIG1 // Fixed length PacketFormat and NodeAddress+Broadcast Address
	config1 = config1 | 0B00000100;		// sets bit 2 from REG_PACKET_CONFIG1
	writeRegister(REG_PACKET_CONFIG1,config1);	// AddressFiltering = NodeAddress + BroadcastAddress , Fixed Length Packet format
	writeRegister(REG_FIFO_THRESH, 0x80);	// condition to start packet tx
	config1 = readRegister(REG_SYNC_CONFIG);
	config1 = config1 & 0B00111111;
	writeRegister(REG_SYNC_CONFIG,config1);

	delayMicroseconds(100);

	st0 = readRegister(REG_OP_MODE);	// Reading config mode
	if( st0 == FSK_STANDBY_MODE )
	{ // FSK mode
		_modem = FSK;
		state = 0;
//		#if (SX1272_debug_mode > 1)
			printf("## FSK set with success ##\n");
			printf("\n");
//		#endif
	}
	else
	{ // LoRa mode
		_modem = LORA;
		state = 1;
		#if (SX1272_debug_mode > 1)
			printf("** There has been an error while setting FSK **\n");
			printf("\n");
		#endif
	}
	return state;
}

/*
 Function: Sets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   addr: address value to set as node address.
*/
int8_t SX1272::setNodeAddress(uint8_t addr)
{
	byte st0;
	byte value;
	uint8_t state = 2;

	#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("Starting 'setNodeAddress'\n");
	#endif

	if( addr > 255 )
	{
		state = -1;
		#if (SX1272_debug_mode > 1)
			printf("** Node address must be less than 255 **\n");
			printf("\n");
		#endif
	}
	else
	{
		// Saving node address
		_nodeAddress = addr;
		st0 = readRegister(REG_OP_MODE);	  // Save the previous status

		if( _modem == LORA )
		{ // Allowing access to FSK registers while in LoRa standby mode
			writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
		}
		else
		{ //Set FSK Standby mode to write in registers
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
		}

		// Storing node and broadcast address
		writeRegister(REG_NODE_ADRS, addr);
		writeRegister(REG_BROADCAST_ADRS, BROADCAST_0);

		value = readRegister(REG_NODE_ADRS);
		writeRegister(REG_OP_MODE, st0);		// Getting back to previous status

		if( value == _nodeAddress )
		{
			state = 0;
			//#if (SX1272_debug_mode > 1)
				printf("## The receiver Node address is ");
				printf("%d", addr);
				printf(" has been successfully set ##\n");
				printf("\n");
			//#endif
		}
		else
		{
			state = 1;
			#if (SX1272_debug_mode > 1)
				printf("** There has been an error while setting address ##\n");
				printf("\n");
			#endif
		}
	}
	return state;
}


/*
 Function: Limits the current supply of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
int8_t SX1272::setMaxCurrent(uint8_t rate)
{
	int8_t state = 2;
	byte st0;

	#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("Starting 'setMaxCurrent'\n");
	#endif

	// Maximum rate value = 0x1B, because maximum current supply = 240 mA
	if (rate > 0x1B)
	{
		state = -1;
		#if (SX1272_debug_mode > 1)
			printf("** Maximum current supply is 240 mA, ");
			printf("so maximum parameter value must be 27 (DEC) or 0x1B (HEX) **\n");
			printf("\n");
		#endif
	}
	else
	{
		// Enable Over Current Protection
        rate |= 0B00100000;

		state = 1;
		st0 = readRegister(REG_OP_MODE);	// Save the previous status
		if( _modem == LORA )
		{ // LoRa mode
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Set LoRa Standby mode to write in registers
		}
		else
		{ // FSK mode
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Set FSK Standby mode to write in registers
		}
		writeRegister(REG_OCP, rate);		// Modifying maximum current supply
		writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
		state = 0;
	}
	return state;
}


/*
 Function: Indicates if module is configured with or without checking CRC.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1272::getCRC()
{
	int8_t state = 2;
	byte value;

	#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("Starting 'getCRC'\n");
	#endif

	if( _modem == LORA )
	{ // LoRa mode

		// take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
		value = readRegister(REG_MODEM_CONFIG1);
		if( bitRead(value, 1) == CRC_OFF )
		{ // CRCoff
			_CRC = CRC_OFF;
			#if (SX1272_debug_mode > 1)
				printf("## CRC is desactivated ##\n");
				printf("\n");
			#endif
			state = 0;
		}
		else
		{ // CRCon
			_CRC = CRC_ON;
			#if (SX1272_debug_mode > 1)
				printf("## CRC is activated ##\n");
				printf("\n");
			#endif
			state = 0;
		}
	}
	else
	{ // FSK mode

		// take out bit 2 from REG_PACKET_CONFIG1 indicates CrcOn
		value = readRegister(REG_PACKET_CONFIG1);
		if( bitRead(value, 4) == CRC_OFF )
		{ // CRCoff
			_CRC = CRC_OFF;
			#if (SX1272_debug_mode > 1)
				printf("## CRC is desactivated ##\n");
				printf("\n");
			#endif
			state = 0;
		}
		else
		{ // CRCon
			_CRC = CRC_ON;
			#if (SX1272_debug_mode > 1)
				printf("## CRC is activated ##\n");
				printf("\n");
			#endif
			state = 0;
		}
	}
	if( state != 0 )
	{
		state = 1;
		#if (SX1272_debug_mode > 1)
			printf("** There has been an error while getting configured CRC **\n");
			printf("\n");
		#endif
	}
	return state;
}

/*
 Function: Sets the module with CRC on.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1272::setCRC_ON()
{
  uint8_t state = 2;
  byte config1;

  #if (SX1272_debug_mode > 1)
	  printf("\n");
	  printf("Starting 'setCRC_ON'\n");
  #endif

  if( _modem == LORA )
  { // LORA mode
	config1 = readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the CRC bit
	config1 = config1 | 0B00000010;				// sets bit 1 from REG_MODEM_CONFIG1 = CRC_ON
	writeRegister(REG_MODEM_CONFIG1,config1);

	state = 1;

	config1 = readRegister(REG_MODEM_CONFIG1);
	if( bitRead(config1, 1) == CRC_ON )
	{ // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
		state = 0;
		_CRC = CRC_ON;
		#if (SX1272_debug_mode > 1)
			printf("## CRC has been activated ##\n");
			printf("\n");
		#endif
	}
  }
  else
  { // FSK mode
	config1 = readRegister(REG_PACKET_CONFIG1);	// Save config1 to modify only the CRC bit
	config1 = config1 | 0B00010000;				// set bit 4 and 3 from REG_MODEM_CONFIG1 = CRC_ON
	writeRegister(REG_PACKET_CONFIG1,config1);

	state = 1;

	config1 = readRegister(REG_PACKET_CONFIG1);
	if( bitRead(config1, 4) == CRC_ON )
	{ // take out bit 4 from REG_PACKET_CONFIG1 indicates CrcOn
		state = 0;
		_CRC = CRC_ON;
		#if (SX1272_debug_mode > 1)
			printf("## CRC has been activated ##\n");
			printf("\n");
		#endif
	}
  }
  if( state != 0 )
  {
	  state = 1;
	  #if (SX1272_debug_mode > 1)
		  printf("** There has been an error while setting CRC ON **\n");
		  printf("\n");
	  #endif
  }
  return state;
}

/*
 Function: Sets the module with CRC off.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1272::setCRC_OFF()
{
  int8_t state = 2;
  byte config1;

  #if (SX1272_debug_mode > 1)
	  printf("\n");
	  printf("Starting 'setCRC_OFF'\n");
  #endif

  if( _modem == LORA )
  { // LORA mode
  	config1 = readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the CRC bit
	config1 = config1 & 0B11111101;				// clears bit 1 from config1 = CRC_OFF
	writeRegister(REG_MODEM_CONFIG1,config1);

	config1 = readRegister(REG_MODEM_CONFIG1);
	if( (bitRead(config1, 1)) == CRC_OFF )
	{ // take out bit 1 from REG_MODEM_CONFIG1 indicates RxPayloadCrcOn
	  state = 0;
	  _CRC = CRC_OFF;
	  #if (SX1272_debug_mode > 1)
		  printf("## CRC has been desactivated ##\n");
		  printf("\n");
	  #endif
	}
  }
  else
  { // FSK mode
	config1 = readRegister(REG_PACKET_CONFIG1);	// Save config1 to modify only the CRC bit
	config1 = config1 & 0B11101111;				// clears bit 4 from config1 = CRC_OFF
	writeRegister(REG_PACKET_CONFIG1,config1);

	config1 = readRegister(REG_PACKET_CONFIG1);
	if( bitRead(config1, 4) == CRC_OFF )
	{ // take out bit 4 from REG_PACKET_CONFIG1 indicates RxPayloadCrcOn
		state = 0;
		_CRC = CRC_OFF;
		#if (SX1272_debug_mode > 1)
		    printf("## CRC has been desactivated ##\n");
		    printf("\n");
	    #endif
	}
  }
  if( state != 0 )
  {
	  state = 1;
	  #if (SX1272_debug_mode > 1)
		  printf("** There has been an error while setting CRC OFF **\n");
		  printf("\n");
	  #endif
  }
  return state;
}

/*
 Function: Checks if channel is a valid value.
 Returns: Boolean that's 'true' if the CR value exists and
		  it's 'false' if the CR value does not exist.
 Parameters:
   ch: frequency channel value to check.
*/
boolean	SX1272::isChannel(uint32_t ch)
{
  #if (SX1272_debug_mode > 1)
	  printf("\n");
	  printf("Starting 'isChannel'\n");
  #endif

  // Checking available values for _channel
  switch(ch)
  {
	  case CH_10_868:
	  case CH_11_868:
	  case CH_12_868:
	  case CH_13_868:
	  case CH_14_868:
	  case CH_15_868:
	  case CH_16_868:
	  case CH_17_868:
	  case CH_00_900:
	  case CH_01_900:
	  case CH_02_900:
	  case CH_03_900:
	  case CH_04_900:
	  case CH_05_900:
	  case CH_06_900:
	  case CH_07_900:
	  case CH_08_900:
	  case CH_09_900:
	  case CH_10_900:
	  case CH_11_900:	return true;
						break;

	  default:			return false;
  }
  #if (SX1272_debug_mode > 1)
	  printf("## Finished 'isChannel' ##\n");
	  printf("\n");
  #endif
}

/*
 Function: Indicates the frequency channel within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getChannel()
{
  uint8_t state = 2;
  uint32_t ch;
  uint8_t freq3;
  uint8_t freq2;
  uint8_t freq1;

  #if (SX1272_debug_mode > 1)
	  printf("\n");
	  printf("Starting 'getChannel'\n");
  #endif

  freq3 = readRegister(REG_FRF_MSB);	// frequency channel MSB
  freq2 = readRegister(REG_FRF_MID);	// frequency channel MID
  freq1 = readRegister(REG_FRF_LSB);	// frequency channel LSB
  ch = ((uint32_t)freq3 << 16) + ((uint32_t)freq2 << 8) + (uint32_t)freq1;
  _channel = ch;				// frequency channel

  if( (_channel == ch) && isChannel(_channel) )
  {
	  state = 0;
	  #if (SX1272_debug_mode > 1)
		  printf("## Frequency channel is ");
		  printf("%X", _channel);
		  printf(" ##\n");
		  printf("\n");
	  #endif
	  printf("## Frequency channel is ");
		  printf("%X", _channel);
		  printf(" ##\n");
		  printf("\n");
  }
  else
  {
	  state = 1;
  }
  return state;
}

/*
 Function: Sets the indicated channel in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   ch: frequency channel value to set in configuration.
*/
int8_t SX1272::setChannel(uint32_t ch)
{
  byte st0;
  int8_t state = 2;
  unsigned int freq3;
  unsigned int freq2;
  uint8_t freq1;
  uint32_t freq;

  #if (SX1272_debug_mode > 1)
	  printf("\n");
	  printf("Starting 'setChannel'\n");
  #endif

  st0 = readRegister(REG_OP_MODE);	// Save the previous status
  if( _modem == LORA )
  {
	  // LoRa Stdby mode in order to write in registers
	  writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
  }
  else
  {
	  // FSK Stdby mode in order to write in registers
	  writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
  }

  freq3 = ((ch >> 16) & 0x0FF);		// frequency channel MSB
  freq2 = ((ch >> 8) & 0x0FF);		// frequency channel MIB
  freq1 = (ch & 0xFF);				// frequency channel LSB

  writeRegister(REG_FRF_MSB, freq3);
  writeRegister(REG_FRF_MID, freq2);
  writeRegister(REG_FRF_LSB, freq1);

  delayMicroseconds(100);

  // storing MSB in freq channel value
  freq3 = (readRegister(REG_FRF_MSB));
  freq = (freq3 << 8) & 0xFFFFFF;

  // storing MID in freq channel value
  freq2 = (readRegister(REG_FRF_MID));
  freq = (freq << 8) + ((freq2 << 8) & 0xFFFFFF);

  // storing LSB in freq channel value
  freq = freq + ((readRegister(REG_FRF_LSB)) & 0xFFFFFF);

  if( freq == ch )
  {
    state = 0;
    _channel = ch;
    #if (SX1272_debug_mode > 1)
		printf("## Frequency channel ");
		printf("%X", ch);
		printf(" has been successfully set ##\n");
		printf("\n");
	#endif
  }
  else
  {
    state = 1;
  }

  if( not isChannel(ch) )
  {
	 state = -1;
	 #if (SX1272_debug_mode > 1)
		 printf("** Frequency channel ");
		 printf("%X", ch);
		 printf("is not a correct value **\n");
		 printf("\n");
	 #endif
  }

  writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  delayMicroseconds(100);
  return state;
}

/*
 Function: Gets the signal power within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPower()
{
  uint8_t state = 2;
  byte value = 0x00;

  #if (SX1272_debug_mode > 1)
	  printf("\n");
	  printf("Starting 'getPower'\n");
  #endif

  value = readRegister(REG_PA_CONFIG);
  state = 1;

  _power = value;
  if( (value > -1) & (value < 16) )
  {
	    state = 0;
		#if (SX1272_debug_mode > 1)
			printf("## Output power is ");
			printf("%X", _power);
			printf(" ##\n");
			printf("\n");
		#endif
	}

  return state;
}

/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t SX1272::setPower(char p)
{
  byte st0;
  int8_t state = 2;
  byte value = 0x00;

  #if (SX1272_debug_mode > 1)
	  printf("\n");
	  printf("Starting 'setPower'\n");
  #endif

  st0 = readRegister(REG_OP_MODE);	  // Save the previous status
  if( _modem == LORA )
  { // LoRa Stdby mode to write in registers
	  writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
  }
  else
  { // FSK Stdby mode to write in registers
	  writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
  }

  switch (p)
  {
    // L = low
    // H = high
    // M = max

    case 'M':  _power = 0x4F;
               break;

    case 'L':  _power = 0x00;
               break;

    case 'H':  _power = 0x07;
               break;

    default:   state = -1;
               break;
  }

  writeRegister(REG_PA_CONFIG, _power);	// Setting output power value
  value = readRegister(REG_PA_CONFIG);

  if( value == _power )
  {
	  state = 0;
	  #if (SX1272_debug_mode > 1)
		  printf("## Output power has been successfully set ##\n");
		  printf("\n");
	  #endif
  }
  else
  {
	  state = 1;
  }

  writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  delayMicroseconds(100);
  return state;
}

/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t SX1272::setPowerNum(uint8_t pow)
{
  byte st0;
  int8_t state = 2;
  byte value = 0x00;

  #if (SX1272_debug_mode > 1)
	  printf("\n");
	  printf("Starting 'setPower'\n");
  #endif

  st0 = readRegister(REG_OP_MODE);	  // Save the previous status
  if( _modem == LORA )
  { // LoRa Stdby mode to write in registers
	  writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
  }
  else
  { // FSK Stdby mode to write in registers
	  writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
  }

  if ( (pow >= 0) & (pow < 15) )
  {
	  _power = pow;
  }
  else
  {
	  state = -1;
	  #if (SX1272_debug_mode > 1)
		  printf("## Power value is not valid ##\n");
		  printf("\n");
	  #endif
  }

  writeRegister(REG_PA_CONFIG, _power);	// Setting output power value
  value = readRegister(REG_PA_CONFIG);

  if( value == _power )
  {
	  state = 0;
	  #if (SX1272_debug_mode > 1)
		  printf("## Output power has been successfully set ##\n");
		  printf("\n");
	  #endif
  }
  else
  {
	  state = 1;
  }

  writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  delayMicroseconds(100);
  return state;
}


/*
 Function: Gets the preamble length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPreambleLength()
{
	int8_t state = 2;
	uint8_t p_length;

	#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("Starting 'getPreambleLength'\n");
	#endif

	state = 1;
	if( _modem == LORA )
  	{ // LORA mode
  		p_length = readRegister(REG_PREAMBLE_MSB_LORA);
  		// Saving MSB preamble length in LoRa mode
		_preamblelength = (p_length << 8) & 0xFFFF;
		p_length = readRegister(REG_PREAMBLE_LSB_LORA);
  		// Saving LSB preamble length in LoRa mode
		_preamblelength = _preamblelength + (p_length & 0xFFFF);
		#if (SX1272_debug_mode > 1)
			printf("## Preamble length configured is ");
			printf("%X", _preamblelength);
			printf(" ##");
			printf("\n");
		#endif
	}
	else
	{ // FSK mode
		p_length = readRegister(REG_PREAMBLE_MSB_FSK);
		// Saving MSB preamble length in FSK mode
		_preamblelength = (p_length << 8) & 0xFFFF;
		p_length = readRegister(REG_PREAMBLE_LSB_FSK);
		// Saving LSB preamble length in FSK mode
		_preamblelength = _preamblelength + (p_length & 0xFFFF);
		#if (SX1272_debug_mode > 1)
			printf("## Preamble length configured is ");
			printf("%X", _preamblelength);
			printf(" ##");
			printf("\n");
		#endif
		printf("## Preamble length configured is ");
			printf("%X", _preamblelength);
			printf(" ##");
			printf("\n");
	}
	state = 0;
	return state;
}

/*
 Function: Sets the preamble length in the module
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   l: length value to set as preamble length.
*/
uint8_t SX1272::setPreambleLength(uint16_t l)
{
	byte st0;
	uint8_t p_length;
	int8_t state = 2;

	#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("Starting 'setPreambleLength'\n");
	#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	state = 1;
	if( _modem == LORA )
  	{ // LoRa mode
  		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set Standby mode to write in registers
  		p_length = ((l >> 8) & 0x0FF);
  		// Storing MSB preamble length in LoRa mode
		writeRegister(REG_PREAMBLE_MSB_LORA, p_length);
		p_length = (l & 0x0FF);
		// Storing LSB preamble length in LoRa mode
		writeRegister(REG_PREAMBLE_LSB_LORA, p_length);
	}
	else
	{ // FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    // Set Standby mode to write in registers
		p_length = ((l >> 8) & 0x0FF);
  		// Storing MSB preamble length in FSK mode
		writeRegister(REG_PREAMBLE_MSB_FSK, p_length);
		p_length = (l & 0x0FF);
  		// Storing LSB preamble length in FSK mode
		writeRegister(REG_PREAMBLE_LSB_FSK, p_length);
	}

	state = 0;
	#if (SX1272_debug_mode > 1)
		printf("## Preamble length ");
		printf("%X", l);
		printf(" has been successfully set ##\n");
		printf("\n");
	#endif

	writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  	delayMicroseconds(100);
	return state;
}

/*
 Function: Gets the payload length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::getPayloadLength()
{
	uint8_t state = 2;

	#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("Starting 'getPayloadLength'\n");
	#endif

	if( _modem == LORA )
  	{ // LORA mode
  		// Saving payload length in LoRa mode
		_payloadlength = readRegister(REG_PAYLOAD_LENGTH_LORA);
		state = 1;
	}
	else
	{ // FSK mode
  		// Saving payload length in FSK mode
		_payloadlength = readRegister(REG_PAYLOAD_LENGTH_FSK);
		state = 1;
	}

	#if (SX1272_debug_mode > 1)
		printf("## Payload length configured is ");
		printf("%X", _payloadlength);
		printf(" ##\n");
		printf("\n");
	#endif
	

	state = 0;
	return state;
}

/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t SX1272::setPacketLength()
{
	uint16_t length;
    //printf(" #### in setPacketLength() the _payloadlength = %d #### \n",_payloadlength);
	length = _payloadlength + OFFSET_PAYLOADLENGTH; // ==========================================0
	return setPacketLength(length);
}

/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   l: length value to set as payload length.
*/
int8_t SX1272::setPacketLength(uint8_t l)
{
	byte st0;
	byte value = 0x00;
	int8_t state = 2;

	#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("### Starting 'setPacketLength(uint8_t l)' with parameter l =  _payloadlength + OFFSET_PAYLOADLENGTH = %d ###\n",l);
	#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	packet_received.length = l;

	if( _modem == LORA )
  	{ // LORA mode
  		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set LoRa Standby mode to write in registers
		writeRegister(REG_PAYLOAD_LENGTH_LORA, packet_sent.length);
		// Storing payload length in LoRa mode
		value = readRegister(REG_PAYLOAD_LENGTH_LORA);
	}
	else
	{ // FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    //  Set FSK Standby mode to write in registers
		writeRegister(REG_PAYLOAD_LENGTH_FSK, packet_received.length);
		// Storing payload length in FSK mode
		value = readRegister(REG_PAYLOAD_LENGTH_FSK);
	}

	if( packet_received.length == value )
	{
		state = 0;
		#if (SX1272_debug_mode > 1)
			printf("### Packet length ");
			printf("%d", packet_received.length);
			printf(" has been successfully set ###\n");
			printf("\n");
		#endif
	}
	else
	{
		state = 1;
	}

	writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  	delayMicroseconds(250);
	return state;
}




/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receive()
{
	  uint8_t state = 2;

	  #if (SX1272_debug_mode > 1)
		  printf("\n");
		  printf("#####Starting 'receive'#####\n");
	  #endif
	
	  // Initializing packet_received struct
	  //printf("packet_received before memset length : %d \n",packet_received.length);
	  memset( &packet_received, 0x00, sizeof(packet_received) );
      //printf("packet_received after memset length : %d \n",packet_received.length);
		// Setting Testmode 
		// register 0x31 REG_PACKET_CONFIG2 --> FSK / REG_DETECT_OPTIMIZE --> LoRa
   		 writeRegister(0x31,0x40); // packet mode , packet length most significant bits are (2:0) for FSK
    	// Set LowPnTxPllOff 
    	writeRegister(REG_PA_RAMP, 0x09);

	  writeRegister(REG_LNA, 0x23);			// Important in reception
	  writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
  	  // change RegSymbTimeoutLsb 
      writeRegister(REG_SYMB_TIMEOUT_LSB, 0xFF);
  	  writeRegister(REG_FIFO_RX_BYTE_ADDR, 0x00); // Setting current value of reception buffer pointer
	  clearFlags();						// Initializing flags // originaly it is commented
	  state = 1;
    
    //printf("_modem: %d\n", _modem);
	  if( _modem == LORA )
	  { // LoRa mode
	  	  state = setPacketLength(MAX_LENGTH);	// With MAX_LENGTH gets all packets with length < MAX_LENGTH
		  writeRegister(REG_OP_MODE, LORA_RX_MODE);  	  // LORA mode - Rx
		  #if (SX1272_debug_mode > 1)
		  	  printf("## Receiving LoRa mode activated with success ##\n");
		  	  printf("\n");
		  #endif
	  }
	  else
	  { // FSK mode
		 
		  state = setPacketLength(); // ==============originaly no MAX_LENGTH_FSK
		  writeRegister(REG_OP_MODE, FSK_RX_MODE);  // FSK mode - Rx
		  #if (SX1272_debug_mode > 1)
		  	  printf("##### Receiving FSK mode activated with success #####\n");
		  	  printf("\n");
		  #endif
	  }
	  return state;
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::receivePacketTimeout(uint16_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("###### Starting 'receivePacketTimeout(uint16_t wait)' ######\n");
	#endif

	state = receive(); // ==========
	if( state == 0 )
	{
		if( availableData(wait) )
		{
			// If packet received, getPacket
			state_f = getPacket();
		}
		else
		{
			state_f = 1;
		}
	}
	else
	{
		state_f = state;
	}
	return state_f;
}


/*
 Function: If a packet is received, checks its destination.
 Returns: Boolean that's 'true' if the packet is for the module and
		  it's 'false' if the packet is not for the module.
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
boolean	SX1272::availableData(uint16_t wait)
{
	byte value;
	byte header = 0;
	boolean forme = false;
	boolean	_hreceived = false;
	unsigned long previous;


	#if (SX1272_debug_mode > 0)
		printf("\n");
		printf("Starting 'availableData'\n");
	#endif

	previous = millis();
	if( _modem == LORA )
	{ // LoRa mode
		value = readRegister(REG_IRQ_FLAGS);
		// Wait to ValidHeader interrupt
		while( (bitRead(value, 4) == 0) && (millis() - previous < (unsigned long)wait) )
		{
			value = readRegister(REG_IRQ_FLAGS);
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		} // end while (millis)
		if( bitRead(value, 4) == 1 )
		{ // header received
			#if (SX1272_debug_mode > 0)
				printf("## Valid Header received in LoRa mode ##\n");
			#endif
			_hreceived = true;
			while( (header == 0) && (millis() - previous < (unsigned long)wait) )
			{ // Waiting to read first payload bytes from packet
				header = readRegister(REG_FIFO_RX_BYTE_ADDR);
                 delay(1000);
				// Condition to avoid an overflow (DO NOT REMOVE)
				if( millis() < previous )
				{
					previous = millis();
				}
			}
			if( header != 0 )
			{ // Reading first byte of the received packet
				_destination = readRegister(REG_FIFO);
			}
		}
		else
		{
			forme = false;
			_hreceived = false;
			#if (SX1272_debug_mode > 0)
				printf("** The timeout has expired **\n");
				printf("\n");
			#endif
		}
	}
	else
	{ // FSK mode
		uint8_t _source = 0;
		uint8_t _packnum =0;
		value = readRegister(REG_IRQ_FLAGS1); // originally REG_IRQ_FLAGS2
		// Wait to Payload Ready interrupt
		//sx1272.getPreambleLength();
		while( (bitRead(value, 1) == 0) && (millis() - previous < wait) ) // originally bitRead(value, 2)
		{
			
			value = readRegister(REG_IRQ_FLAGS1);
			
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		}// end while (millis)
	
		if( bitRead(value, 1) == 1 )	// something received // originally bitRead(value, 2)
		{
			_hreceived = true;
			
		
				printf("##====== Valid Preamble detected in FSK mode====== ##\n");
			
			value = readRegister(REG_IRQ_FLAGS1);
			while( (bitRead(value, 0) == 0) && (millis() - previous < wait) ) 
			{
			
			value = readRegister(REG_IRQ_FLAGS1);
			
			// Condition to avoid an overflow (DO NOT REMOVE)
				if( millis() < previous )
				{
					previous = millis();
				}
			}// end while (millis)
			if( bitRead(value, 0) == 1 )
			printf("MATCHing synch word \n");
			delay(10);
			_destination = readRegister(REG_FIFO);
			
			
		    
		}
		else
		{
			forme = false;
			_hreceived = false;
			//#if (SX1272_debug_mode > 0)
				printf("** The timeout has expired **\n");
				printf("\n");
			//#endif
		}
	}
// We use _hreceived because we need to ensure that _destination value is correctly
// updated and is not the _destination value from the previously packet
	if( _hreceived )
	{ // Checking destination
		#if (SX1272_debug_mode > 0)
			printf("## Checking destination ##\n");
		#endif
		if( (_destination == _nodeAddress)  ) // || (_destination == BROADCAST_0)
		{ // LoRa or FSK mode
			forme = true;
			#if (SX1272_debug_mode > 0)
				printf("## Packet received is for me ##\n");
			#endif
		}
		else
		{
			forme = false;
			#if (SX1272_debug_mode > 0)
				printf("## Packet received is not for me ##\n");
				printf("\n");
			#endif
			if( _modem == LORA )	// STANDBY PARA MINIMIZAR EL CONSUMO
			{ // LoRa mode
				writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
			}
			else
			{ //  FSK mode
				writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
			}
		}
	}
//----else
//	{
//	}
	return forme;
}

/*
 Function: It gets and stores a packet if it is received.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
int8_t SX1272::getPacket()
{
	return getPacket(MAX_TIMEOUT);
}

/*
 Function: It gets and stores a packet if it is received before ending 'wait' time.
 Returns:  Integer that determines if there has been any error
   state = 3  --> The command has been executed but packet has been incorrectly received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
int8_t SX1272::getPacket(uint16_t wait)
{
	uint8_t state = 2;
	byte value = 0x00;
	unsigned long previous;
	boolean p_received = false;

	//#if (SX1272_debug_mode > 0)
		printf("\n");
		printf("Starting 'getPacket' wait\n");
	//#endif

	previous = millis();
	if( _modem == LORA )
	{ // LoRa mode
		value = readRegister(REG_IRQ_FLAGS);
		// Wait until the packet is received (RxDone flag) or the timeout expires
		while( (bitRead(value, 6) == 0) && (millis() - previous < (unsigned long)wait) )
		{
			value = readRegister(REG_IRQ_FLAGS);
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		} // end while (millis)

		if( (bitRead(value, 6) == 1) && (bitRead(value, 5) == 0) )
		{ // packet received & CRC correct
			p_received = true;	// packet correctly received
			_reception = CORRECT_PACKET;
			#if (SX1272_debug_mode > 0)
				printf("## Packet correctly received in LoRa mode ##\n");
			#endif
		}
		else
		{
			if( bitRead(value, 5) != 0 )
			{ // CRC incorrect
				_reception = INCORRECT_PACKET;
				state = 3;
				#if (SX1272_debug_mode > 0)
					printf("** The CRC is incorrect **\n");
					printf("\n");
				#endif
			}
		}
		//writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
	}
	else
	{ // FSK mode
		
		// big dutchman =============================
		//value = readRegister(REG_IRQ_FLAGS1);
		//while( (bitRead(value, 0) == 0) && (millis() - previous < wait) )
		//{
			//value = readRegister(REG_IRQ_FLAGS1);
			//// Condition to avoid an overflow (DO NOT REMOVE)
			//if( millis() < previous )
			//{
				//previous = millis();
			//}
		//} // end while (millis)
		//if( bitRead(value, 0) == 1 )
		//printf("MATCHing synch word \n");
		//if( bitRead(value, 1) == 1 )
		//{
			//printf("CRC is OK  \n");
			//_reception = CORRECT_PACKET;
				//p_received = true;
		//}
		//value = readRegister(REG_IRQ_FLAGS2);
		//if( bitRead(value, 2) == 1 )
		//printf("IR Payload is OK  \n");
		
		//writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
		
		// big dutchman =============================
		
		value = readRegister(REG_IRQ_FLAGS2);
		while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
		{
			value = readRegister(REG_IRQ_FLAGS2);
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		} // end while (millis)
		if( bitRead(value, 2) == 1 )
		{ // packet received
 			if( bitRead(value, 1) == 1 )
			{ // CRC correct
				_reception = CORRECT_PACKET;
				p_received = true;
				//#if (SX1272_debug_mode > 0)
					printf("## Packet correctly received in FSK mode ##\n");
				//#endif
			}
			else
			{ // CRC incorrect
				_reception = INCORRECT_PACKET;
				state = 3;
				p_received = false;
				//#if (SX1272_debug_mode > 0)
					printf("## Packet incorrectly received in FSK mode ##\n");
				//#endif
			}
		}
		else
		{
			//#if (SX1272_debug_mode > 0)
				printf("** The timeout has expired **\n");
				printf("\n");
			//#endif
		}
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
	}
	if( p_received )
	{
		// Store the packet
		if( _modem == LORA )
		{
			writeRegister(REG_FIFO_ADDR_PTR, 0x00);  		// Setting address pointer in FIFO data buffer
			packet_received.dst = readRegister(REG_FIFO);	// Storing first byte of the received packet
		}
		else
		{
			//value = readRegister(REG_PACKET_CONFIG1);
			value = readRegister(REG_IRQ_FLAGS2);
			if( (bitRead(value, 2) == 0) && (bitRead(value, 1) == 0) )
			if( (bitRead(value, 2) == 1) && (bitRead(value, 1) == 1) )
			{
				printf("CRC Ok and IR PayloadRead ok \n");
				packet_received.dst = readRegister(REG_FIFO); // Storing first byte of the received packet
			}
			else
			{
				packet_received.dst = _destination;			// Storing first byte of the received packet
			}
		}
		packet_received.src = readRegister(REG_FIFO);		// Reading second byte of the received packet
		packet_received.packnum = readRegister(REG_FIFO);	// Reading third byte of the received packet
		packet_received.length = readRegister(REG_FIFO);	// Reading fourth byte of the received packet
		if( _modem == LORA )
		{
			_payloadlength = packet_received.length - OFFSET_PAYLOADLENGTH;
		}
		if( packet_received.length > (MAX_LENGTH + 1) )
		{
			#if (SX1272_debug_mode > 0)
				printf("Corrupted packet, length must be less than 256\n");
			#endif
		}
		else
		{
			for(unsigned int i = 0; i < _payloadlength; i++)
			{
				packet_received.data[i] = readRegister(REG_FIFO); // Storing payload
			}
			packet_received.retry = readRegister(REG_FIFO);
			// Print the packet if debug_mode
			//#if (SX1272_debug_mode > 0)
				printf("## Packet received:\n");
				printf("Destination: ");
				printf("%d\n", packet_received.dst);			 	// Printing destination
				printf("Source: ");
				printf("%d\n", packet_received.src);			 	// Printing source
				printf("Packet number: ");
				printf("%d\n", packet_received.packnum);			// Printing packet number
				printf("Packet length: ");
				printf("%d\n", packet_received.length);			// Printing packet length
				printf("Data: ");
				for(unsigned int i = 0; i < _payloadlength; i++)
				{
					printf("%c", packet_received.data[i]);		// Printing payload
				}
				printf("\n");
				printf("Retry number: ");
				printf("%d\n", packet_received.retry);			// Printing number retry
				printf(" ##\n");
				printf("\n");
			//#endif
			state = 0;
		}
	}
	else
	{
		state = 1;
		if( (_reception == INCORRECT_PACKET) && (_retries < _maxRetries) )
		{
			_retries++;
			#if (SX1272_debug_mode > 0)
				printf("## Retrying to send the last packet ##\n");
				printf("\n");
			#endif
		}
	}
	if( _modem == LORA )
	{
		writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
	}
	clearFlags();	// Initializing flags
	if( wait > MAX_WAIT )
	{
		state = -1;
		#if (SX1272_debug_mode > 0)
			printf("** The timeout must be smaller than 12.5 seconds **\n");
			printf("\n");
		#endif
	}

	return state;
}


/*
 Function: It sets the timeout according to the configured mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1272::setTimeout()
{
	uint8_t state = 2;
	uint16_t delay;

	//#if (SX1272_debug_mode > 1)
		printf("\n");
		printf("Starting 'setTimeout()'\n");
	//#endif

	state = 1;
	if( _modem == LORA )
	{
		switch(_spreadingFactor)
		{	// Choosing Spreading Factor
			case SF_6:	switch(_bandwidth)
						{	// Choosing bandwidth
							case BW_125:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 335;
														break;
												case CR_6: _sendTime = 352;
														break;
												case CR_7: _sendTime = 368;
														break;
												case CR_8: _sendTime = 386;
														break;
											}
											break;
							case BW_250:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 287;
														break;
												case CR_6: _sendTime = 296;
														break;
												case CR_7: _sendTime = 305;
														break;
												case CR_8: _sendTime = 312;
														break;
											}
											break;
							case BW_500:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 242;
														break;
												case CR_6: _sendTime = 267;
														break;
												case CR_7: _sendTime = 272;
														break;
												case CR_8: _sendTime = 276;
														break;
											}
											break;
						}
						break;

			case SF_7:	switch(_bandwidth)
						{	// Choosing bandwidth
							case BW_125:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 408;
														break;
												case CR_6: _sendTime = 438;
														break;
												case CR_7: _sendTime = 468;
														break;
												case CR_8: _sendTime = 497;
														break;
											}
											break;
							case BW_250:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 325;
														break;
												case CR_6: _sendTime = 339;
														break;
												case CR_7: _sendTime = 355;
														break;
												case CR_8: _sendTime = 368;
														break;
											}
											break;
							case BW_500:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 282;
														break;
												case CR_6: _sendTime = 290;
														break;
												case CR_7: _sendTime = 296;
														break;
												case CR_8: _sendTime = 305;
														break;
											}
											break;
						}
						break;

			case SF_8:	switch(_bandwidth)
						{	// Choosing bandwidth
							case BW_125:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 537;
														break;
												case CR_6: _sendTime = 588;
														break;
												case CR_7: _sendTime = 640;
														break;
												case CR_8: _sendTime = 691;
														break;
											}
											break;
							case BW_250:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 388;
														break;
												case CR_6: _sendTime = 415;
														break;
												case CR_7: _sendTime = 440;
														break;
												case CR_8: _sendTime = 466;
														break;
											}
											break;
							case BW_500:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 315;
														break;
												case CR_6: _sendTime = 326;
														break;
												case CR_7: _sendTime = 340;
														break;
												case CR_8: _sendTime = 352;
														break;
											}
											break;
						}
						break;

			case SF_9:	switch(_bandwidth)
						{	// Choosing bandwidth
							case BW_125:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 774;
														break;
												case CR_6: _sendTime = 864;
														break;
												case CR_7: _sendTime = 954;
														break;
												case CR_8: _sendTime = 1044;
														break;
											}
											break;
							case BW_250:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 506;
														break;
												case CR_6: _sendTime = 552;
														break;
												case CR_7: _sendTime = 596;
														break;
												case CR_8: _sendTime = 642;
														break;
											}
											break;
							case BW_500:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 374;
														break;
												case CR_6: _sendTime = 396;
														break;
												case CR_7: _sendTime = 418;
														break;
												case CR_8: _sendTime = 441;
														break;
											}
											break;
						}
						break;

			case SF_10:	switch(_bandwidth)
						{	// Choosing bandwidth
							case BW_125:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 1226;
														break;
												case CR_6: _sendTime = 1388;
														break;
												case CR_7: _sendTime = 1552;
														break;
												case CR_8: _sendTime = 1716;
														break;
											}
											break;
							case BW_250:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 732;
														break;
												case CR_6: _sendTime = 815;
														break;
												case CR_7: _sendTime = 896;
														break;
												case CR_8: _sendTime = 977;
														break;
											}
											break;
							case BW_500:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 486;
														break;
												case CR_6: _sendTime = 527;
														break;
												case CR_7: _sendTime = 567;
														break;
												case CR_8: _sendTime = 608;
														break;
											}
											break;
						}
						break;

			case SF_11:	switch(_bandwidth)
						{	// Choosing bandwidth
							case BW_125:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 2375;
														break;
												case CR_6: _sendTime = 2735;
														break;
												case CR_7: _sendTime = 3095;
														break;
												case CR_8: _sendTime = 3456;
														break;
											}
											break;
							case BW_250:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 1144;
														break;
												case CR_6: _sendTime = 1291;
														break;
												case CR_7: _sendTime = 1437;
														break;
												case CR_8: _sendTime = 1586;
														break;
											}
											break;
							case BW_500:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 691;
														break;
												case CR_6: _sendTime = 766;
														break;
												case CR_7: _sendTime = 838;
														break;
												case CR_8: _sendTime = 912;
														break;
											}
											break;
						}
						break;

			case SF_12: switch(_bandwidth)
						{	// Choosing bandwidth
							case BW_125:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 4180;
														break;
												case CR_6: _sendTime = 4836;
														break;
												case CR_7: _sendTime = 5491;
														break;
												case CR_8: _sendTime = 6146;
														break;
											}
											break;
							case BW_250:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 1965;
														break;
												case CR_6: _sendTime = 2244;
														break;
												case CR_7: _sendTime = 2521;
														break;
												case CR_8: _sendTime = 2800;
														break;
											}
											break;
							case BW_500:	switch(_codingRate)
											{	// Choosing coding rate
												case CR_5: _sendTime = 1102;
														break;
												case CR_6: _sendTime = 1241;
														break;
												case CR_7: _sendTime = 1381;
														break;
												case CR_8: _sendTime = 1520;
														break;
											}
											break;
						}
						break;
			default: _sendTime = MAX_TIMEOUT;
		}
	}
	else
	{
		_sendTime = MAX_TIMEOUT;
	}
	delay = ((0.1*_sendTime) + 1);
	_sendTime = (uint16_t) ((_sendTime * 1.2) + (rand()%delay));
	//#if (SX1272_debug_mode > 1)
		printf("Timeout to send/receive is: ");
		printf("%d",_sendTime);
		printf("\n");
	//#endif
	
	return state;
}





SX1272 sx1272 = SX1272();
