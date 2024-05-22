#ifndef _ADS1258_h
#define _ADS1258_h


/*Configuration Register MUXDIF Multiplexer Differential Input Select Register (Address = 03h) ( see p39)
This register selects input channels of multiplexer to be used for Fixed channel mode*/
/* BIT7 - BIT6   -  BIT5  -  BIT4 - BIT3   - BIT2 - BIT1 - BIT0 */
/* DIFF7	- DIFF6 - DIFF5 - DIFF4 - DIFF3 - DIFF2 - DIFF1 - DIFF0 */
#define DIFF7 B10000000 // Select DIFF7
#define DIFF6 B01000000 // Select DIFF6
#define DIFF5 B00100000 // Select DIFF5
#define DIFF4 B00010000 // Select DIFF4
#define DIFF3 B00001000 // Select DIFF3
#define DIFF2 B00000100 // Select DIFF2
#define DIFF1 B00000010 // Select DIFF1
#define DIFF0 B00000001 // Select DIFF0

//Single-ended inputs
#define SING_0 0b00001111 //A0 + GND (common) as single-ended input
#define SING_1 0b00011111 //A1 + GND (common) as single-ended input
#define SING_2 0b00101111 //A2 + GND (common) as single-ended input
#define SING_3 0b00111111 //A3 + GND (common) as single-ended input
#define SING_4 0b01001111 //A4 + GND (common) as single-ended input
#define SING_5 0b01011111 //A5 + GND (common) as single-ended input
#define SING_6 0b01101111 //A6 + GND (common) as single-ended input
#define SING_7 0b01111111 //A7 + GND (common) as single-ended input

//PGA settings			  //Input voltage range
#define PGA_1 0b00000000  //± 5 V
#define PGA_2 0b00000001  //± 2.5 V
#define PGA_4 0b00000010  //± 1.25 V
#define PGA_8 0b00000011  //± 625 mV
#define PGA_16 0b00000100 //± 312.5 mV
#define PGA_32 0b00000101 //+ 156.25 mV
#define PGA_64 0b00000110 //± 78.125 mV

//Datarate						  //DEC
#define DRATE_30000SPS 0b11110000 //240
#define DRATE_15000SPS 0b11100000 //224
#define DRATE_7500SPS 0b11010000  //208
#define DRATE_3750SPS 0b11000000  //192
#define DRATE_2000SPS 0b10110000  //176
#define DRATE_1000SPS 0b10100001  //161
#define DRATE_500SPS 0b10010010   //146
#define DRATE_100SPS 0b10000010   //130
#define DRATE_60SPS 0b01110010    //114
#define DRATE_50SPS 0b01100011    //99
#define DRATE_30SPS 0b01010011    //83
#define DRATE_25SPS 0b01000011    //67
#define DRATE_15SPS 0b00110011    //51
#define DRATE_10SPS 0b00100011    //35
#define DRATE_5SPS 0b00010011     //19
#define DRATE_2SPS 0b00000011     //3

//Status register
#define BITORDER_MSB 0
#define BITORDER_LSB 1
#define ACAL_DISABLED 0
#define ACAL_ENABLED 1
#define BUFFER_DISABLED 0
#define BUFFER_ENABLED 1

//Register addresses
#define CONFIG0_REG 0x00
#define CONFIG1_REG 0x01
#define MUXSCH_REG 0x02
#define MUXDIF_REG 0x03
#define MUXSG0_REG 0x04
#define MUXSG1_REG 0x05
#define SYSRED_REG 0x06
#define GPIOC_REG 0x07
#define GPIOD_REG 0x08
#define ID_REG 0x09

//Command definitions

#define MUL		0x10	//Multiple register write enabled
#define CDRD	0x00	//Channel Data read direct (no command) 
#define RDATA	0x30	//Channel data read command (register format, set MUL=1, status byte included in data) 0x0010 xxxx 
#define RREG	0x40	//Register read command 0x0100 xxxx
#define WREG	0x60	//Register write command 0x0110 xxxx
#define PULCON	0x80	//Pulse convert command, 0x1000 xxxx
#define RESET	0xC0	//Reset 0x1100 xxxx
#define	NOP		0x00	//
//----------------------------------------------------------------
#define	FORCEMAX	20	//


class ADS1258
{
	
public:

	//Constructor
	ADS1258(const byte DRDY_pin, const byte CS_pin);
	
	//Initializing function
	void InitializeADC();	
	//ADS1258(int drate, int pga, int byteOrder, bool bufen);
	void acquireBasePara();
	//Read a register
	long readRegister(uint8_t registerAddress);
	
	//Write a register
	void writeRegister(uint8_t registerAddress, uint8_t registerValueToWrite);	

	//Individual methods
	// void setDRATE(uint8_t drate);
	// void setPGA(uint8_t pga);
	// uint8_t getPGA();
	void setMUX(uint8_t mux);
	// void setByteOrder(uint8_t byteOrder);
	// void getByteOrder();
	// void setBuffer(uint8_t bufen);
	// void getBuffer();
	// void setAutoCal(uint8_t acal);
	// void getAutoCal();
	// void setGPIO(uint8_t dir0, uint8_t dir1, uint8_t dir2, uint8_t dir3);
	// void writeGPIO(uint8_t dir0value, uint8_t dir1value, uint8_t dir2value, uint8_t dir3value);
	// uint8_t readGPIO(uint8_t gpioPin);	
	// void setCLKOUT(uint8_t clkout);
	// void setSDCS(uint8_t sdcs);	
	void sendDirectCommand(uint8_t directCommand);	

	//Get a single conversion
	// long readSingle();
	long readDifferential();
	long readDataFromChannel(byte channelx );
	//Single input continuous reading
	// long readSingleContinuous();
	
	//Cycling through the single-ended inputs
	// long cycleSingle(); //Ax + COM
	
	//Cycling through the differential inputs
	// long cycleDifferential(); //Ax + Ay
		
	//Converts the reading into a voltage value
	float convertToVoltage(int32_t rawData);
		
	//Stop AD
	void stopConversion();
	
	long baseValue(byte channelx,int avg_times);
	float computeSGGain(byte channelx,int avg_times);
	float meassureForce(byte channelx);
	float meassureTimesForce(byte channelx,int avg_times);
	void setGain(float *sg_gains);
	float getGain(byte channelx);
	long getBase(byte channelx);
	void computeBaseValue(int avg_times);
	void setChannels(byte *channels);
	void resetChip();
	float* readChannelsForce(int avg_times);

	float _vref; //Value of the reference voltage
	float _gain;
	// float _tempr;
	float _vcc;




private:

//Pins
byte _DRDY_pin; //Pin assigned for DRDY
byte _RESET_pin; //Pin assigned for RESET
byte _SYNC_pin; //Pin assigned for SYNC
byte _CS_pin; //Pin assigned for CS

//Register-related variables
uint8_t _registerAddress; //Value holding the address of the register we want to manipulate
uint8_t _registerValueToWrite; //Value to be written on a selected register
uint8_t _registerValuetoRead; //Value read from the selected register

//Register values
byte _CONFIG0; 
byte _CONFIG1;
byte _MUXSCH; 
byte _MUXDIF; 
byte _MUXSG0; 
byte _MUXSG1; 
byte _GPIOC; 
byte _GPIOD; 
byte _SYSRED; 
byte _PGA=0; 

byte _outputBuffer[3]; //3-byte (24-bit) buffer for the fast acquisition - Single-channel, continuous
long _outputValue; //Combined value of the _outputBuffer[3]
// bool _isAcquisitionRunning; //bool that keeps track of the acquisition (running or not)
uint8_t _cycle; //Tracks the cycles as the MUX is cycling through the input channels

long _sg_base[8]={0,0,0,0,0,0,0,0};
float _sg_gain[8]={1,1,1,1,1,1,1,1};
byte _channels[6]={1,2,3,4,5,6};
float* _force_data;

};


#endif
