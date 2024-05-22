#include "Arduino.h"
#include "ADS1258.h"
#include "SPI.h"

//----
volatile bool DRDY_value = false; //It is chosen to be false (HIGH) because that's the default state when a conversion is completed

void DRDY_ISR() 
{
	DRDY_value = true; //Change the DRDY_value to true (LOW). (The ISR was fired by a FALLING signal)
	// Serial.print("DRDY_ISR");

}


void waitForDRDY() //A function that waits for the DRDY status to change via the ISR()
{		
	// Serial.print("DRDY before:");
	// Serial.println(DRDY_value);
	while (DRDY_value == false) {}
	noInterrupts();
	// Serial.print("DRDY after:");
	// Serial.println(DRDY_value);
	DRDY_value = false; //Reset the DRDY_value manually to false while disabling the interrupts
	interrupts();	
}

//----

//Constructor
ADS1258::ADS1258(const byte DRDY_pin, const byte CS_pin)
{
	_DRDY_pin = DRDY_pin; 
	pinMode(_DRDY_pin, INPUT);	

	_CS_pin = CS_pin;
	pinMode(_CS_pin, OUTPUT);
	
}
	

//Initialization
void ADS1258::InitializeADC()
{
	//Chip select LOW  
	digitalWrite(_CS_pin, LOW);
	
	//We do a manual chip reset on the ADS1258 - Datasheet Page 27/ RESET
	// if(_RESET_pin != 0)
	// {
	// digitalWrite(_RESET_pin, LOW);
	// delay(200);
	// digitalWrite(_RESET_pin, HIGH); //RESET is set to high
	// delay(1000);
	// }
	resetChip();

  //Sync pin is also treated if it is defined
	// if(_SYNC_pin != 0)
	// {
	// 	digitalWrite(_SYNC_pin, HIGH); //RESET is set to high  
	// }

	// SPI.setClockDivider(SPI_CLOCK_DIV4);
	
	// SPI.begin();


	attachInterrupt(digitalPinToInterrupt(_DRDY_pin), DRDY_ISR, FALLING); //Make the interrupt fire on FALLING signal	 

	//Applying arbitrary default values to speed up the starting procedure if the user just want to get quick readouts
	//We both pass values to the variables and then send those values to the corresponding registers
	// delay(200);

	// sendDirectCommand(0B01110000);
	// delay(200);
	// Serial.println("0");

	_CONFIG0 = 0b00000010; 
	writeRegister(CONFIG0_REG, _CONFIG0); 
	delay(5);

	_CONFIG1 = 0b00000011; 
	writeRegister(CONFIG1_REG, _CONFIG1); 
	delay(5);

	_MUXSCH = 0b00000000; 
	writeRegister(MUXSCH_REG, _MUXSCH); 
	delay(5);

	_MUXDIF = 0b01111110; 
	writeRegister(MUXDIF_REG, _MUXDIF); 
	delay(5);

	_MUXSG0 = 0b00000000; 
	writeRegister(MUXSG0_REG, _MUXSG0); 
	delay(5);

	_MUXSG1 = 0b00000000; 
	writeRegister(MUXSG1_REG, _MUXSG1); 
	delay(5);

	_SYSRED = 0b00110100;
	writeRegister(SYSRED_REG, _SYSRED); 
	delay(5);

	_GPIOC = 0b00000000; 
	writeRegister(GPIOC_REG, _GPIOC); 
	delay(5);

	_GPIOD = 0b00000000; 
	writeRegister(GPIOD_REG, _GPIOD); 
	delay(5);

//   _isAcquisitionRunning = false; //MCU will be waiting to start a continuous acquisition

}

void ADS1258::acquireBasePara()
{
	_vref = readDataFromChannel(0x1D)/786432.0;
	// Serial.println(" _vref: ");

	_vcc = readDataFromChannel(0x1A)/786432.0;
	// Serial.println(" _vcc: ");
	_gain = readDataFromChannel(0x1C)/7864320.0;
	// Serial.println(" _gain: ");

}




void ADS1258::stopConversion() //Sending SDATAC to stop the continuous conversion
{	
	waitForDRDY(); //SDATAC should be called after DRDY goes LOW (p35. Figure 33)
	SPI.transfer(0b00001111); //Send SDATAC to the ADC	
	digitalWrite(_CS_pin, HIGH); //We finished the command sequence, so we switch it back to HIGH
	SPI.endTransaction();
	
	// _isAcquisitionRunning = false; //Reset to false, so the MCU will be able to start a new conversion
}


void ADS1258::setMUX(uint8_t mux) //Setting MUX (input channel)
{	
	writeRegister(MUXDIF_REG, mux);
	_MUXDIF = mux;
	delay(200);
}

void ADS1258::sendDirectCommand(uint8_t directCommand)
{
  //Direct commands can be found in the datasheet Page 34, Table 24.
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  digitalWrite(_CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"
  delayMicroseconds(5);
  SPI.transfer(directCommand); //Send Command
  delayMicroseconds(5);
  digitalWrite(_CS_pin, HIGH); //REF: P34: "CS must stay low during the entire command sequence"

  SPI.endTransaction();
}


float ADS1258::convertToVoltage(int32_t rawData) //Converting the 24-bit data into a voltage value
{	
  if (rawData >> 23 == 1) //if the 24th digit (sign) is 1, the number is negative
  {
    rawData = rawData - 16777216;  //conversion for the negative sign
    //"mirroring" around zero
  }  
  
  float voltage = ((float)rawData/(float)0x780000)*_vref/_gain;
  // float voltage = ((2 * _vref) / 8388608) * rawData / (pow(2, _PGA)); //8388608 = 2^{23} - 1
  //REF: p25, Table 10.
	
  return voltage;
}

void ADS1258::writeRegister(uint8_t registerAddress, uint8_t registerValueToWrite)
{	
  waitForDRDY();

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  //SPI_MODE0 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.

  digitalWrite(_CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]

  delayMicroseconds(5); //see t6 in the datasheet

  SPI.transfer(WREG | registerAddress); // 0x50 = 01010000 = WREG

  // SPI.transfer(0x00); //ADS1258 no need for this line

  SPI.transfer(registerValueToWrite); //pass the value to the register
  
  digitalWrite(_CS_pin, HIGH);
  SPI.endTransaction();

  delay(100);
  
}

long ADS1258::readRegister(uint8_t registerAddress) //Reading a register
{
	waitForDRDY();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  //SPI_MODE0 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.

  digitalWrite(_CS_pin, LOW); //CS must stay LOW during the entire sequence [Ref: P34, T24]

  SPI.transfer(RREG | registerAddress); //0x10 = 0001000 = RREG - OR together the two numbers (command + address)

//   SPI.transfer(0x00); //ADS1258 no need for this line

  delayMicroseconds(1); //see t6 in the datasheet

  _registerValuetoRead = SPI.transfer(0x30); //read out the register value

  digitalWrite(_CS_pin, HIGH);
  SPI.endTransaction();
  delay(100);
  return _registerValuetoRead;
}



long ADS1258::readDifferential()
{
	SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
	digitalWrite(_CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"  
	waitForDRDY();
	SPI.transfer(RDATA); //Issue RDATA (0000 0001) command
	// delayMicroseconds(2); //Wait t6 time (~6.51 us) REF: P34, FIG:30.

  	byte stutes = SPI.transfer(NOP);
	// Serial.print("stutes: ");
	// Serial.println(stutes & 0x1F);
	_outputBuffer[0] = SPI.transfer(0); // MSB
	_outputBuffer[1] = SPI.transfer(0); // Mid-byte
	_outputBuffer[2] = SPI.transfer(0); // LSB		

	//Shifting and combining the above three items into a single, 24-bit number
	_outputValue = ((long)_outputBuffer[0]<<16) | ((long)_outputBuffer[1]<<8) | (_outputBuffer[2]);
	
	digitalWrite(_CS_pin, HIGH); //We finished the command sequence, so we set CS to HIGH
	SPI.endTransaction();
  
	return _outputValue;
}

long ADS1258::readDataFromChannel(byte channelx )
{
	SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
	digitalWrite(_CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"  
	// SPI.transfer(RDATA); //Issue RDATA (0000 0001) command
	// delayMicroseconds(2); //Wait t6 time (~6.51 us) REF: P34, FIG:30.

	byte channel = 0x20;
	byte status;

	while(channel != channelx || status != 4 || _outputValue == 0){
		waitForDRDY();
		channel = SPI.transfer(NOP);
		status = channel >> 5;
		channel = channel & 0x1F;

		if (channel == channelx && status == 4)
		{
			_outputBuffer[0] = SPI.transfer(0); // MSB
			_outputBuffer[1] = SPI.transfer(0); // Mid-byte
			_outputBuffer[2] = SPI.transfer(0); // LSB		
			_outputValue =  ((long)_outputBuffer[0]<<16) | ((long)_outputBuffer[1]<<8) | (_outputBuffer[2]);
			
		}
		

		// Serial.print("channel: ");
		// Serial.print(channel);
		// Serial.print(" status: ");
		// Serial.print(status);
		// Serial.print(" _outputValue: ");
		// Serial.print(_outputValue);
		// Serial.println(" ");
		

	}


	if (_outputValue >> 23 ==1)
	{
		_outputValue = 65535 + _outputValue - 16777216;
	}else
		_outputValue = 65535 + _outputValue;
	
	// Serial.print("status: ");
	// Serial.print(status);
	// Serial.print("   channel: ");
	// Serial.println(channel);
	//Shifting and combining the above three items into a single, 24-bit number

	digitalWrite(_CS_pin, HIGH); //We finished the command sequence, so we set CS to HIGH
	SPI.endTransaction();

	return _outputValue;
}

long ADS1258::baseValue(byte channelx,int avg_times)
{

	_sg_base[channelx] = 0;
	readDataFromChannel(channelx);
	bool flag_ini = true;
	long max,min;
	for (int i = 0; i < avg_times; i++)
	{
		long tmp = readDataFromChannel(channelx);
		_sg_base[channelx] = _sg_base[channelx] + tmp;

		if (flag_ini)
		{
			max = tmp;
			min = tmp;
		}else{
			if(tmp > max)
				max=tmp;
			else if(tmp < min)
				min=tmp;
		}

		// Serial.print(" tmp: ");
		// Serial.println(tmp);
	}
	// Serial.print(" max: ");
	// Serial.print(max);
	// Serial.print(" min: ");
	// Serial.print(min);
	// Serial.println(" ");
	// _sg_base[channelx] = (_sg_base[channelx])/(avg_times);
	_sg_base[channelx] = (_sg_base[channelx] - max - min)/(avg_times-2);

	return _sg_base[channelx];
}

float ADS1258::computeSGGain(byte channelx,int avg_times)
{

	long _sg_0p5kg = 0;
	for (int i = 0; i < avg_times; i++)
	{
		_sg_0p5kg = _sg_0p5kg + readDataFromChannel(channelx);
	}
	_sg_0p5kg = _sg_0p5kg/avg_times;

	_sg_gain[channelx] = ((float)_sg_0p5kg - (float)_sg_base[channelx])/5.0;
	// Serial.print("_sg_0p5kg: ");
	// Serial.println((float)_sg_0p5kg);
	return _sg_gain[channelx];
}

float ADS1258::meassureForce(byte channelx)
{

	long force_data = readDataFromChannel(channelx);


	return (float)((force_data - _sg_base[channelx])/_sg_gain[channelx]);
}

void ADS1258::setGain(float *sg_gains)
{
	// Serial.print("Gain: ");
	for (int i = 0; i < 6; i++)
	{
		_sg_gain[_channels[i]] = sg_gains[i];
		// Serial.print(_channels[i]);
		// Serial.print(" ");
		// Serial.println((float)_sg_gain[_channels[i]] );
		// Serial.print(" ");
	}
}
float ADS1258::getGain(byte channelx)
{
	return _sg_gain[channelx];
}
long ADS1258::getBase(byte channelx)
{
	return _sg_base[channelx];
}
void ADS1258::computeBaseValue(int avg_times)
{
	for (int j = 0; j < 6; j++)
	{
		_sg_base[_channels[j]] = baseValue(_channels[j],avg_times);
	}
}
void ADS1258::setChannels(byte *channels)
{
	// Serial.print("channels: ");
	for (int i = 0; i < 6; i++)
	{
		_channels[i] = channels[i];
		// Serial.print(channels[i]);
		// Serial.print(" ");
		// Serial.println((float)_channels[i] );
		// Serial.print(" ");
	}
}

void ADS1258::resetChip()
{
	SPI.end();
	delay(100);

	SPI.setClockDivider(SPI_CLOCK_DIV4);
	SPI.begin();
	delay(50);
  	sendDirectCommand(RESET);
	delay(100);

}

float ADS1258::meassureTimesForce(byte channelx,int avg_times)
{

	long force_data =0;
	// long read_data[avg_times];
	readDataFromChannel(channelx);
	for (int i = 0; i < avg_times; i++)
	{
		// read_data[i] = readDataFromChannel(channelx);
		// force_data = force_data + read_data[i];
		force_data = force_data + readDataFromChannel(channelx);
	}
	force_data = force_data/avg_times;
	return (float)((force_data - _sg_base[channelx])/_sg_gain[channelx]);

	// float data = (float)((force_data - _sg_base[channelx])/_sg_gain[channelx]);

	// if (abs(data)>50)
	// {
	// 	Serial.print("Data: ");
	// 	for (int i = 0; i < avg_times; i++)
	// 	{
	// 		Serial.print(" ");
	// 		Serial.print(read_data[i]);
	// 	}
		
	// }
	
	// return data;
}

float* ADS1258::readChannelsForce(int avg_times) //Channels: 1-6
{
	bool flag[6] = {0,0,0,0,0,0};
	int data_acquire_cnt[6] = {0,0,0,0,0,0};
	float data_arr[6] = {0,0,0,0,0,0};
	_force_data = data_arr;
	byte channel = 0x20;
	byte status =0;
	bool flag_while = false;

	float data_max[6] = {0,0,0,0,0,0};
	float data_min[6] = {0,0,0,0,0,0};

	SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
	digitalWrite(_CS_pin, LOW); //REF: P34: "CS must stay low during the entire command sequence"  

	while(!flag_while){
		waitForDRDY();
		channel = SPI.transfer(NOP);
		status = channel >> 5;
		channel = channel & 0x1F;

		if (channel <=_channels[5] && channel >=_channels[0] && status == 4 ){

			
			if (!flag[channel-1]){

				_outputBuffer[0] = SPI.transfer(0); // MSB
				_outputBuffer[1] = SPI.transfer(0); // Mid-byte
				_outputBuffer[2] = SPI.transfer(0); // LSB		
				_outputValue =  ((long)_outputBuffer[0]<<16) | ((long)_outputBuffer[1]<<8) | (_outputBuffer[2]);
				
				if (_outputValue >> 23 ==1)
					_outputValue = 65535 + _outputValue - 16777216;
				else
					_outputValue = 65535 + _outputValue;

				float force = (float)((_outputValue - _sg_base[channel])/_sg_gain[channel]);
				if (abs(force) <=FORCEMAX)
				{
					_force_data[channel-1] = _force_data[channel-1] + force;

					if (data_acquire_cnt[channel-1]==0)
					{
						data_max[channel-1]=force;
						data_min[channel-1]=force;
					}else{
						if(force > data_max[channel-1])
							data_max[channel-1]=force;
						else if(force < data_min[channel-1])
							data_min[channel-1]=force;
					}
					


					data_acquire_cnt[channel-1]++;

					if (data_acquire_cnt[channel-1] >= avg_times)
						flag[channel-1] = 1;
				}
				
				// Serial.print("channel: ");
				// Serial.print(channel);
				// Serial.print(" status: ");
				// Serial.print(status);
				// Serial.print(" _outputValue: ");
				// Serial.print(_outputValue);
				// Serial.print(" force: ");
				// Serial.print(force);
				// Serial.print(" _sg_base: ");
				// Serial.print(_sg_base[channel]);
				// Serial.println(" ");
	
			}

		}

		if (flag[0]==1 && flag[1]==1 && flag[2]==1 && flag[3]==1 && flag[4]==1 && flag[5]==1)
		{
			flag_while = true;
		}
			

	}

	for (int i = 0; i < 6; i++)
	{
		_force_data[i] = (_force_data[i]-data_min[i] -data_max[i])/(avg_times-2);
		// Serial.print(i);
		// Serial.print(" max: ");
		// Serial.print(data_max[i]);
		// Serial.print(" min: ");
		// Serial.print(data_min[i]);
		// Serial.println(" ");
	}
	

	digitalWrite(_CS_pin, HIGH); //We finished the command sequence, so we set CS to HIGH
	SPI.endTransaction();

	return _force_data;

}
