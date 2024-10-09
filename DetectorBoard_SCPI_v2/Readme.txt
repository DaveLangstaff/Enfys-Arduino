
Detectorboard EGSE commands

Deafult baud rate: 115200
All commands temrinated with cr-lf

Identify Command
*IDN?    - return identifier string. Eg "Aberystwyth University,Enfys Detector,#01,#05"

Debug Command
*DBG?	 - returns debugging information on current monitors installed

Analog Input
AIn#?    - return data from analog inputs (ADC convertor) # is channel 0-7, channel values outside the range 0-7 will return ERR_BAD_SUFFIX -4
		 - This is a summation of OS individual readings and is returned as an unsigned long int

Set Offset DAC		 
DOut# param	 - sets DAC on channel # (where channel 0=SWIR, 1=MWIR, any other # returns ERR_BAD_SUFFIX -4)
             - param is in range 0-4095, values outside this range return ERR_BAD_PARAM -5


Oversampling
OS param	 - set oversampling to param, where param is in the range 1-4096 values outside this range return ERR_BAD_PARAM -5
OS?			 - returns current value of oversampling

Clock speed
CLK param    - sets SPI clock speed. Param should be in the range of 1,000,000 to 16,000,000 values outside this range return ERR_BAD_PARAM -5
CLK?         - return current SPI clock speed

Current and Voltage monitoring
CURR#?			return current from supply #. 
					# is in the range 0-3 values outside the range 0-3 will return ERR_BAD_SUFFIX -4
					0: 3.3V supply
					1: Heater supply
					2: +12V supply
					3: -12V supply
				
VBUS#?			return bus voltage (in mV) from supply #
					# is in the range 0-3 values outside the range 0-3 will return ERR_BAD_SUFFIX -4
					0: 3.3V supply
					1: Heater supply
					2: +12V supply
					3: -12V supply
					
NAME#?			return device name from supply #
					# is in the range 0-3 values outside the range 0-3 will return ERR_BAD_SUFFIX -4
					0: 3.3V supply INA3221
					1: Heater supply INA3221
					2: +12V supply INA3221
					3: -12V supply INA260
					


					
  my_instrument.RegisterCommand(F("NAME#?"), &getName);
  my_instrument.RegisterCommand(F("TIME?"), &getElapsed);
  my_instrument.RegisterCommand(F("BURST#?"), &ReadADCBurst);
  my_instrument.RegisterCommand(F("DELAY"), &setSampleDelay);
  my_instrument.RegisterCommand(F("DELAY?"), &getSampleDelay);
  my_instrument.RegisterCommand(F("THROW"), &setThrowAway);
  my_instrument.RegisterCommand(F("THROW?"), &getThrowAway);
  my_instrument.SetCommandTreeBase(F("HTR"));
  my_instrument.RegisterCommand(F(":ON"), &HTROn);
  my_instrument.RegisterCommand(F(":OFF"), &HTROff);
  my_instrument.RegisterCommand(F(":DAC"), &setHTRDAC);
  my_instrument.RegisterCommand(F(":DAC?"), &getHTRDAC);
  my_instrument.SetCommandTreeBase(F("POwer"));
  my_instrument.RegisterCommand(F(":ON"), &PowerOn);
  my_instrument.RegisterCommand(F(":OFF"), &PowerOff);
  my_instrument.SetCommandTreeBase(F("RTD"));
  my_instrument.RegisterCommand(F(":TEMP?"), &getRTDTemperature);