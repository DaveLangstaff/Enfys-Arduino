# Enfys-Arduino

Arduino code for SCPI instrument to control detector board

Dave Langstaff dpl@aber.ac.uk


Code runs on Arduino nano esp 32
Detectorboard EGSE commands
===========================
Deafult baud rate: 57600
All commands temrinated with cr-lf

Identify Command
================
*IDN?    - return identifier string. Eg "Aberystwyth University,Enfys Detector,#01,#05"

Debug Command
=============
*DBG?	 - returns debugging information on current monitors installed

Analog Input
============
AIn#?    - return data from analog inputs (ADC convertor) # is channel 0-7, channel values outside the range 0-7 will return ERR_BAD_SUFFIX -4
		 - This is a summation of OS individual readings and is returned as an unsigned long int

Set Offset DAC		 
==============
DOut# param	 - sets DAC on channel # (where channel 0=SWIR, 1=MWIR, any other # returns ERR_BAD_SUFFIX -4)
             - param is in range 0-4095, values outside this range return ERR_BAD_PARAM -5

Oversampling
============
OS param	 - set oversampling to param, where param is in the range 1-4096 values outside this range return ERR_BAD_PARAM -5
OS?			 - returns current value of oversampling

Clock speed
===========
CLK param    - sets SPI clock speed. Param should be in the range of 1,000,000 to 16,000,000 values outside this range return ERR_BAD_PARAM -5
CLK?         - return current SPI clock speed

Current and Voltage monitoring
==============================
CURR#?			return current from supply #.

					# is in the range 0-3 values outside the range 0-3 will return ERR_BAD_SUFFIX -4
					0: 3.3V supply
					
					1: Heater supply
					
					2: +12V supply
					
					3: -12V supply
					
				

VBUS#?			return bus voltage (in mV) from supply 
#
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


TIME?		- gives time in usec for last read operation

BURST#?		- reads OS readings into buffer and returns whole databuffer

SCIENCE? -  reads all 8 ADC channels according to current OS and THROW parameters. Sums readings and then shifts to discard lower 4 bits
         - returns all 8 readings in a comma seperated line

DELAY xxxx 	- sets delay between SPI reads in usec
DELAY?		- returns delay between SPI reads in usec

THROW xxxx	- sets number of readings to throw away at start of each read cycle (0..1024)
THROW?		- returns number of readings to throw away at start of each read cycle (0..1024)					

HTR:ON		- enable on board heater supply
HTR:OFF		- disable on-board heater supply

POwer:ON	- Enable power to detector board
POwer:OFF	- Disable power to detector board

RTD:TEMP?	- get temperature from RTD device
  all commands will return either the requested data (? commands) 
  or an error code indicating successful or otherwise completion
