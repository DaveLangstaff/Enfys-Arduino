# Enfys-Arduino

Arduino code for SCPI instrument to control detector board

Dave Langstaff dpl@aber.ac.uk


Code runs on Arduino nano esp 32


Commands:

  *IDN? - Gets the instrument's identification string
    
  *DBG? - Print out debug information on SCIP parser and INA bus monitors
    
  DOut# xxx - Sets DAC # to value xxx, where # is in the range of 0..1 and xxx is in the range 0..4095
    
  AIn#? - Reads ADC value from ADC channel #, where # is in the range 0..7. Value returned is in the range 0..4095
    This is the sum of a buffer full of readings, as set by OS
    
  OS xxx - Sets oversampling to 2^xxx, where x is in the range 0 to 10 corresponding to oversampling of 1,2,4,8...1024
  OS? -    Returns current level of oversampling
    
  POwer:ON - Turns power off
  
  POwer:OFF - Turns power on
  
  BURST#? - takes reading and returns whole dataBuffer, length as set by OS
  
  TIME? - returns time in usec for last read operation
    
  THROW xxx - Sets number of readings to throw away at start of each burst
   
  THROW? - Gets number of readings to throw away at start of each burst

  CURR#? - Returns current on bus number # (3v3, +12V, Heater, -12V)

  VBUS#? - Returns current on bus number # (3v3, +12V, Heater, -12V) 

  NAME#? - Returns name of INA device used for monitoring bus#

  DELAY xxx - Sets time in usec to delay after each reading

  DELAY? - Returns time in usec to delay after each reading

  HTR: ON - turn on board heater current

  HTR: OFF - turn off board heater current

  HTR:DAC xxx - set heater DAC (0..4095)

  HTR:DAC? - get heater DAC (0..4095)
  
  CLK xxx  -  set SPI clock value in Hz (100,000 - 16,000,000)

  CLK?   -  get SPI clock value in Hz (100,000 - 16,000,000)

  RTD:TEMP? - get temperature from RTD controller
  
  
