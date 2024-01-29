# Enfys-Arduino

Arduino code for SCPI instrument to control detector board

Dave Langstaff dpl@aber.ac.uk

Code runs on Arduino nano esp 32

Commands:
  *IDN?
    Gets the instrument's identification string
  DOut# xxx
    Sets DAC # to value xxx, where # is in the range of 0..1 and xxx is in the range 0..4095
  AIn#?
    Reads ADC value from ADC channel #, where # is in the range 0..7. Value returned is in the range 0..4095
    This is the sum of a buffer full of readings, as set by OS
  OS xxx
    Sets oversampling to 2^xxx, where x is in the range 0 to 10 corresponding to oversampling of 1,2,4,8...1024
  OS?
    Returns current level of oversampling
  PO:ON Turns power off
  PO:OFF Turns power on
  BURST#? - takes reading and returns whole dataBuffer, length as set by OS
  TIME? - returns time in usec for last read operation