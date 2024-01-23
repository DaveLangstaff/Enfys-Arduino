/*
Vrekrer_scpi_parser library.
Control code for Enfyf Detector board
uses Vrekrer SCPI library to parse commands

Commands:
  *IDN?
    Gets the instrument's identification string
  DOut# xxx
    Sets DAC # to value xxx, where # is in the range of 0..1 and xxx is in the range 0..4095
  AIn#?
    Reads ADC value from ADC channel #, where # is in the range 0..7. Value returned is in the range 0..4095
  OS xxx
    Sets oversampling to 2^xxx, where x is in the range 0 to 10 corresponding to oversampling of 1,2,4,8...1024
  OS?
    Returns current level of oversampling
*/


#include "Arduino.h"
#include "Vrekrer_scpi_parser.h"
//#include "ADC128S102.h"

#include <SPI.h>

#define SPICLOCK 13//sck
#define DATAIN   12//CIPO
#define DATAOUT  11//COPI
#define ADC_CS  10//cs
#define DAC1_CS  9 //cs
#define DAC0_CS  8 //cs
#define TRIG   7 // Trigger for scope

#define ClockSpeed 1000000   //1MHz SPI clock
#define serialSpeed 115200     // serial baud rate
unsigned int DAC0_value=0;
unsigned int DAC1_value=0;
byte OSLevel=0;
unsigned int OSvalue=1;


SCPI_Parser my_instrument;

void setup()
{
 my_instrument.RegisterCommand(F("*IDN?"), &Identify);

  //Use "#" at the end of a token to accept numeric suffixes.
  my_instrument.RegisterCommand(F("AIn#?"), &ReadADC);
  //my_instrument.RegisterCommand(F("DOut#"), &WriteDAC);
  my_instrument.RegisterCommand(F("DOut#"), &WriteDAC);
  my_instrument.RegisterCommand(F("DOut#?"), &ReadDAC);
  my_instrument.RegisterCommand(F("OS"), &setOS);  
  my_instrument.RegisterCommand(F("OS?"), &getOS);  
  
  Serial.begin(serialSpeed);
  pinMode(DATAOUT, OUTPUT); 
  pinMode(SPICLOCK,OUTPUT);
  SPI.begin();

  pinMode(DAC0_CS,OUTPUT); digitalWrite(DAC0_CS,HIGH);
  pinMode(DAC1_CS,OUTPUT); digitalWrite(DAC1_CS,HIGH);
  pinMode(TRIG,OUTPUT);  digitalWrite(TRIG,HIGH);
  pinMode(ADC_CS,OUTPUT);  digitalWrite(ADC_CS,HIGH);
  pinMode(DATAIN, INPUT);

}

void loop()
{
  my_instrument.ProcessInput(Serial, "\n");
}

void Identify(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  interface.println(F("Aberystwyth University,Enfys Detector,#01," VREKRER_SCPI_VERSION));
  //*IDN? Suggested return string should be in the following format:
  // "<vendor>,<model>,<serial number>,<firmware>"
  trigger();
}

void DoNothing(SCPI_C commands, SCPI_P parameters, Stream& interface) {
}

void trigger() {
  digitalWrite(TRIG, LOW);  //put negative pulse on Trigger line
  digitalWrite(TRIG, HIGH);
}

void setOS(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  int param = -1;
  String first_parameter = String(parameters.First());
  sscanf(first_parameter.c_str(),"%d",&param) ;
  param=constrain(param,0,10);
  OSLevel = byte(param);
  OSvalue = (1 << OSLevel);
}

void getOS(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  interface.println(OSLevel);
}

void WriteDAC(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = -1;
  sscanf(header.c_str(),"%*[DOUT]%u", &suffix);

  //If the suffix is valid,
  if ( (suffix >= 0) && (suffix < 2) ) {
    //use the first parameter (if valid) to set the digital Output
    int param = -1;
    String first_parameter = String(parameters.First());
    sscanf(first_parameter.c_str(),"%d",&param) ;
    param=constrain(param,0,4095);
    uint8_t byteLow = param & 0xff;
    uint8_t byteHigh = ((param >> 8) & 0x0f);
    trigger();
    SPI.beginTransaction(SPISettings(ClockSpeed, MSBFIRST, SPI_MODE1));
      if (suffix==0) {
      digitalWrite(DAC0_CS, LOW);
      DAC0_value=param;
    }
    else {
      digitalWrite(DAC1_CS, LOW);
      DAC1_value=param;
    }
    delayMicroseconds(10);
    SPI.transfer(byteHigh);
    SPI.transfer(byteLow);
    if (suffix==0) {
      digitalWrite(DAC0_CS, HIGH);
    }
    else {
      digitalWrite(DAC1_CS, HIGH);
    }
    delayMicroseconds(5);
    SPI.endTransaction();
/*    interface.print("DAC");interface.print(suffix);
    interface.print(", Value: ");interface.print("0x");
    interface.print(byteHigh,HEX); interface.print("-"); interface.print(byteLow,HEX);
    interface.print("  ");
    interface.println(param);
*/
  }
}

void ReadDAC(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = -1;
  int param=-1;
  sscanf(header.c_str(),"%*[DOUT]%u?", &suffix);

  //If the suffix is valid,
  if ( (suffix >= 0) && (suffix < 2) ) {
        if (suffix==0) {
          param=DAC0_value;
          }
        else {
          param=DAC1_value;
          }
  interface.println(param);
  }
}

void ReadADC(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  unsigned long reading;
  uint16_t buffer;
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int chan = -1;
  sscanf(header.c_str(),"%*[AIN]%u", &chan);
  //If the suffix is valid,
  if ( (chan >= 0) && (chan < 8) ) {
    //use the first parameter (if valid) to set the digital Output  
    // read from ADC128 chip. Transfers channel number to the chip first and reads back data
    trigger();
    SPI.beginTransaction(SPISettings(ClockSpeed, MSBFIRST, SPI_MODE0));
    reading=0;
    for (int i=0;i<OSvalue;i++){
// Read for 16 bit transfer
        digitalWrite(ADC_CS, LOW);
        uint16_t control = chan << 11;
        buffer = SPI.transfer16(control);
        digitalWrite(ADC_CS,HIGH);
//
/* Read for 8 bit transfer
        digitalWrite(ADC_CS, LOW);
        uint16_t control = chan << 3;
        buffer = SPI.transfer(control);
        buffer = buffer << 8;
        buffer = buffer | SPI.transfer(0);
        digitalWrite(ADC_CS,HIGH);
*/
        reading += buffer;
    }
//    interface.print("AI");interface.print(chan); interface.print(" :");
    interface.println(reading);
//    interface.println(micros());
    SPI.endTransaction();
  }
}

