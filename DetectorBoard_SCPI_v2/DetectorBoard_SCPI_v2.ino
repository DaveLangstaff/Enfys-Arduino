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
    This is the sum of a buffer full of readings, as set by OS
  OS xxx
    Sets oversampling to xxx where xxx is the number of samples to take, in the range 1..1024
  OS?
    Returns current level of oversampling
  PO:ON Turns power off
  PO:OFF Turns power on
  BURST#? - takes reading and returns whole dataBuffer, length as set by OS
  TIME? - returns time in usec for last read operation
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

#define POWER3V3 A7 // enable pin for 3v3 power
#define POWER12V A6 // enable pin for 12v power

#define ClockSpeed 1000000   //1MHz SPI clock
#define serialSpeed 115200     // serial baud rate
unsigned int DAC0_value=0;
unsigned int DAC1_value=0;
//byte OSLevel=0;
unsigned int OSvalue=1;
unsigned int sampleDelay=0;

unsigned int dataBuffer[1024];
unsigned long Elapsed;

SCPI_Parser my_instrument;

void setup()
{
 my_instrument.RegisterCommand(F("*IDN?"), &Identify);

  //Use "#" at the end of a token to accept numeric suffixes.
  my_instrument.RegisterCommand(F("AIn#?"), &ReadADC);
  //my_instrument.RegisterCommand(F("DOut#"), &WriteDAC);
  my_instrument.RegisterCommand(F("DOut#"), &WriteDAC);
  my_instrument.RegisterCommand(F("OS"), &setOS);  
  my_instrument.RegisterCommand(F("OS?"), &getOS);
  my_instrument.RegisterCommand(F("POwer:ON"), &PowerOn);
  my_instrument.RegisterCommand(F("POwer:OFF"), &PowerOff);
  my_instrument.RegisterCommand(F("TIME?"), &getElapsed);
  my_instrument.RegisterCommand(F("BURST#?"), &ReadADCBurst);
  my_instrument.RegisterCommand(F("DELAY"), &setsampleDelay);
  my_instrument.RegisterCommand(F("DELAY?"), &getsampleDelay);
    
  
  Serial.begin(serialSpeed);

  // setup SPI bus
  pinMode(DATAOUT, OUTPUT); 
  pinMode(SPICLOCK,OUTPUT);
  pinMode(DAC0_CS,OUTPUT); digitalWrite(DAC0_CS,HIGH);
  pinMode(DAC1_CS,OUTPUT); digitalWrite(DAC1_CS,HIGH);
  pinMode(TRIG,OUTPUT);  digitalWrite(TRIG,HIGH);
  pinMode(ADC_CS,OUTPUT);  digitalWrite(ADC_CS,HIGH);
  pinMode(DATAIN, INPUT);
  SPI.begin();
    
  // setup pins for power rail control
  pinMode(POWER3V3, OUTPUT); digitalWrite(POWER3V3, LOW);
  pinMode(POWER12V, OUTPUT); digitalWrite(POWER12V, LOW);
  pinMode(LED_BLUE, OUTPUT); digitalWrite(LED_BLUE, HIGH); // N LED drive is inverted

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

void PowerOn(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    digitalWrite(POWER12V, HIGH);     // turn ON 12V power
    delay(2);
    digitalWrite(POWER3V3, HIGH);    // turn ON 3v3 power
    digitalWrite(LED_BLUE, LOW);
}

void PowerOff(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    digitalWrite(POWER3V3, LOW);    // turn OFF 3v3 power
    delay(1);                       // wait 1ms
    digitalWrite(POWER12V, LOW);    // turn OFF 12v power
    digitalWrite(LED_BLUE, HIGH);
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
  param=constrain(param,1,1024);
  OSvalue = param;
}

void getOS(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  interface.println(OSvalue);
}

void setsampleDelay(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  int param = -1;
  String first_parameter = String(parameters.First());
  sscanf(first_parameter.c_str(),"%d",&param) ;
  sampleDelay=constrain(param,0,1023);
}

void getElapsed(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  interface.println(Elapsed);
}

void getsampleDelay(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  interface.println(sampleDelay);
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
    SPI.transfer16(param);
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

void ReadADCtoBuffer(int chan){
    trigger();
    SPI.beginTransaction(SPISettings(ClockSpeed, MSBFIRST, SPI_MODE0));
    unsigned long startTime = micros();
    digitalWrite(ADC_CS, LOW);
    for (int i=0;i<OSvalue;i++){
        uint16_t control = chan << 11;
        dataBuffer[i] = SPI.transfer16(control);
        delayMicroseconds(sampleDelay);
    }
    digitalWrite(ADC_CS,HIGH);
    Elapsed = micros() - startTime;
    SPI.endTransaction();
  }

void ReadADC(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  unsigned long summation=0;
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int chan = -1;
  sscanf(header.c_str(),"%*[AIN]%u", &chan);
  //If the suffix is valid,
  if ( (chan >= 0) && (chan < 8) ) {
    ReadADCtoBuffer(chan);
    for (int i=0;i<OSvalue;i++){
        summation += dataBuffer[i];
    }
    interface.println(summation);
  }
}

void ReadADCBurst(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int chan = -1;
  sscanf(header.c_str(),"%*[BURST]%u", &chan);
  //If the suffix is valid,
  if ( (chan >= 0) && (chan < 8) ) {
    ReadADCtoBuffer(chan);
    int i;
    for (i=0;i<(OSvalue-1);i++){
        interface.print(dataBuffer[i]);
        interface.print(",");
    }
    interface.println(dataBuffer[i]);
  }
}

