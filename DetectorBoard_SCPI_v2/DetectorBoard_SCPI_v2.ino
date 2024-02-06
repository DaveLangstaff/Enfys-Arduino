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
    Sets oversampling to xxx where xxx is the number of samples to take, in the range 1..MaxDataSize
  OS?
    Returns current level of oversampling
  PO:ON Turns power off
  PO:OFF Turns power on
  BURST#? - takes reading and returns whole dataBuffer, length as set by OS
  TIME? - returns time in usec for last read operation
  THROW - Sets number of readings to throw away at start of each burst
  THROW? - Gets number of readings to throw away at start of each burst

  all commands will return either the requested data (? commands) 
  or an error code indicating successful or otherwise completion
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
#define MaxDataSize 2000
#define MaxSampleDelay 1024     // maximum delay in usec between samples
#define MaxThrowAway 1024       // Maximum number of samples to throw away
unsigned int DAC0_value=0;
unsigned int DAC1_value=0;

//byte OSLevel=0;
unsigned int OSvalue=1;
unsigned int sampleDelay=0;

unsigned int dataBuffer[MaxDataSize];
unsigned long Elapsed;
int throwAway = 1;    // number of readings to throwaway before accumulating in buffer

// define errors codes to return
#define ERR_NO_ERROR 0
#define ERR_BAD_COMMAND -1
#define ERR_TIMEOUT -2
#define ERR_BUFF_OVERFLOW -3
#define ERR_BAD_SUFFIX -4
#define ERR_BAD_PARAM -5

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
  my_instrument.RegisterCommand(F("DELAY"), &setSampleDelay);
  my_instrument.RegisterCommand(F("DELAY?"), &getSampleDelay);
  my_instrument.RegisterCommand(F("THROW"), &setThrowAway);
  my_instrument.RegisterCommand(F("THROW?"), &getThrowAway);
  my_instrument.SetErrorHandler(&myErrorHandler);
  
  
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
  // Blue LED used as power indicator. Start turned OFF, turns on with POWER:ON command
  pinMode(LED_BLUE, OUTPUT); digitalWrite(LED_BLUE, HIGH); // N LED drive is inverted

}

void loop()
{
  my_instrument.ProcessInput(Serial, "\n");
}


void GetLastEror(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  switch(my_instrument.last_error){
    case my_instrument.ErrorCode::BufferOverflow: 
      interface.println(ERR_BUFF_OVERFLOW);
      break;
    case my_instrument.ErrorCode::Timeout:
      interface.println(ERR_TIMEOUT);
      break;
    case my_instrument.ErrorCode::UnknownCommand:
      interface.println(ERR_BAD_COMMAND);
      break;
    case my_instrument.ErrorCode::NoError:
      interface.println(ERR_NO_ERROR);
      break;
  }
  my_instrument.last_error = my_instrument.ErrorCode::NoError;
}

void myErrorHandler(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //This function is called every time an error occurs

  /* The error type is stored in my_instrument.last_error
     Possible errors are:
       SCPI_Parser::ErrorCode::NoError
       SCPI_Parser::ErrorCode::UnknownCommand
       SCPI_Parser::ErrorCode::Timeout
       SCPI_Parser::ErrorCode::BufferOverflow
  */

  /* For BufferOverflow errors, the rest of the message, still in the interface
  buffer or not yet received, will be processed later and probably 
  trigger another kind of error.
  Here we flush the incomming message*/
  switch(my_instrument.last_error){
    case my_instrument.ErrorCode::BufferOverflow: 
      delay(2);
      while (interface.available()) {
        delay(2);
        interface.read();
      }
      interface.println(ERR_BUFF_OVERFLOW);
      break;
    case my_instrument.ErrorCode::Timeout:
      interface.println(ERR_TIMEOUT);
      break;
    case my_instrument.ErrorCode::UnknownCommand:
      interface.println(ERR_BAD_COMMAND);
      break;
    case my_instrument.ErrorCode::NoError:
      interface.println(ERR_NO_ERROR);
      break;
  }
  my_instrument.last_error = my_instrument.ErrorCode::NoError;

  /*
  For UnknownCommand errors, you can get the received unknown command and
  parameters from the commands and parameters variables.
  */
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
    interface.println(ERR_NO_ERROR);
}

void PowerOff(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    digitalWrite(POWER3V3, LOW);    // turn OFF 3v3 power
    delay(1);                       // wait 1ms
    digitalWrite(POWER12V, LOW);    // turn OFF 12v power
    digitalWrite(LED_BLUE, HIGH);
    interface.println(ERR_NO_ERROR);
}

void trigger() {
  digitalWrite(TRIG, LOW);  //put negative pulse on Trigger line
  digitalWrite(TRIG, HIGH);
}

void setThrowAway(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  int param = -1;
  String first_parameter = String(parameters.First());
  sscanf(first_parameter.c_str(),"%d",&param) ;
  if ((param>1) && (param<=MaxThrowAway)){
    throwAway=constrain(param,1,MaxThrowAway);
    interface.println(ERR_NO_ERROR);
  }  else {
    interface.println(ERR_BAD_PARAM);
  }
}
void getThrowAway(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  interface.println(throwAway);
}

void setOS(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  int param = -1;
  String first_parameter = String(parameters.First());
  sscanf(first_parameter.c_str(),"%d",&param) ;
  if ((param>1) && (param<=MaxDataSize)){
    OSvalue=constrain(param,1,MaxDataSize);
    interface.println(ERR_NO_ERROR);
  }  else {
    interface.println(ERR_BAD_PARAM);
  }
}


void getOS(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  interface.println(OSvalue);
}


void setSampleDelay(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  int param = -1;
  String first_parameter = String(parameters.First());
  sscanf(first_parameter.c_str(),"%d",&param) ;
  if ((param>0) && (param<=MaxSampleDelay)){
    sampleDelay=constrain(param,0,MaxSampleDelay);
    interface.println(ERR_NO_ERROR);
  }  else {
    interface.println(ERR_BAD_PARAM);
  }
}

void getElapsed(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  interface.println(Elapsed);
}

void getSampleDelay(SCPI_C commands, SCPI_P parameters, Stream& interface) {
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
    if ((param>0)&&(param<=4095)) {  // if the parameter is in range, then set the DAC
      param=constrain(param,0,4095);
      trigger();
      SPI.beginTransaction(SPISettings(ClockSpeed, MSBFIRST, SPI_MODE1));
      switch (suffix) {
        case 0:
          digitalWrite(DAC0_CS, LOW);
          DAC0_value=param;
          delayMicroseconds(10);
          SPI.transfer16(param);
          digitalWrite(DAC0_CS, HIGH);
          break;
        case 1:
          digitalWrite(DAC1_CS, LOW);
          DAC1_value=param;
          delayMicroseconds(10);
          SPI.transfer16(param);
          digitalWrite(DAC1_CS, HIGH);
          break;
      }
      delayMicroseconds(5);
      SPI.endTransaction();
      interface.println(ERR_NO_ERROR);
    } else{
      interface.println(ERR_BAD_PARAM);  // if paramter is out of range, print error code
    }
  } else {
    interface.println(ERR_BAD_SUFFIX);    // if suffix is out of range, print error code
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
  } else {
    interface.println(ERR_BAD_SUFFIX);
  }
}

void ReadADCtoBuffer(int chan){
    trigger();
    SPI.beginTransaction(SPISettings(ClockSpeed, MSBFIRST, SPI_MODE0));
    uint16_t control = chan << 11;  // set control word to point to ADC channel
    digitalWrite(ADC_CS, LOW);
    // throwaway readings first
    for (int i=0;i<throwAway;i++){
      dataBuffer[0] = SPI.transfer16(control); // dummy read, will be overwritten in loop
    }
    unsigned long startTime = micros();  // record start time
    for (int i=0;i<OSvalue;i++){
        dataBuffer[i] = SPI.transfer16(control);
        delayMicroseconds(sampleDelay);
    }
    digitalWrite(ADC_CS,HIGH);
    Elapsed = micros() - startTime;      // calculted elapsed time
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
  } else {
    interface.println(ERR_BAD_SUFFIX);
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
  } else {
    interface.println(ERR_BAD_SUFFIX);
  }
}

