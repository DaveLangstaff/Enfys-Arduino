/*
Vrekrer_scpi_parser library.
Control code for Enfyf Detector board
uses Vrekrer SCPI library to parse commands

Commands:
Detectorboard EGSE commands

Deafult baud rate: 57600
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
TIME?		- gives time in usec for last read operation

BURST#?		- reads OS readings into buffer and returns whole databuffer

DELAY xxxx 	- sets delay between SPI reads in usec
DELAY?		- returns delay between SPI reads in usec

THROW xxxx	- sets number of readings to throw away at start of each read cycle (0..1024)
THROW?		- returns number of readings to throw away at start of each read cycle (0..1024)					

HTR:ON		- enable on board heater supply
HTR:OFF		- disable on-board heater supply
HTR:DAC xxxx	- set heater DAC (0..4095)
HTR:DAC?	- return current value of heater DAC

POwer:ON	- Enable power to detector board
POwer:OFF	- Disable power to detector board

RTD:TEMP?	- get temperature from RTD device
  all commands will return either the requested data (? commands) 
  or an error code indicating successful or otherwise completion
*/
#define ID_STRING "Aberystwyth University,Enfys Detector EGSE,#02.1,"

#include "Arduino.h"

// SCPI Parser constants
#define SCPI_MAX_TOKENS 40 
#define SCPI_ARRAY_SYZE 6 
#define SCPI_MAX_COMMANDS 30
#define SCPI_HASH_TYPE uint16_t

#include "Vrekrer_scpi_parser.h"
//#include "ADC128S102.h"

#include <SPI.h>
#include <INA.h>  // Zanshin INA Library for INA238 and INA3221 devices used for current monitoring
#include <Adafruit_MAX31865.h>


#define DETEC_SCK   13//sck
#define DETEC_MISO  12//CIPO
#define DETEC_MOSI  11//COPI
#define DETEC_ADC_CS  10//cs
#define MWIR_DAC_CS  9 //cs
#define SWIR_DAC_CS  8 //cs
#define HTR_DAC_CS   7 // DAC for board heater current control
#define HTR_EN       6 // Enable board heater supply
#define RTD_MON_CS   5 // Enable for board temperature monitor
#define TRIG         4 // Trigger for scope
#define TRIG2        3 // Trigger for light source

// IN238 and INA3221 current monitors are both on the I2C bus
#define INA238_ADDR  0x45 // Current monitor for -12V supply
#define INA3221_ADDR 0x40 // Current monitor for +12, +3v3 and heater supplies
#define SHUNT_1R 1000000  // 1 ohm resistor in microohms
#define SHUNT_0R1 100000  // 0.1 ohm shunt resistor
#define SHUNT_0R15 150000  // 0.1 ohm shunt resistor
#define MAX_AMPS 0.1      // 100mA maximum current for current monitoring
uint8_t        devicesFound{0};          ///< Number of INAs found
INA_Class      INA;                      ///< INA class instantiation to use EEPROM


#define EN_3V3      A7 // enable pin for 3v3 power
#define EN_12V      A6 // enable pin for 12v power

#define ClockSpeed 1000000   //1MHz SPI clock (default)
#define minClk 100000     // 100KHz minimum clock
#define maxClk 16000000   // 16MHz maximum clock
#define serialSpeed 57600     // serial baud rate
#define MaxDataSize 4096
#define MaxSampleDelay 1024     // maximum delay in usec between samples
#define MaxThrowAway 4096       // Maximum number of samples to throw away
#define MaxHeaterDAC 4095       // Maximum value of heater DAC
unsigned int SWIR_DAC_Value=0;  // DAC for SWIR offset
unsigned int MWIR_DAC_Value=0;  // DAC for MWIR offset
unsigned int HTR_DAC_Value=0;   // DAC for on-board heater
unsigned int ClkSpeed;

Adafruit_MAX31865 RTD_Monitor = Adafruit_MAX31865(RTD_MON_CS);  // define RTD monitor object
#define RREF 4020.0
#define RNOMINAL 1000.0    // PT1000 sensor

//byte OSLevel=0;
unsigned int OSvalue=1;
unsigned int sampleDelay=0;

unsigned int dataBuffer[MaxDataSize];
unsigned long Elapsed;
int throwAway = 0;    // number of readings to throwaway before accumulating in buffer

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

 Serial.begin(serialSpeed);
 //delay(5000);

 my_instrument.RegisterCommand(F("*IDN?"), &Identify);
 my_instrument.RegisterCommand(F("*DBG?"), &PrintDebug);

  //Use "#" at the end of a token to accept numeric suffixes.
  my_instrument.RegisterCommand(F("AIn#?"), &ReadADC);
  my_instrument.RegisterCommand(F("DOut#"), &WriteDAC);
  my_instrument.RegisterCommand(F("OS"), &setOS);  
  my_instrument.RegisterCommand(F("OS?"), &getOS);
  my_instrument.RegisterCommand(F("CLK"), &setClk);  
  my_instrument.RegisterCommand(F("CLK?"), &getClk);
  my_instrument.RegisterCommand(F("CURR#?"), &getCurrent);
  my_instrument.RegisterCommand(F("VBUS#?"), &getVBus);
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
  my_instrument.SetErrorHandler(&myErrorHandler);
  
  //Serial.println("Commands registered");

  // setup SPI bus
  ClkSpeed = ClockSpeed;
  pinMode(DETEC_MOSI, OUTPUT); 
  pinMode(DETEC_SCK,OUTPUT);
  pinMode(SWIR_DAC_CS,OUTPUT); digitalWrite(SWIR_DAC_CS,HIGH);
  pinMode(MWIR_DAC_CS,OUTPUT); digitalWrite(MWIR_DAC_CS,HIGH);
  pinMode(TRIG,OUTPUT);  digitalWrite(TRIG,HIGH);
  pinMode(TRIG2,OUTPUT);  digitalWrite(TRIG2,LOW);
  pinMode(DETEC_ADC_CS,OUTPUT);  digitalWrite(DETEC_ADC_CS,HIGH);
  pinMode(DETEC_MISO, INPUT);
  //Serial.println("Pins allocated");

  SPI.begin();
  //Serial.println("SPI started");

  
  // setup pins for power rail control
  pinMode(EN_3V3, OUTPUT); digitalWrite(EN_3V3, LOW);
  pinMode(EN_12V, OUTPUT); digitalWrite(EN_12V, LOW);
  // Blue LED used as power indicator. Start turned OFF, turns on with POWER:ON command
  pinMode(LED_BLUE, OUTPUT); digitalWrite(LED_BLUE, HIGH); // N LED drive is inverted

  // pins for heater control
  pinMode(HTR_EN, OUTPUT); digitalWrite(HTR_EN, HIGH);    // HTR enable pin, active LOW
  pinMode(HTR_DAC_CS, OUTPUT); digitalWrite(HTR_DAC_CS, HIGH);  // HTR DAC enable, active low
  // set heater DAC to 0
  SPI.beginTransaction(SPISettings(ClkSpeed, MSBFIRST, SPI_MODE1));
  digitalWrite(HTR_DAC_CS, LOW);           // select heater DAC
  HTR_DAC_Value=0;
  delayMicroseconds(10);
  SPI.transfer16(0);                   // use 16-bit mode to transfer DAC data
  digitalWrite(HTR_DAC_CS, HIGH);          // unselect heater DAC
  SPI.endTransaction();
  //Serial.println("Heater DAC initialised");

  // setup RTD monitor
  RTD_Monitor.begin(MAX31865_2WIRE);
  //Serial.println("RTD Monitor initialised");

  /* set up current monitoring 
  channels are:
  1: INA3221_0, 3v3 supply, 1 ohm, 10mA
  2: INA3221_1, Heater supply, 0.1 ohm, 100mA
  3: INA3221_2, +12V supply, 0.1 ohm, 100mA
  4: INA238, -12V supply, 0.1 ohm, 100mA
  */
//Serial.println("INA0 begins");
INA.begin(1,SHUNT_1R, 0);

//Serial.println("INA1 begins");
INA.begin(1,SHUNT_0R1, 1);

//Serial.println("INA2 begins");
INA.begin(1,SHUNT_0R1, 2);

//Serial.println("INA3 begins");
INA.begin(1,SHUNT_0R1, 3);
INA.setBusConversion(8500);             // Maximum conversion time 8.244ms
INA.setShuntConversion(8500);           // Maximum conversion time 8.244ms
INA.setAveraging(20);                  // Average each reading n-times
INA.setMode(INA_MODE_CONTINUOUS_BOTH);  // Bus/shunt measured continuously
//Serial.println("INA all set up");

}

void loop()
{
  my_instrument.ProcessInput(Serial, "\n");
}

void PrintDebug(SCPI_C commands, SCPI_P parameters, Stream& interface) {

  static char     sprintfBuffer[100];  // Buffer to format output
  static char     busChar[8], shuntChar[10], busMAChar[10], busMWChar[10];  // Output buffers

  my_instrument.PrintDebugInfo(interface);
  interface.print("INA current monitors:");
  interface.println(devicesFound);
  //Serial.println(INA.getShuntMicroVolts(3));
  Serial.print(F("Nr Adr Type   Bus      Shunt       Bus         Bus\n"));
  Serial.print(F("== === ====== ======== =========== =========== ===========\n"));
  for (uint8_t i = 0; i < 4; i++)  // Loop through all devices
  {
    dtostrf(INA.getBusMilliVolts(i) / 1000.0, 7, 4, busChar);      // Convert floating point to char
    dtostrf(INA.getShuntMicroVolts(i) / 1000.0, 9, 4, shuntChar);  // Convert floating point to char
    dtostrf(INA.getBusMicroAmps(i) / 1000.0, 9, 4, busMAChar);     // Convert floating point to char
    dtostrf(INA.getBusMicroWatts(i) / 1000.0, 9, 4, busMWChar);    // Convert floating point to char
    sprintf(sprintfBuffer, "%2d %3d %s %sV %smV %smA %smW\n", i + 1, INA.getDeviceAddress(i),
            INA.getDeviceName(i), busChar, shuntChar, busMAChar, busMWChar);
    Serial.print(sprintfBuffer);
  }  // for-next each INA device loop
  Serial.println();
  

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
  interface.println(F(ID_STRING VREKRER_SCPI_VERSION));
  //*IDN? Suggested return string should be in the following format:
  // "<vendor>,<model>,<serial number>,<firmware>"
  trigger();
}

void DoNothing(SCPI_C commands, SCPI_P parameters, Stream& interface) {
}

void PowerOn(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    digitalWrite(EN_12V, HIGH);     // turn ON 12V power
    //delay(2);
    digitalWrite(EN_3V3, HIGH);    // turn ON 3v3 power
    digitalWrite(LED_BLUE, LOW);
    interface.println(ERR_NO_ERROR);
}

void PowerOff(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    digitalWrite(EN_3V3, LOW);    // turn OFF 3v3 power
    //delay(1);                       // wait 1ms
    digitalWrite(EN_12V, LOW);    // turn OFF 12v power
    digitalWrite(LED_BLUE, HIGH);
    interface.println(ERR_NO_ERROR);
}

void HTROn(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    digitalWrite(HTR_EN, LOW);
    interface.println(ERR_NO_ERROR);
}

void HTROff(SCPI_C commands, SCPI_P parameters, Stream& interface) {
    digitalWrite(HTR_EN, HIGH);
    interface.println(ERR_NO_ERROR);
}

void getVBus(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //get bus voltage from indicated channel
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = -1;
  sscanf(header.c_str(),"%*[VBUS]%u?", &suffix);
  //If the suffix is valid,
  if ( (suffix >= 0) && (suffix <= 3) ) {
    interface.println(INA.getBusMilliVolts(suffix));
  }
  else{
  interface.println(ERR_BAD_SUFFIX);
  }
} 

void getCurrent(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //get bus voltage from indicated channel
  //interface.println("getCurrent...");
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = 1;
  //interface.println(header);
  sscanf(header.c_str(),"%*[CURR]%u?", &suffix);
  //If the suffix is valid,
  if ( (suffix >= 0) && (suffix <= 3) ) {
    interface.println(INA.getBusMicroAmps(suffix));
  }
  else{
  interface.println(ERR_BAD_SUFFIX);
  }
} 



void getName(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //get bus voltage from indicated channel
  String header = String(commands.Last());
  header.toUpperCase();
  int suffix = -1;
  sscanf(header.c_str(),"%*[NAME]%u?", &suffix);

  //If the suffix is valid,
  if ( (suffix >= 1) && (suffix <= 4) ) {
    interface.println(INA.getDeviceName(suffix));
  }
  else{
  interface.println(ERR_BAD_SUFFIX);
  }
} 

void setHTRDAC(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the 12-bit heater DAC
  int param = -1;
  String first_parameter = String(parameters.First());
  sscanf(first_parameter.c_str(),"%d",&param) ;
  // check parameter is in range
  if ((param>=0) && (param<=MaxHeaterDAC)){
    HTR_DAC_Value=constrain(param,0,MaxHeaterDAC);   // store the value away
    SPI.beginTransaction(SPISettings(ClkSpeed, MSBFIRST, SPI_MODE1));
    digitalWrite(HTR_DAC_CS, LOW);           // select heater DAC
    HTR_DAC_Value=param;
    delayMicroseconds(10);
    SPI.transfer16(param);                   // use 16-bit mode to transfer DAC data
    digitalWrite(HTR_DAC_CS, HIGH);          // unselect heater DAC
    SPI.endTransaction();
    interface.println(ERR_NO_ERROR);         // return success code
  }  else {
    interface.println(ERR_BAD_PARAM);        // otherwise return out of range error
  }
}

void getHTRDAC(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //get current value set on Heater DAC
  interface.println(HTR_DAC_Value);
}

void getRTDTemperature(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  // read temperature sensor and return as float
  interface.println(RTD_Monitor.temperature(RNOMINAL,RREF));
}

void trigger() {
  digitalWrite(TRIG, LOW);  //put negative pulse on Trigger line
  digitalWrite(TRIG, HIGH);
  digitalWrite(TRIG2, HIGH);  //put positive pulse on Trigger2 line
  digitalWrite(TRIG2, LOW);
}

void setThrowAway(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  int param = -1;
  String first_parameter = String(parameters.First());
  sscanf(first_parameter.c_str(),"%d",&param) ;
  if ((param>=0) && (param<=MaxThrowAway)){
    throwAway=constrain(param,0,MaxThrowAway);
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
  if ((param>=1) && (param<=MaxDataSize)){
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

void setClk(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  int param = -1;
  String first_parameter = String(parameters.First());
  sscanf(first_parameter.c_str(),"%d",&param) ;
  if ((param>=minClk) && (param<=maxClk)){
    ClkSpeed=constrain(param,minClk,maxClk);
    interface.println(ERR_NO_ERROR);
  }  else {
    interface.println(ERR_BAD_PARAM);
  }
}


void getClk(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the oversampling level
  interface.println(ClkSpeed);
}


void setSampleDelay(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //use the first parameter to set the delay between SPI reads in usec
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
      SPI.beginTransaction(SPISettings(ClkSpeed, MSBFIRST, SPI_MODE1));
      switch (suffix) {
        case 0:
          digitalWrite(SWIR_DAC_CS, LOW);
          SWIR_DAC_Value=param;
          delayMicroseconds(10);
          SPI.transfer16(param);
          digitalWrite(SWIR_DAC_CS, HIGH);
          break;
        case 1:
          digitalWrite(MWIR_DAC_CS, LOW);
          MWIR_DAC_Value=param;
          delayMicroseconds(10);
          SPI.transfer16(param);
          digitalWrite(MWIR_DAC_CS, HIGH);
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
          param=SWIR_DAC_Value;
          }
        else {
          param=MWIR_DAC_Value;
          }
  interface.println(param);
  } else {
    interface.println(ERR_BAD_SUFFIX);
  }
}

void ReadADCtoBuffer(int chan){
    trigger();
    SPI.beginTransaction(SPISettings(ClkSpeed, MSBFIRST, SPI_MODE0));
    uint16_t control = chan << 11;  // set control word to point to ADC channel
    // throwaway readings first
    unsigned long startTime = micros();  // record start time
    for (int i=0;i<throwAway;i++){
        digitalWrite(DETEC_ADC_CS, LOW);
        dataBuffer[0] = SPI.transfer16(control); // dummy read, will be overwritten in loop
        delayMicroseconds(sampleDelay);
        digitalWrite(DETEC_ADC_CS, HIGH);
    }
    for (int i=0;i<OSvalue;i++){
        digitalWrite(DETEC_ADC_CS, LOW);
        dataBuffer[i] = SPI.transfer16(control);
        delayMicroseconds(sampleDelay);
        digitalWrite(DETEC_ADC_CS, HIGH);
    }
//    digitalWrite(DETEC_ADC_CS,HIGH);
    Elapsed = micros() - startTime;      // calculted elapsed time
    SPI.endTransaction();
  }

void ReadADC(SCPI_C commands, SCPI_P parameters, Stream& interface) {
  //Get the numeric suffix/index (if any) from the commands
  String header = String(commands.Last());
  header.toUpperCase();
  int chan = -1;
  sscanf(header.c_str(),"%*[AIN]%u", &chan);
  //If the suffix is valid,
  if ( (chan >= 0) && (chan < 8) ) {
    unsigned long summation=0;
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
