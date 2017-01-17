/* Temperature and Humidity Code
Jose Flores
Steven Lin
ME170 - Capstone Design Project
Shipping Sensor Team
UC Merced
Date Created: 4/4/2012
Date Mod: 5/3/2012
This program integrates both the Piezo Code, Temperature and Humidity Code,
Accelerometer Code, and
Data-logging code into a single code.
This code was composed using libraries and code from the following intelects:
Ben Adams 2011, Craig Ringer 2012, David Curtilles 2007, Tom Igoe 2011, and
RandomRobotics (alias), a member of "SparkFun Electronics" forum, dated Jun 28, 2011.
All open source code.
*/

#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>                    // Only used for sprintf
#include <DHT_U.h>
#include <SD.h>                     // Library for SD
#include "RTClib.h"                 // Library for RTC


// Parameters for Piezo sensor
const int ledPin = 13;              // led connected to digital pin 13
const int knockSensor = A0;         // the piezo is connected to analog pin 0
const int threshold = 450;
/* Threshold value to decide when the detected sound is a knock or not (not yet sure of
displacement) real values ie: Max value is 1023, this value can easily be reached by smashing on a table
*/
int sensorReading = 0;              // variable to store the value read from the sensor pin
int ledState = LOW;                 // variable used to store the last LED status, to toggle the light on/off
(HIGH = ON; LOW = OFF)
int lowestOut = 0;                  // variable to store low value
unsigned short timer = 0;


// Parameters for accelerometer
#define SS 2                        // Serial Select -> CS on LIS331
#define MOSI 11                     // MasterOutSlaveIn -> SDI
#define MISO 12                     // MasterInSlaveOut -> SDO
#define SCK 13                      // Serial Clock -> SPC on LIS331
#define SCALE 0.0007324;            // approximate scale factor for full range (+/-24g)
                                    // scale factor: +/-24g = 48G range. 2^16 bits. 48/65536 = 0.0007324
                                    // global acceleration values
double xAcc, yAcc, zAcc;


//Paramters for humidity and temperature sensor
#define DHT22_PIN 7                 //Data wire is plugged into port 7 on the Arduino
DHT22 myDHT22(DHT22_PIN);           // Setup a DHT22 instance, recognize that we have a sensor


// Paramters for data logging shield
RTC_DS1307 RTC;                     // Define the Real Time Clock object
const int chipSelect = 10;          // for the data logging shield, we use digital pin 10 for the SD cs line
// Error show program
void error(char *str){
Serial.print("error: ");
Serial.println(str);
}


void setup(void){
Serial.begin(9600);                 // start serial port
// Configure SPI
SPI_SETUP();
// Configure accelerometer
Accelerometer_Setup();
// initialize the SD card
Serial.print("Initializing SD card...");
// make sure that the default chip select pin is set to
// output, even if you don't use it:
pinMode(10, OUTPUT);
// see if the card is present and can be initialized:
if (!SD.begin(chipSelect)) {
error("Card failed, or not present.");
}
else {
Serial.println("Card initialized.");
}
pinMode(ledPin, OUTPUT);            // declare the ledPin as as OUTPUT
// Connect to RTC
Wire.begin();
if (!RTC.begin()){
Serial.println("RTC failed.");
}
File logfile = SD.open("Data.csv", FILE_WRITE); // Open file to store data
if (logfile) {
logfile.println(", , , , ,");       // Leading blank line incase there is previous data
// Heading - 1st line of Data
logfile.println("Date, Time, X-Axis [g], Y-Axis [g], Z-Axis [g], Shock, Temperature [C], Humidity[%]");
logfile.close();                    // Close file so no data is lost
Serial.print("Saving to: Data.csv");
}
else{
error("Couldn't create file.");
ledState = !ledState;               // toggle the status of the ledPin
digitalWrite(ledPin, ledState);     // Light up LED if this happens
}
}


void loop(void){
DateTime now = RTC.now();           // Fetch the date and time
if (timer < 2000){
// read the sensor and store it in the variable sensorReading:
sensorReading = analogRead(knockSensor);
readVal();                          // get accelerometer values and put into global variables
// if the sensor reading is greater than the threshold:
if (sensorReading >= threshold) {
// this loop outputs the lowest value when the old low value is compared with new output
if (sensorReading > lowestOut){ lowestOut = sensorReading; }
File logfile = SD.open("Data.csv", FILE_WRITE);
// Log date and time
logfile.print(now.month(), DEC);    // Log month
logfile.print("/");
logfile.print(now.day(), DEC);      // Log day
logfile.print("/");
logfile.print(now.year(), DEC);     // Log year
logfile.print(",");                 // Go into next box on Excel
logfile.print(now.hour(), DEC);     // Log hour
logfile.print(":");
logfile.print(now.minute(), DEC);   // Log minute
logfile.print(":");
logfile.print(now.second(), DEC);   // Log second
logfile.print(",");                 // Go into next box on Excel

//Print & Store Acceleration readout; Might need to take out if its checking all the time in here only prints if shock is triggered
Serial.print(xAcc, 1);
Serial.print(",");
logfile.print(xAcc, 1);
logfile.print(",");
Serial.print(yAcc, 1);
Serial.print(",");
logfile.print(yAcc, 1);
logfile.print(",");
Serial.println(zAcc, 1);
Serial.print(",");
logfile.print(zAcc, 1);
logfile.print(",");
//Print & Store Piezo Output
Serial.println(sensorReading);
logfile.println(sensorReading);
logfile.close();                      // Close file so data is not lost
}
timer += 50;
delay(50);                            // delay to avoid overloading the serial port buffer(min limit unknown) Faster means more data points
}
else
{
DHT22_ERROR_t errorCode;
sensorReading = analogRead(knockSensor);  //read Piezo readout
readVal();                            // get accelerometer values and put into global variables
// The sensor can only be read from every 1-2s, and requires a minimum 2s warm-up after power-on.
errorCode = myDHT22.readData();
switch(errorCode) {
case DHT_ERROR_NONE: //Successful temperature and humidity read
{
File logfile = SD.open("Data.csv", FILE_WRITE);
// Log date and time
logfile.print(now.month(), DEC);
logfile.print("/");
logfile.print(now.day(), DEC);
logfile.print("/");
logfile.print(now.year(), DEC);
logfile.print(",");
logfile.print(now.hour(), DEC);
logfile.print(":");
logfile.print(now.minute(), DEC);
logfile.print(":");
logfile.print(now.second(), DEC);
logfile.print(",");
//Print & Store Acceleration readout
Serial.print(xAcc, 1);
Serial.print(",");
logfile.print(xAcc, 1);
logfile.print(",");
Serial.print(yAcc, 1);
Serial.print(",");
logfile.print(yAcc, 1);
logfile.print(",");
Serial.println(zAcc, 1);
Serial.print(",");
logfile.print(zAcc, 1);
logfile.print(",");
logfile.close();
//Must revise this zone to optimize
if (sensorReading >= threshold) {
// this loop outputs the lowest value when the old low value is compared with new output
if (sensorReading > lowestOut){
lowestOut = sensorReading;
}
logfile = SD.open("Data.csv", FILE_WRITE);
logfile.print(sensorReading);             // Log output from Piezo sensor
Serial.print(sensorReading);
logfile.print(",");                       // Finish logging
Serial.print(",");
logfile.close();                          // Close file so data is not lost
}
else {
logfile = SD.open("Data.csv", FILE_WRITE);
logfile.print(",");                       // Leave shock column blank
Serial.print(",");
logfile.close();
}
logfile = SD.open("Data.csv", FILE_WRITE);
Serial.print(myDHT22.getTemperatureC());
Serial.print(",");
Serial.println(myDHT22.getHumidity());
logfile.print(myDHT22.getTemperatureC()); // Log temperature output
logfile.print(",");                       // Go into next box
logfile.println(myDHT22.getHumidity());   // Log humidity output
logfile.close();                          // Close file so no data is lost
break;
}
case DHT_ERROR_CHECKSUM:
{
Serial.print("check sum error ");
Serial.print(myDHT22.getTemperatureC());
Serial.print("C ");
Serial.print(myDHT22.getHumidity());
Serial.println("%");
break;
}
case DHT_BUS_HUNG:
{
Serial.println("BUS Hung ");
break;
}
case DHT_ERROR_NOT_PRESENT:
{
Serial.println("Not Present ");
break;
}
case DHT_ERROR_ACK_TOO_LONG:
{
Serial.println("ACK time out ");
break;
}
case DHT_ERROR_SYNC_TIMEOUT:
{
Serial.println("Sync Timeout ");
break;
}
case DHT_ERROR_DATA_TIMEOUT:
{
Serial.println("Data Timeout ");
break;
}
case DHT_ERROR_TOOQUICK:
{
Serial.println("Polled to quick ");
break;
}
}
timer = 0;
}
}


// Programs for accelerometer
// Read the accelerometer data and put values into global variables
void readVal()
{
byte xAddressByteL = 0x28;              // Low Byte of X value (the first data register)
byte readBit = B10000000;               // bit 0 (MSB) HIGH means read register
byte incrementBit = B01000000;          // bit 1 HIGH means keep incrementing registers
// this allows us to keep reading the data registers by pushing an empty byte
byte dataByte = xAddressByteL | readBit | incrementBit;
byte b0 = 0x0;                          // an empty byte, to increment to subsequent registers
digitalWrite(SS, LOW);                  // SS must be LOW to communicate
delay(1);
SPI.transfer(dataByte);                 // request a read, starting at X low byte
byte xL = SPI.transfer(b0);             // get the low byte of X data
byte xH = SPI.transfer(b0);             // get the high byte of X data
byte yL = SPI.transfer(b0);             // get the low byte of Y data
byte yH = SPI.transfer(b0);             // get the high byte of Y data
byte zL = SPI.transfer(b0);             // get the low byte of Z data
byte zH = SPI.transfer(b0);             // get the high byte of Z data
delay(1);
digitalWrite(SS, HIGH);
// shift the high byte left 8 bits and merge the high and low
int xVal = (xL | (xH << 8));
int yVal = (yL | (yH << 8));
int zVal = (zL | (zH << 8));
// scale the values into G's
xAcc = xVal * SCALE;
yAcc = yVal * SCALE;
zAcc = zVal * SCALE;
}


void SPI_SETUP()
{
pinMode(SS, OUTPUT);
//digitalWrite(SS, HIGH);
// wake up the SPI bus
SPI.begin();
// This device reads MSB first:
SPI.setBitOrder(MSBFIRST);
/*
SPI.setDataMode()
Mode Clock Polarity (CPOL) Clock Phase (CPHA)
SPI_MODE0 0 0
SPI_MODE1 0 1
SPI_MODE2 1 0
SPI_MODE3 1 1
*/
SPI.setDataMode(SPI_MODE3);
/*
SPI.setClockDivider()
sets SPI clock to a fraction of the system clock
Arduino UNO system clock = 16 MHz
Mode SPI Clock
SPI_CLOCK_DIV2 8 MHz
SPI_CLOCK_DIV4 4 MHz
SPI_CLOCK_DIV8 2 MHz
SPI_CLOCK_DIV16 1 MHz
SPI_CLOCK_DIV32 500 Hz
SPI_CLOCK_DIV64 250 Hz
SPI_CLOCK_DIV128 125 Hz
*/
SPI.setClockDivider(SPI_CLOCK_DIV16); // SPI clock 1000Hz
}


void Accelerometer_Setup()
{
// Set up the accelerometer
// write to Control register 1: address 20h
byte addressByte = 0x20;
/* Bits:
PM2 PM1 PM0 DR1 DR0 Zen Yen Xen
PM2PM1PM0: Power mode (001 = Normal Mode)
25
DR1DR0: Data rate (00=50Hz, 01=100Hz, 10=400Hz, 11=1000Hz)
Zen, Yen, Xen: Z enable, Y enable, X enable
*/
byte ctrlRegByte = 0x37; // 00111111 : normal mode, 1000Hz, xyz enabled
// Send the data for Control Register 1
digitalWrite(SS, HIGH);
digitalWrite(SS, LOW);
delay(1);
SPI.transfer(addressByte);
SPI.transfer(ctrlRegByte);
delay(1);
digitalWrite(SS, HIGH);
delay(100);
// write to Control Register 2: address 21h
addressByte = 0x21;
// This register configures high pass filter
ctrlRegByte = 0x00; // High pass filter off
// Send the data for Control Register 2
digitalWrite(SS, LOW);
delay(1);
SPI.transfer(addressByte);
SPI.transfer(ctrlRegByte);
delay(1);
digitalWrite(SS, HIGH);
delay(100);
// Control Register 3 configures Interrupts
// Since I'm not using Interrupts, I'll leave it alone
// write to Control Register 4: address 23h
addressByte = 0x23;
/* Bits:
BDU BLE FS1 FS0 STsign 0 ST SIM
BDU: Block data update (0=continuous update)
26
BLE: Big/little endian data (0=accel data LSB at LOW address)
FS1FS0: Full-scale selection (00 = +/-6G, 01 = +/-12G, 11 = +/-24G)
STsign: selft-test sign (default 0=plus)
ST: self-test enable (default 0=disabled)
SIM: SPI mode selection(default 0=4 wire interface, 1=3 wire interface)
*/
ctrlRegByte = 0x30; // 00110000 : 24G (full scale)
// Send the data for Control Register 4
digitalWrite(SS, LOW);
delay(1);
SPI.transfer(addressByte);
SPI.transfer(ctrlRegByte);
delay(1);
digitalWrite(SS, HIGH);
}
