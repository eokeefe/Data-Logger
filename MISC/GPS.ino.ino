#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/*
 * Pinout:
 * 5V(Red)= N/A
 * RX     = 1 ------  Along the lines of GPS_out  -> Board_in
 * TX     = 0 ------  Along the lines of GPS_in   -> Board_out
 * 3.3V   = 3.3V
 * GND    = GND
 * 
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 0(rx) and 1(tx).
*/
static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 9600;         // uBlox GPS needs to run at 9600. The usual 4800 not recommended.

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

//  delay(10000);         // The GPS takes a long time to acquire usable data. No need to read. Unless powered for a while.

}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0){
    
    if (gps.encode(ss.read())){

      displayLoc();
      displayDate();
      displayTime();

      Serial.println();
    }

  }
  
  delay(100);         // Wait a little before fetching next data/process (100 millis works great, others rate not recomended)
    

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayLoc(){
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    // Display Lon and Lat at 12 decimal places
    Serial.print(gps.location.lat(), 12);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 12);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}

void displayDate(){
  // Display date based on uBlox own clock
  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}

void displayTime(){
  // Display time based on uBlock own clock ----- "centiseconds" not recorded by the clock
  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
}
