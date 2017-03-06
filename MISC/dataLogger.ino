/*
 * Simple data logger.
 */

#include <SPI.h>
#include "SdFat.h"

// Set USE_SDIO to zero for SPI card access. 
#define USE_SDIO 1

// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;

/*
 * Set DISABLE_CHIP_SELECT to disable a second SPI device.
 * For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
 * to 10 to disable the Ethernet controller.
 */
const int8_t DISABLE_CHIP_SELECT = -1;

// Interval between data records in milliseconds.
// The interval mrust be greater than the maximum SD write latency plus the
// time to acquire and write data to the SD to avoid overrun errors.
// Run the bench example to check the quality of your SD card.
const uint32_t SAMPLE_INTERVAL_MS = 1000;

//------------------------------------------------------------------------------
// File system object.
#if USE_SDIO
// Use faster SdioCardEX
SdFatSdioEX sd;
// SdFatSdio sd;
#else // USE_SDIO
SdFat sd;
#endif  // USE_SDIO

// Log file.
SdFile file;

// Time in micros for next data record.
uint32_t logTime;

const size_t BUF_SIZE = 512;
uint8_t buff [BUF_SIZE];

//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(1000);

  // Throw an error when opening SD card
  if (!sd.begin()) {
    Serial.println(F("Error: Cannot start SD card"));
    return;
  }

  // Create a new file
  file.open("test.dat", O_CREAT | O_APPEND | O_RDWR);

  
}
//------------------------------------------------------------------------------
void loop() {
  // Discard any input.
  do {
    delay(10);
  } while (Serial.available() && Serial.read() >= 0);

  // Start the program
  Serial.println(F("Type any character to start"));
  while (!Serial.available()) {
    SysCall::yield();
  } Serial.readString();



  // Run a write Test
  Serial.println("What to store in SD?"); delay(10000);
  String in = Serial.readString();
  newWrt(in);
 

//  // Do a read Test
//  file.rewind();
//  buff[20] = 5;
//  char tmp[sizeof(tst)];
//  for (unsigned int i = 0; i < sizeof(tst)-1; i++){
//    tmp[i] = file.read();
//  } 
//  Serial.print(F("This is the RAW read: "));
//  Serial.println(tmp);
//  if (sizeof(tmp) == sizeof(tst) && tst[0] == tmp[0] && tst[sizeof(tmp)-1] == tmp[sizeof(tmp)-1]){
//    Serial.println(F("Success Write and Read"));
//    return;
//  }
//  else{
//    Serial.println(F("Read and Write are not successful")); 
//    Serial.print(F("The difference is "));
//    Serial.println(tmp-tst);
//
//    Serial.print("\nFirst of each: ");
//    Serial.print(tst[0], DEC);
//    Serial.print(", ");
//    Serial.println(tmp[0], DEC);
//    Serial.println((tst));
//    Serial.println(sizeof(tmp));
//    Serial.print("Last of each: ");
//    Serial.print(tst[sizeof(tst)], DEC);
//    Serial.print(", ");
//    Serial.println(tmp[sizeof(tst)], DEC);
//  }

  file.close();
}

void newWrt(String a){

  file.open("test.dat", O_CREAT | O_APPEND | O_RDWR);

  if (!file.isOpen()) {
  Serial.println(F("Open failed - Check SD card!"));
  return;
  }
  
  char txt[sizeof(a)];
  a.toCharArray(txt, sizeof(a));
  Serial.print(F("Printing the test script: "));
  Serial.println(F(txt));
  file.write(txt);
}
