/*
 * 10DOF sensor
 * 
 * Pinout:
 *  V_in  = 3.3V
 *  GND   = GND
 *  SCL   = 19
 *  SDA   = 18
 *  
 * No pin selection required
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{

  // Discard any input.
  do {
    delay(10);
  } while (Serial.available() && Serial.read() >= 0);

  // Run a selection
  Serial.println(F("What to sensor call [A] acceleration, [M] magnetic, [G] gyros?")); delay(10000);
  String in = Serial.readString();

  if(in.toUpperCase() != 'A' || 'M' || 'G'){
    Serial.println(F("Wrong selection! \nPlease try again and make a correct selections."));
  }
  else{ rawData(in);}
  
//  sensors_event_t accel_event;
//  sensors_event_t mag_event;
//  sensors_event_t bmp_event;
//  sensors_vec_t   orientation;

  /* Read the accelerometer and magnetometer */
//  accel.getEvent(&accel_event);
//  mag.getEvent(&mag_event);

  /* Use the new fusionGetOrientation function to merge accel/mag data */  
//  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
//  {
//    /* 'orientation' should have valid .roll and .pitch fields */
//    Serial.print(F("Orientation: "));
//    Serial.print(orientation.roll);
//    Serial.print(F(" "));
//    Serial.print(orientation.pitch);
//    Serial.print(F(" "));
//    Serial.print(orientation.heading);
//    Serial.println(F(""));
//  }

  /* Previous code removed handling accel and mag data separately */
  //  /* Calculate pitch and roll from the raw accelerometer data */
  //  Serial.print(F("Orientation: "));
  //  accel.getEvent(&accel_event);
  //  if (dof.accelGetOrientation(&accel_event, &orientation))
  //  {
  //    /* 'orientation' should have valid .roll and .pitch fields */
  //    Serial.print(orientation.roll);
  //    Serial.print(F(" "));
  //    Serial.print(orientation.pitch);
  //    Serial.print(F(" "));
  //  }
  //  
  //  /* Calculate the heading using the magnetometer */
  //  mag.getEvent(&mag_event);
  //  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  //  {
  //    /* 'orientation' should have valid .heading data now */
  //    Serial.print(orientation.heading);
  //  }
  //  Serial.println(F(""));

  /* Calculate the altitude using the barometric pressure sensor */
//  bmp.getEvent(&bmp_event);
//  if (bmp_event.pressure)
//  {
//    /* Get ambient temperature in C */
//    float temperature;
//    bmp.getTemperature(&temperature);
//    /* Convert atmospheric pressure, SLP and temp to altitude */
//    Serial.print(F("Alt: "));
//    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
//                                        bmp_event.pressure,
//                                        temperature)); 
//    Serial.println(F(""));
//    /* Display the temperature */
//    Serial.print(F("Temp: "));
//    Serial.print(temperature);
//    Serial.println(F(""));
//  }
  
  delay(100);
}

void rawData(String sel)
{
  sel = sel.toUpperCase();
  sensors_event_t event;          //Create new sensor event (call)
  
  if(sel == 'A')
  {
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  }

  if(sel == 'M')
  {
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  Serial.print(F("MAG   "));
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  }

  if(sel == 'G')
  {
  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");  
  }
  
  Serial.println(F(""));
  delay(1000);
  
}

