// included for LSM 303 interface
#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_LSM303_U.h"

Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
const double Pi = 3.14159;

void setup() {
  Serial.begin(2000000);
  // Check LSM 303 wiring
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  // initialize current direction as rotational setpoint
  sensors_event_t event; 
  mag.getEvent(&event);
  Serial.println((atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi);
}

void loop() 
{
    
  // Read from LSM 303 magnetometer
  sensors_event_t event; 
  mag.getEvent(&event);

  // Calculate heading by taking arctan of x and y vector and convert to degrees
  double heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;

  if(heading < 0)
    heading += 360;

  Serial.println(heading);
  delay(2);

}
