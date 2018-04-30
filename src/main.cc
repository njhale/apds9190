#include <Arduino.h>
#include <Wire.h>

#include <apds9190.h>

APDS9190 *proximity_sensor;

void setup() {

  Serial.begin(9600);  // initialize hardware serial
  Serial.println("Setup!");

  // instantiate and initialize the proximity sensor
  proximity_sensor = new APDS9190();
  proximity_sensor->Init();  // initialize the APDS

}

void loop() {

  // get proxmity
  uint16_t proximity = proximity_sensor->Proximity();
  Serial.print("Proximity: "); Serial.println(proximity);

  delay(20);

}
