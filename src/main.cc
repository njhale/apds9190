#include <Arduino.h>
#include <Wire.h>

#include <apds9190.h>
#include <ups_adxl.h>

APDS9190 *proximity_sensor;
ADXL345 *accelerometer;

void setup() {

  Serial.begin(9600);  // initialize hardware serial
  Serial.println("Setup!");

  // flip peripherals fet
  Serial.println("Pulling peripherals fet high...");
  digitalWrite(4, HIGH);

  // instantiate and initialize the proximity sensor
  proximity_sensor = new APDS9190();
  proximity_sensor->Init(4);  // initialize the APDS

  // instantiate and initialize the accelerometer
  accelerometer = new ADXL345();
  accelerometer->powerOn();

}

void loop() {

  // get proxmity
  uint16_t proximity = proximity_sensor->Proximity();
  Serial.print("Proximity: "); Serial.println(proximity);

  // get acceleration
  // int x, y, z;
  // accelerometer->readAccel(&x, &y, &z);
  // char str[50] = "";
  //
  // sprintf(str, "Acceleration: {%d,%d,%d}", x, y, z);
  // Serial.println(str);


  delay(20);

}
