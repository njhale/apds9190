#include <Arduino.h>

#include <apds9190.h>

APDS9190 *proximity_sensor;

void setup() {
  Serial.begin(9600);  // initialize hardware serial
  Serial.println("Setup!");

  proximity_sensor = new APDS9190();
  proximity_sensor->Init(4);  // initialize the APDS

}

void loop() {

  // Serial.print("CONTROL REG: ");
  // proximity_sensor->PrintlnBits(
  //     proximity_sensor->ReadFromRegister(APDS9190_CONTROL_REG)
  // );
  //
  Serial.print("Proximity: "); Serial.println(proximity_sensor->Proximity());

  delay(1000);

}
