#include "apds9190.h"


/**
 * @brief Writes a single byte to the given register over i2c
 *
 * @param  data Byte of data to send
 * @param  reg  Register to send byte of data to
 * @return      [description]
 */
bool APDS9190::WriteToRegister(byte data, byte reg) {
  Wire.beginTransmission(APDS9190_ADDRESS);  // start transmission
  Wire.write(reg);  // send register to use
  Wire.write(data);  // write data to that register

  // end the transmission and check for failure
  if( Wire.endTransmission() != 0 ) {
    Serial.println(F("End transmission failure"));
    return false;
  }

  // Serial.println("Transmission successful!");

  return true;

}

/**
 * @breif Reads an 8 but word containing data from teh given registry
 *
 * @param  reg Register to read from
 * @return     A 8 bit word containing the data from the registry
 */
uint8_t APDS9190::ReadFromRegister(byte reg) {

  uint8_t data = -1;  // < 0 is a bad status

  Wire.beginTransmission(APDS9190_ADDRESS);  // start transmission to device
	Wire.write(reg);  // sends registry to read from
	if (Wire.endTransmission(false) != 0) {
    Serial.println("Write register failed");
  }  // end transmission

	// Wire.beginTransmission(APDS9190_ADDRESS);  // start transmission to device
	Wire.requestFrom(APDS9190_ADDRESS, 1);  // request 1 byte from the device

	while (Wire.available()) {
		data = Wire.read();  // receive a byte
	}

	// Wire.endTransmission();  // end transmission

  return data;

}

/**
 * @brief Reads a variable length amount of bytes from the given register into
 *        the given buffer
 *
 * @param  reg       Address of the register to read data from
 * @param  num_bytes Number of bytes to read from the given register
 * @param  buffer    Buffer to store the read data
 */
uint8_t APDS9190::ReadBlockFromRegister(byte reg, int num_bytes, byte* buffer) {

  Wire.beginTransmission(APDS9190_ADDRESS);  // start transmission to device
	Wire.write(reg);  // sends registry to read from
	Wire.endTransmission();  // end transmission

	Wire.requestFrom(APDS9190_ADDRESS, num_bytes);  // request some bytes from the register

  int i = 0;
	while (Wire.available()) {
    if (i >= num_bytes) return i;  // more bytes received than anticipated
		buffer[i++] = Wire.read();  // receive and store a byte
	}

  return i;

}

/**
 * @brief Reads a 16 bit word from the given register
 *
 * @return     A 16 bit word containing the 16 bit proximity value
 */
uint16_t APDS9190::Proximity() {

  // get the low proximity bits register value
  uint8_t low = ReadFromRegister(APDS9190_PDATAL_REG);
  // Serial.print("low bits: "); Serial.println(low);
  // get the high proximity bits register value
  uint8_t high = ReadFromRegister(APDS9190_PDATAH_REG);
  // Serial.print("high bits: "); Serial.println(high);

  uint16_t proximity = (uint16_t)high << 8;  // shift high left by 8 bites
  proximity |= low;  // bitwise or by the lower bits
  //
  // byte buffer[2] = {0, 0};   // declare buffer for register data
  // ReadBlockFromRegister(APDS9190_PILTH_REG, 2, buffer);  // read the data
  //
  // uint16_t proximity = (buffer[1]<<8)&0xFF00;  // shift left and mask high bits
  // proximity |= (buffer[0]&0xFF);  // mask and or the low bits

  return proximity;

}

/**
 * @brief Initializes the APDS9190
 *
 * @param apds_fet Fet pin used to power the APDS9190
 * @return True if the initialization was successful. False otherwise.
 */
bool APDS9190::Init() {

  uint8_t atime = 0xff;  // 2.7ms - minimum wait time
  uint8_t ptime = 0xff;  // 2.7ms - minimum proximity integration time
  uint8_t ppcount = 4;  // minimum proximity pulse count

  Wire.begin();  // join I2C as a master

  Serial.println(F("Setting APDS9190 registers..."));

  // disable and powerdown
  WriteToRegister(0, APDS9190_ENABLE_REG);

  // set minimums
  WriteToRegister(ptime, APDS9190_PTIME_REG);  // set min prox integration time
  WriteToRegister(atime, APDS9190_ATIME_REG);  // set the a time
  WriteToRegister(ppcount, APDS9190_PPCOUNT_REG);  // set the min prox pulse count

  // configure control register
  // byte pdrive = 0;  // 100mA of LED power
  // byte pdiode = 0x20;  // channel 1 diode
  // byte pgain = 0;  // 1x proximity gain
  // uint8_t again;
  byte control = 42;//pdrive | pdiode | pgain;
  // Serial.print(F("control: ")); Serial.println(control);
  // Serial.print(F("control bits: ")); PrintlnBits(control);
  WriteToRegister(control, APDS9190_CONTROL_REG);

  // configure enable register
  // byte wen = 8;  // enable wait
  // byte pen = 4;  // enable proximity
  // byte pon = 1;  // enable power on
  byte enable = 0x07;//wen | pen | pon;
  WriteToRegister(enable, APDS9190_ENABLE_REG);
  // Serial.print(F("enable: ")); Serial.println(enable);
  // Serial.print(F("enable bits: ")); PrintlnBits(enable);

  delay(12);  // take a short break

  return true;

}

void APDS9190::PrintBits(byte b) {

  for (byte mask = 0x80; mask; mask >>= 1) {  // shift a mask to 0
    if (mask & b) {
      Serial.print('1');
    } else {
      Serial.print('0');
    }
  }

}

void APDS9190::PrintlnBits(byte b) {

  PrintBits(b);  // print the bits

  Serial.println();  // end the line

}
