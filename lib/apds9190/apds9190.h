#include <Arduino.h>
#include <Wire.h>

#ifndef APDS9190_H
#define APDS9190_H

// I2C address
#define APDS9190_ADDRESS 0x39

// register names
#define APDS9190_ENABLE_REG 0x00  // enable of states and interrupts
#define APDS9190_PTIME_REG 0x02  // proximity ADC time
#define APDS9190_WTIME_REG 0x03  // wait time
#define APDS9190_PILTL_REG 0x08  // proximity interrupt low threshold low byte
#define APDS9190_PILTH_REG 0x09  // proximity interrupt low threshold hi byte
#define APDS9190_PIHTL_REG 0x0A  // proximity interrupt hi threshold low byte
#define APDS9190_PIHTH_REG 0x0B  // proximity interrupt hi threshold hi byte
#define APDS9190_PERS_REG 0x0C  // interrupt persistence filters
#define APDS9190_CONFIG_REG 0x0D  // configuration
#define APDS9190_PPCOUNT_REG 0x0E  // proximity pulse count
#define APDS9190_CONTROL_REG 0x0F  // gain control register
#define APDS9190_REV_REG 0x11  // revision number
#define APDS9190_STATUS_REG 0x13  // device status
#define APDS9190_PDATAL_REG 0x18  // proximity ADC low data register
#define APDS9190_PDATAH_REG 0x19  // proximity ADC high data register


class APDS9190 {
public:

  /**
   * @brief Writes a single byte to the given register over i2c
   *
   * @param  data Byte of data to send
   * @param  reg  Register to send byte of data to
   * @return      [description]
   */
  bool WriteToRegister(byte data, byte reg);

  /**
   * @brief Reads a single byte from the I2C device and specified register
   *
   * @param[in] reg the register to read from
   * @return True if successful read operation. False otherwise.
   */
  uint8_t ReadFromRegister(byte reg);

  void ReadWordFromRegister(byte reg);

  /**
   * @breif Reads a 16 bit word from the given register
   *
   * @param  reg Register to read from
   * @return     A 16 bit word containing the data from the registry
   */
  uint16_t Proximity();

  /**
   * @brief Initializes the APDS9190
   *
   * @param apds_fet Fet pin used to power the APDS9190
   * @return True if the initialization was successful. False otherwise.
   */
  bool Init(uint8_t apds_fet);

  void PrintBits(byte b);

  void PrintlnBits(byte b);

};




#endif
