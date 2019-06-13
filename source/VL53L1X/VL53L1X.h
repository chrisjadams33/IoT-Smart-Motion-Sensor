#ifndef VL53L1X_H
#define VL53L1X_H

#include "mbed.h"

#include "VL53L1X_register_map.h"

#define I2C_BUFFER_LENGTH 32

const char defaultAddress_VL53L1X = 0x29;

class VL53L1X {
  public:
    VL53L1X(PinName SDA, PinName SCL);
    bool begin();
    void softReset(); //Reset the sensor via software
    void startMeasurement(uint8_t offset = 0); //Write a block of bytes to the sensor to configure it to take a measurement
    bool newDataReady(); //Polls the measurement completion bit
    uint16_t getDistance(); //Returns the results from the last measurement, distance in mm
    uint16_t getSignalRate(); //Returns the results from the last measurement, signal rate
    void setDistanceMode(uint8_t mode = 2);//Defaults to long range
    uint8_t getDistanceMode();
    uint8_t getRangeStatus(); //Returns the results from the last measurement, 0 = valid

    uint8_t readRegister(uint16_t addr); //Read a byte from a 16-bit address
    uint16_t readRegister16(uint16_t addr); //Read two bytes from a 16-bit address
    void writeRegister(uint16_t addr, uint8_t val); //Write a byte to a spot
    void writeRegister16(uint16_t addr, uint16_t val); //Write two bytes to a spot
    
  private:
  I2C _i2c;
  
  uint8_t _deviceAddress;
  uint8_t _distanceMode;// = 0; <--- might cause a problem
};


#endif