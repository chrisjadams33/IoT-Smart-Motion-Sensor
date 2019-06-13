#ifndef _BMI160_H_
#define _BMI160_H_
/**
 * @brief BMI160 Bosch Small, low power inertial measurement unit
 *
 */
class BMI160 {
public:
 /**
  * BMI160 I2C Interface
  *
  * @param sda SDA pin
  * @param scl SCL pin
  * @param addr address of the I2C peripheral
  */
  BMI160(PinName sda, PinName scl, int addr) ;

 /**
  * BMI160 SPI Interface
  *
  * @param sck  SPI SCKL pin
  * @param miso SPI Master In Slave Out pin
  * @param mosi SPI Master Out Slave In pin
  * @param cs   SPI Chip Select pin
  */
  BMI160(PinName sck, PinName miso, PinName mosi, PinName cs) ;

/**
 * BMI160 destructor
 */
  ~BMI160() ;

/**
 * setCMD set value to the CMD register (0x7E)
 *
 * @param cmd uint8_t value to write
 * @returns none
 */
  void setCMD(uint8_t cmd) ;
  
/**
 * getStatus get value of the STATUS register (0x1B)
 * @param none
 * @returns the value of the STATUS register
 */
  uint8_t getStatus(void) ;
  
/**
 * getChipID get value of the CHIP_ID register (0x10)
 * @param none
 * @returns the chip ID (supposed to be 0xD1)
 */
  uint8_t getChipID(void) ;
  
/**
 * get range of Acc  (0x41)
 * @param none
 * @returns accelerometer g-range
 * @note the value is 2, 4, or 8
 */
  uint8_t getAccRange(void) ;
  
/**
 * get range of Gyr (0x43)
 * @param none
 * @returns Angular Rate Range and Resolution
 * @note the values are
 * @note 2000 ( 16.4 LSB/deg/s <-> 61.0 mdeg/s/LSB)
 * @note 1000 ( 32.8 LSB/deg/s <-> 30.5 mdeg/s/LSB)
 * @note  500 ( 65.6 LSB/deg/s <-> 15.3 mdeg/s/LSB)
 * @note  250 (131.2 LSB/deg/s <->  7.6 mdeg/s/LSB)
 * @note  125 (262.4 LSB/deg/s <->  3.8 mdeg/s/LSB)
 */
  int16_t getGyrRange(void) ;

/**
 * get Raw acc x value
 * @param none
 * @returns 16bit signed value
 */
  int16_t getAccRawX(void) ;
  
/**
 * get Raw acc y value
 * @param none
 * @returns 16bit signed value
 */
  int16_t getAccRawY(void) ;

/**
 * get Raw acc z value
 * @param none
 * @returns 16bit signed value
 */  
  int16_t getAccRawZ(void) ;

/**
 * get Raw gyr x value
 * @param none
 * @returns 16bit signed value
 */
  int16_t getGyrRawX(void) ;

/**
 * get Raw gyr y value
 * @param none
 * @returns 16bit signed value
 */
  int16_t getGyrRawY(void) ;

/**
 * get Raw gyr z value
 * @param none
 * @returns 16bit signed value
 */
  int16_t getGyrRawZ(void) ;

/**
 * get Raw acc x,y,z values
 * @param 16bit array address to receive the data
 * @returns none
 */
  void getAccRaw(int16_t *value) ;
  
/**
 * get Raw gyr x,y,z values
 * @param 16bit array address to receive the data
 * @returns none
 */
  void getGyrRaw(int16_t *value) ;

/**
 * get acc x value
 * @param none
 * @returns value (-acc_range ~ +acc_range)
 */
  float getAccX(void) ;
  
/**
 * get acc y value
 * @param none
 * @returns value (-acc_range ~ +acc_range)
 */  
  float getAccY(void) ;
  
/**
 * get acc z value
 * @param none
 * @returns value (-acc_range ~ +acc_range)
 */  
  float getAccZ(void) ;
  
/**
 * get gyr x value
 * @param none
 * @returns value (-gyr_range ~ +gyr_range)
 */    
  float getGyrX(void) ;
    
/**
 * get gyr y value
 * @param none
 * @returns value (-gyr_range ~ +gyr_range)
 */ 
  float getGyrY(void) ;
  
/**
 * get gyr z value
 * @param none
 * @returns value (-gyr_range ~ +gyr_range)
 */ 
  float getGyrZ(void) ;
  
/**
 * get acc x, y, z values
 * @param float array address to receive the values
 * @returns none
 * @note the value range is (-acc_range ~ +acc_range) 
 */
  void getAcc(float *value) ;

/**
 * get gyr x, y, z values
 * @param float array address to receive the values
 * @returns none
 * @note the value range is (-gyr_range ~ +gyr_range) 
 */
  void getGyr(float *value) ;
  
private:
  SPI *m_spi ;
  I2C *m_i2c ;
  DigitalOut *m_cs ;
  int m_addr ;
  int acc_range ;
  int gyr_range ;
  
  void init(void) ;
  void i2c_readRegs(int addr, uint8_t *data, int len) ;
  void i2c_writeRegs(uint8_t *data, int len) ;
  void spi_readRegs(int addr, uint8_t *data, int len) ;
  void spi_writeRegs(uint8_t *data, int len) ;
  void readRegs(int addr, uint8_t *data, int len) ;
  void writeRegs(uint8_t *data, int len) ;
} ;

#define ACC_PMU_SUSPEND  0x00
#define ACC_PMU_NORMAL   0x01
#define ACC_PMU_LOWPOWER 0x02
#define GYR_PMU_SUSPEND  0x00
#define GYR_PMU_NORMAL   0x01
#define GYR_PMU_FASTSU   0x03
#define MAG_PMU_SUSPEND  0x00
#define MAG_PMU_NORMAL   0x01
#define MAG_PMU_LOWPOWER 0x02
#endif /* _BMI160_H_ */