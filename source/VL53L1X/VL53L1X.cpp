#include "VL53L1X.h"
#include "mbed.h"
 
//Serial pc(USBTX,USBRX);
//DigitalOut led1(LED1);
 
uint8_t configBlock[] = {
  0x29, 0x02, 0x10, 0x00, 0x28, 0xBC, 0x7A, 0x81, //8
  0x80, 0x07, 0x95, 0x00, 0xED, 0xFF, 0xF7, 0xFD, //16
  0x9E, 0x0E, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00, //24
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x00, //32
  0x28, 0x00, 0x0D, 0x0A, 0x00, 0x00, 0x00, 0x00, //40
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, //48
  0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, //56
  0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x02, //64
  0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, //72
  0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x02, 0x00, //80
  0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, //88
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x91, 0x0F, //96
  0x00, 0xA5, 0x0D, 0x00, 0x80, 0x00, 0x0C, 0x08, //104
  0xB8, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x10, 0x00, //112
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, //120
  0x0D, 0x0E, 0x0E, 0x01, 0x00, 0x02, 0xC7, 0xFF, //128
  0x8B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40 //129 - 135 (0x81 - 0x87)
};
 
VL53L1X::VL53L1X(PinName SDA, PinName SCL) : _i2c(SDA,SCL){
    //Set I2C fast and bring reset line high
   _i2c.frequency(400000);
    _deviceAddress = defaultAddress_VL53L1X << 1;
    }
    
bool VL53L1X::begin()
{ 
  //Check the device ID
  uint16_t modelID = readRegister16(VL53L1_IDENTIFICATION__MODEL_ID);
  if (modelID != 0xEACC){
    return (false);
  }
  softReset();
 
  //Polls the bit 0 of the FIRMWARE__SYSTEM_STATUS register to see if the firmware is ready
  int counter = 0;
  int  Firmware = readRegister16(VL53L1_FIRMWARE__SYSTEM_STATUS);
  printf("Firmware = %x\r\n", Firmware);
  while ((Firmware & 0x01) == 0)
  {
    Firmware = readRegister16(VL53L1_FIRMWARE__SYSTEM_STATUS);
    printf("Firmware = %x\r\n", Firmware);
    if (counter++ == 100) return (false); //Sensor timed out
    wait(.1);
  }
 
  //Set I2C to 2.8V mode. In this mode 3.3V I2C is allowed.
  uint16_t result = readRegister16(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG);
  result = (result & 0xFE) | 0x01;
  writeRegister16(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG, result);
 
  //Gets trim resistors from chip
  for (uint16_t i = 0; i < 36; i++) {
      uint8_t regVal = readRegister(i + 1);
      configBlock[i] = regVal;
  }
  
  startMeasurement();
  return (true); //Sensor online!

}
 
    
void VL53L1X::startMeasurement(uint8_t offset)
{
  offset = 0; //Start at a location within the configBlock array
  uint8_t address = 1 + offset; //Start at memory location 0x01, add offset
  char data_write[32];
  uint8_t leftToSend = sizeof(configBlock) - offset;
  while (leftToSend > 0)
  {

    data_write[0] = 0; //MSB of register address 
    data_write[1] = address; //LSB of register address 
    
    uint8_t toSend = 30; //Max I2C buffer on Arduino is 32, and we need 2 bytes for address
    if (toSend > leftToSend) toSend = leftToSend;
    for(int x = 0; x < toSend; x++)
    {
        data_write[x+2] = configBlock[x+address-1];
    }
    _i2c.write(_deviceAddress, data_write, toSend+2); 

    leftToSend -= toSend;
    address += toSend;
  }
}
 
bool VL53L1X::newDataReady(void)
{
  int read = readRegister(VL53L1_GPIO__TIO_HV_STATUS);
  if (read != 0x03) return(true); //New measurement!
  return(false); //No new data
}
 
//Reset sensor via software
void VL53L1X::softReset()
{
  writeRegister(VL53L1_SOFT_RESET, 0x00); //Reset
  wait(.001); //Driver uses 100us
  writeRegister(VL53L1_SOFT_RESET, 0x01); //Exit reset
}
 
uint16_t VL53L1X::getDistance()
{
  return (readRegister16(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0));
}
 
uint16_t VL53L1X::getSignalRate()
{
  //From vl53l1_api.c line 2041
  uint16_t reading = readRegister16(VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);// << 9; //FIXPOINT97TOFIXPOINT1616
  //float signalRate = (float)reading/65536.0;
  return (reading);
}
 
void VL53L1X::setDistanceMode(uint8_t mode)
{
    uint8_t periodA;
    uint8_t periodB;
    uint8_t phaseHigh;
    uint8_t phaseInit;
    
    switch (mode)
    {
        case 0:
          periodA = 0x07;
          periodB = 0x05;
          phaseHigh = 0x38;
          phaseInit = 6;
          break;
        case 1:
          periodA = 0x0B;
          periodB = 0x09;
          phaseHigh = 0x78;
          phaseInit = 10;
          break;
        case 2:
          periodA = 0x0F;
          periodB = 0x0D;
          phaseHigh = 0xB8;
          phaseInit = 14;
          break;
        //If user inputs wrong range, we default to long range
        default:
          periodA = 0x0F;
          periodB = 0x0D;
          phaseHigh = 0xB8;
          phaseInit = 14;
          break;
    }
    //timing
    writeRegister(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, periodA);
    writeRegister(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B , periodB);
    writeRegister(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, phaseHigh);
    
    //dynamic
    writeRegister(VL53L1_SD_CONFIG__WOI_SD0 , periodA);
    writeRegister(VL53L1_SD_CONFIG__WOI_SD1, periodB);
    writeRegister(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, phaseInit);
    writeRegister(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, phaseInit);
    
    _distanceMode = mode;
}
uint8_t VL53L1X::getDistanceMode()
{
    return _distanceMode;
}
 
uint8_t VL53L1X::getRangeStatus()
{
#define VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE  ( 1)
#define VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE    ( 2)
#define VL53L1_DEVICEERROR_NOVHVVALUEFOUND             ( 3)
#define VL53L1_DEVICEERROR_MSRCNOTARGET                ( 4)
#define VL53L1_DEVICEERROR_RANGEPHASECHECK             ( 5)
#define VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK         ( 6)
#define VL53L1_DEVICEERROR_PHASECONSISTENCY            ( 7)
#define VL53L1_DEVICEERROR_MINCLIP                     ( 8)
#define VL53L1_DEVICEERROR_RANGECOMPLETE               ( 9)
#define VL53L1_DEVICEERROR_ALGOUNDERFLOW               ( 10)
#define VL53L1_DEVICEERROR_ALGOOVERFLOW                ( 11)
#define VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD        ( 12)
#define VL53L1_DEVICEERROR_USERROICLIP                 ( 13)
#define VL53L1_DEVICEERROR_REFSPADCHARNOTENOUGHDPADS   ( 14)
#define VL53L1_DEVICEERROR_REFSPADCHARMORETHANTARGET   ( 15)
#define VL53L1_DEVICEERROR_REFSPADCHARLESSTHANTARGET   ( 16)
#define VL53L1_DEVICEERROR_MULTCLIPFAIL                ( 17)
#define VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY        ( 18)
#define VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK ( 19)
#define VL53L1_DEVICEERROR_EVENTCONSISTENCY            ( 20)
#define VL53L1_DEVICEERROR_MINSIGNALEVENTCHECK         ( 21)
#define VL53L1_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE  ( 22)
 
#define VL53L1_RANGESTATUS_RANGE_VALID       0 /*!<The Range is valid. */
#define VL53L1_RANGESTATUS_SIGMA_FAIL        1 /*!<Sigma Fail. */
#define VL53L1_RANGESTATUS_SIGNAL_FAIL       2 /*!<Signal fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED 3 /*!<Target is below minimum detection threshold. */
#define VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL      4 /*!<Phase out of valid limits -  different to a wrap exit. */
#define VL53L1_RANGESTATUS_HARDWARE_FAIL     5 /*!<Hardware fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL  6 /*!<The Range is valid but the wraparound check has not been done. */
#define VL53L1_RANGESTATUS_WRAP_TARGET_FAIL     7 /*!<Wrapped target - no matching phase in other VCSEL period timing. */
#define VL53L1_RANGESTATUS_PROCESSING_FAIL      8 /*!<Internal algo underflow or overflow in lite ranging. */
#define VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL      9 /*!<Specific to lite ranging. */
#define VL53L1_RANGESTATUS_SYNCRONISATION_INT     10 /*!<1st interrupt when starting ranging in back to back mode. Ignore data. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE   11 /*!<All Range ok but object is result of multiple pulses merging together.*/
#define VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL  12 /*!<Used  by RQL  as different to phase fail. */
#define VL53L1_RANGESTATUS_MIN_RANGE_FAIL     13 /*!<User ROI input is not valid e.g. beyond SPAD Array.*/
#define VL53L1_RANGESTATUS_RANGE_INVALID      14 /*!<lld returned valid range but negative value ! */
#define VL53L1_RANGESTATUS_NONE        255 /*!<No Update. */
 
  //Read status
  uint8_t measurementStatus = (readRegister(VL53L1_RESULT__RANGE_STATUS) & 0x1F);
  //Convert status from one to another - From vl53l1_api.c
  switch (measurementStatus) {
    case VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY:
      measurementStatus = VL53L1_RANGESTATUS_SYNCRONISATION_INT;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
      measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGEPHASECHECK:
      measurementStatus = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
      break;
    case VL53L1_DEVICEERROR_MSRCNOTARGET:
      measurementStatus = VL53L1_RANGESTATUS_SIGNAL_FAIL;
      break;
    case VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK:
      measurementStatus = VL53L1_RANGESTATUS_SIGMA_FAIL;
      break;
    case VL53L1_DEVICEERROR_PHASECONSISTENCY:
      measurementStatus = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD:
      measurementStatus = VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
      break;
    case VL53L1_DEVICEERROR_MINCLIP:
      measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE:
      measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID;
      break;
    default:
      measurementStatus = VL53L1_RANGESTATUS_NONE;
  }
 
  return measurementStatus;
}
 
uint8_t VL53L1X::readRegister(uint16_t registerAddr)
{
  uint8_t data;
  char data_write[2];
  char data_read[1];
  data_write[0] = (registerAddr >> 8) & 0xFF; //MSB of register address 
  data_write[1] = registerAddr & 0xFF; //LSB of register address 
  _i2c.write(_deviceAddress, data_write, 2,0); 
  _i2c.read(_deviceAddress,data_read,1,false);
  //Read Data from selected register
  data=data_read[0];
  return data;
}
 
uint16_t VL53L1X::readRegister16(uint16_t registerAddr)
{
  uint8_t data_low;
  uint8_t data_high;
  uint16_t data;
 
  char data_write[2];
  char data_read[2];
  data_write[0] = (registerAddr >> 8) & 0xFF; //MSB of register address 
  data_write[1] = registerAddr & 0xFF; //LSB of register address 
  _i2c.write(_deviceAddress, data_write, 2,0); 
  _i2c.read(_deviceAddress,data_read,2,false);
  data_high = data_read[0]; //Read Data from selected register
  data_low = data_read[1]; //Read Data from selected register
  data = (data_high << 8)|data_low;
 
  return data;
}
 
void VL53L1X::writeRegister(uint16_t registerAddr, uint8_t data)
{
    char data_write[3];
    data_write[0] = (registerAddr >> 8) & 0xFF; //MSB of register address 
    data_write[1] = registerAddr & 0xFF; //LSB of register address 
    data_write[2] = data & 0xFF; 
    _i2c.write(_deviceAddress, data_write, 3); 
}
 
void VL53L1X::writeRegister16(uint16_t registerAddr, uint16_t data)
{
    char data_write[4];
    data_write[0] = (registerAddr >> 8) & 0xFF; //MSB of register address 
    data_write[1] = registerAddr & 0xFF; //LSB of register address 
    data_write[2] = (data >> 8) & 0xFF;
    data_write[3] = data & 0xFF; 
    _i2c.write(_deviceAddress, data_write, 4); 
}
 