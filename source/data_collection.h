#include "mbed.h"

#include "max32630fthr.h"
#include "VL53L1X.h"
#include "BMI160.h"

#include "mbed.h"

#define PIN_SDA P3_4
#define PIN_SCL P3_5
#define PIN_SDA2 P5_7
#define PIN_SCL2 P6_0
#define BMI160_I2C_ADDRESS 0x68

class data_collection {
public:
    // constructor
    data_collection(PinName SDA1, PinName SCL1, PinName XSHUT, PinName SDA2, PinName SCL2, int addr);
    
    bool setup();
    float BMI_baseline();
    uint16_t take_data();
    uint16_t ToF_baseline();
    uint16_t check_ToF();
    float check_accelerometer();

private:
    DigitalOut *xshut;
    VL53L1X *ToF; //(PIN_SDA, PIN_SCL);
    BMI160 *bmi160; //(PIN_SDA2, PIN_SCL2, BMI160_I2C_ADDRESS);
};
