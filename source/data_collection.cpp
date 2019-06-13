#include "mbed.h"

#include "max32630fthr.h"
#include "VL53L1X.h"
#include "BMI160.h"

#include "data_collection.h"

#define PIN_SDA P3_4
#define PIN_SCL P3_5
#define PIN_SDA2 P5_7
#define PIN_SCL2 P6_0
#define BMI160_I2C_ADDRESS 0x68
#define SHUTDOWN_PIN P3_3

data_collection::data_collection(PinName SDA1, PinName SCL1, PinName XSHUT, PinName SDA2, PinName SCL2, int addr){
    xshut = new DigitalOut(XSHUT, 0);
    ToF = new VL53L1X(SDA1, SCL1);
    bmi160 = new BMI160(SDA2, SCL2, addr) ;
}

bool data_collection::setup(){
    
    *xshut = 1;
    ToF->softReset();
    ToF->setDistanceMode(0);//Defaults to long range
    
    if (!ToF->begin())
    {
        *xshut = 0;
        return false;
    }
    
    *xshut = 0 ;      //ToF HW standby
    return true;
}

float data_collection::BMI_baseline(){
    int id = 0;
    float a[3], a_sum, a_total, a_init;
    
    for( int i = 0; i < 8; i++ ) {
        //get sensor data
        bmi160->getAcc(a) ;
        a_sum = abs(a[0]) + abs(a[1]) + abs(a[2]);
        a_total += a_sum;
        wait(0.05);
    } 
    
    a_init = a_total/8;
    return a_init;
}

uint16_t data_collection::take_data()
{
    uint16_t dist = 0;
    ToF->startMeasurement(0); //Write a block of bytes to the sensor to configure it to take a measurement
    if (ToF->newDataReady() == false)
        dist = ToF->getDistance();   //Returns the results from the last measurement, distance in mm
    else
        dist = 0;
    return dist;
}

uint16_t data_collection::ToF_baseline(){
    uint16_t d = 0;
    int d_tot = 0;
    int i = 0;
    *xshut = 1;
    double dist_mm, dist_in;
    while (i < 15) {
        //get sensor data
        d = take_data() ;
        d_tot += d;
        if (d < 20){
            d_tot -= d;
            i --;
        }
        i++;
        wait(0.05);
    } 
    d = d_tot/15;
    *xshut = 0;
    return d;
}

uint16_t data_collection::check_ToF(){
    uint16_t d = 0;
    int d_tot = 0;
    int i = 0;

    *xshut = 1;
    while (i < 8) {
        //get sensor data
        d = take_data() ;
        d_tot += d;
        if (d < 15){
            d_tot -= d;
            i --;
        }
        i++;
        wait(0.05);
    }
    d = d_tot/8;;
    *xshut = 0;
    d = 0.8672*d+4;
    return d;
}

float data_collection::check_accelerometer(){
    float a[3], a_sum, a_total;
    a_total = 0;
    //printf("Ready\n\r") ;
    for( int i = 0; i < 3; i++ ) {
        //get sensor data
        bmi160->getAcc(a) ;
        a_sum = abs(a[0]) + abs(a[1]) + abs(a[2]);
        a_total += a_sum;
        wait(0.01);
    }
    a_sum = a_total/3;
    return a_sum;
    
}
