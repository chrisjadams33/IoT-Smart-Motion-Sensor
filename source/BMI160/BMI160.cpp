#include "mbed.h"
#include "BMI160.h"

/* register address defintions */
#define REG_CHIP_ID       0x00 /* chip id */
#define REG_ERR_REG       0x02 
#define REG_PMU_STATUS    0x03
#define REG_DATA_0        0x04 /* MAG_X_LSB */
#define REG_DATA_1        0x05 /* MAG_X_MSB */
#define REG_DATA_2        0x06 /* MAG_Y_LSB */
#define REG_DATA_3        0x07 /* MAG_Y_MSB */
#define REG_DATA_4        0x08 /* MAG_Z_LSB */
#define REG_DATA_5        0x09 /* MAG_Z_MSB */
#define REG_DATA_6        0x0A /* RHALL_LSB */
#define REG_DATA_7        0x0B /* RHALL_MSB */
#define REG_DATA_8        0x0C /* GRY_X_LSB */
#define REG_DATA_9        0x0D /* GRY_X_MSB */
#define REG_DATA_10       0x0E /* GRY_Y_LSB */
#define REG_DATA_11       0x0F /* GRY_Y_MSB */
#define REG_DATA_12       0x10 /* GRY_Z_LSB */
#define REG_DATA_13       0x11 /* GRY_Z_MSB */
#define REG_DATA_14       0x12 /* ACC_X_LSB */
#define REG_DATA_15       0x13 /* ACC_X_MSB */
#define REG_DATA_16       0x14 /* ACC_Y_LSB */
#define REG_DATA_17       0x15 /* ACC_Y_MSB */
#define REG_DATA_18       0x16 /* ACC_Z_LSB */
#define REG_DATA_19       0x17 /* ACC_Z_MSB */
#define REG_SENSORTIME_0  0x18 /* SENSOR_TIME_LSB */
#define REG_SENSORTIME_1  0x19 /* SENSOR_TIME_CSB */
#define REG_SENSORTIME_2  0x1A /* SENSOR_TIME_MSB */
#define REG_STATUS        0x1B
#define REG_INT_STATUS_0  0x1C
#define REG_INT_STATUS_1  0x1D
#define REG_INT_STATUS_2  0x1E
#define REG_INT_STATUS_3  0x1F
#define REG_TEMPERATURE_0 0x20
#define REG_TEMPERATURE_1 0x21
#define REG_FIFO_LENGTH_0 0x22
#define REG_FIFO_LENGTH_1 0x23
#define REG_FIFO_DATA     0x24
#define REG_ACC_CONF      0x40
#define REG_ACC_RANGE     0x41
#define REG_GYR_CONF      0x42
#define REG_GYR_RANGE     0x43
#define REG_MAG_CONF      0x44
#define REG_FIFO_DOWNS    0x45
#define REG_FIFO_CONFIG_0 0x46
#define REG_FIFO_CONFIG_1 0x47
#define REG_MAG_IF_0      0x4B
#define REG_MAG_IF_1      0x4C
#define REG_MAG_IF_2      0x4D
#define REG_MAG_IF_3      0x4E
#define REG_MAG_IF_4      0x4F
#define REG_INT_EN_0      0x50
#define REG_INT_EN_1      0x51
#define REG_INT_EN_2      0x52
#define REG_INT_OUT_CTRL  0x53
#define REG_INT_LATCH     0x54
#define REG_INT_MAP_0     0x55
#define REG_INT_MAP_1     0x56
#define REG_INT_MAP_2     0x57
#define REG_INT_DATA_0    0x58
#define REG_INT_DATA_1    0x59
#define REG_LOWHIGHT_0    0x5A
#define REG_LOWHIGHT_1    0x5B
#define REG_LOWHIGHT_2    0x5C
#define REG_LOWHIGHT_3    0x5D
#define REG_LOWHIGHT_4    0x5E
#define REG_INT_MOTION_0  0x5F
#define REG_INT_MOTION_1  0x60
#define REG_INT_MOTION_2  0x61
#define REG_INT_MOTION_3  0x62
#define REG_INT_TAP_0     0x63
#define REG_INT_TAP_1     0x64
#define REG_INT_ORIENT_0  0x65
#define REG_INT_ORIENT_1  0x66
#define REG_INT_FLAT_0    0x67
#define REG_INT_FLAT_1    0x68
#define REG_FOC_CONF      0x69
#define REG_CONF          0x6A
#define REG_IF_CONF       0x6B
#define REG_PMU_TRIGGER   0x6C
#define REG_SELF_TEST     0x6D
#define REG_NV_CONF       0x70
#define REG_OFFSET_0      0x71
#define REG_OFFSET_1      0x72
#define REG_OFFSET_2      0x73
#define REG_OFFSET_3      0x74
#define REG_OFFSET_4      0x75
#define REG_OFFSET_5      0x76
#define REG_OFFSET_6      0x77
#define REG_STEP_CNT_0    0x78
#define REG_STEP_CNT_1    0x79
#define REG_STEP_CONF_0   0x7A
#define REG_STEP_CONF_1   0x7B
#define REG_CMD           0x7E

#define FLOAT_MAX_16BIT   32768.0

/* 0x00 CHIP_ID reset value = 0xD1 */
/* 0x02 ERR_REG Reports sensor error flags. Flags are cleared when read.
 *      bit[7]   mag_drdy_err
 *      bit[6]   drop_cmd_err Dropped command to Register
 *      bit[5]   i2c_fail_err
 *      bit[4:1] err_code error code
 *        0000: no error
 *        0001: error
 *        0010: error
 *        0011: low-power mode and interrupt uses pre-filtered data
 *        0100-0101: reserved
 *        0110: ODRs of enabled sensors in headerless mode do not match
 *        0111: pre-filtered data are used in low power mode
 *        1000-1111: reserved
 *        The first reported error will be shown in the error code
 *      bit[0]   fatal_err : Chip not operatable
 */
/* 0x03 PMU_STATUS
 *      bit[7:6] (reserved)
 *      bit[5:4] acc_pmu_status
 *      bit[3:2] gyr_pmu_status
 *      bit[1:0] mag_pmu_status
 */
/* 0x04 DATA_0  : MAG_X[7:0]  */
/* 0x05 DATA_1  : MAG_X[15:8] */
/* 0x06 DATA_2  : MAG_Y[7:0]  */
/* 0x07 DATA_3  : MAG_Y[15:8] */
/* 0x08 DATA_4  : MAG_Z[7:0]  */
/* 0x09 DATA_5  : MAG_Z[15:8] */
/* 0x0A DATA_6  : RHALL[7:0]  */
/* 0x0B DATA_7  : RHALL[15:8] */
/* 0x0C DATA_8  : GYR_X[7:0]  */
/* 0x0D DATA_9  : GYR_X[15:8] */
/* 0x0E DATA_10 : GYR_Y[7:0]  */
/* 0x0F DATA_11 : GYR_Y[15:8] */
/* 0x10 DATA_12 : GYR_Z[7:0]  */
/* 0x11 DATA_13 : GYR_Z[15:8] */
/* 0x12 DATA_14 : ACC_X[7:0]  */
/* 0x13 DATA_15 : ACC_X[15:8] */
/* 0x14 DATA_16 : ACC_Y[7:0]  */
/* 0x15 DATA_17 : ACC_Y[15:8] */
/* 0x16 DATA_18 : ACC_Z[7:0]  */
/* 0x17 DATA_19 : ACC_Z[15:8] */
/* 0x18 SENSORTIME_0 : sensor_time[7:0]   */
/* 0x19 SENSORTIME_1 : sensor_time[15:8]  */
/* 0x1A SENSORTIME_2 : sensor_time[23:16] */
/* 0x1B STATUS
 *      bit[7] drdy_acc
 *      bit[6] drdy_gyr
 *      bit[5] drfy_mag
 *      bit[4] nvm_rdy
 *      bit[3] foc_rdy
 *      bit[2] mag_man_op
 *      bit[1] gry_self_test_of
 *      bit[0] (reserved)
 */
/* 0x1C INT_STATUS_0
 *      bit[7] flat_int
 *      bit[6] orient_int
 *      bit[5] s_tap_int
 *      bit[4] d_tap_int
 *      bit[3] pmu_trigger_int
 *      bit[2] anym_int
 *      bit[1] sigmot_int
 *      bit[0] step_int
 */
/* 0x1D INT_STATUS_1
 *      bit[7] nomo_int
 *      bit[6] fwm_int
 *      bit[5] ffull_int
 *      bit[4] drdy_int
 *      bit[3] lowg_int
 *      bit[2] highg_int
 *      bit[1:0] (reserved)
 */
/* 0x1E INT_STATUS_2
 *      bit[7] tap_sign
 *      bit[6] tap_first_z
 *      bit[5] tap_first_y
 *      bit[4] tap_first_x
 *      bit[3] anym_sign
 *      bit[2] anym_first_z
 *      bit[1] anym_first_y
 *      bit[0] anym_first_x
 */
/* 0x1F INT_STATUS_3
 *      bit[7] flat
 *      bit[6] orient_2
 *      bit[5] orient_1
 *      bit[4] orient_0
 *      bit[3] high_sign
 *      bit[2] high_first_z
 *      bit[1] high_first_y
 *      bit[0] high_first_x
 */
/* 0x20 TEMPERATURE_0 temperature[7:0] */
/* 0x21 TEMPERATURE_1 reset value = 0x80 temperature[15:8] */
/* 0x22 FIFO_LENGTH_0 fifo_byte_counter[7:0] */
/* 0x23 FIFO_LENGTH_1
 *      bit[7:3] (reserved)
 *      bit[2:0] fifo_byte_counter[10:8] 
 */
/* 0x40 ACC_CONF reset value 0x28
 *      bit[7]   acc_us
 *      bit[6:4] acc_bwp
 *      bit[3:0] acc_odr
 */
/* 0x41 ACC_RANGE reset value = 0x03
 *      bit[7:4] (reserved)
 *      bit[3:0] acc_range
 */
/* 0x42 GYR_CONF reset value = 0x28
 *      bit[7:6] (reserved)
 *      bit[5:4] gyr_bwp
 *      bit[3:0] gyr_odr
 */
/* 0x43 GYR_RANGE
 *      bit[7:3] (reserved)
 *      bit[2:0] gyr_range
 */
/* 0x44 MAG_CONF reset value = 0x0B
 *      bit[7:4] (reserved)
 *      bit[3:0] mag_odr
 */
/* 0x45 FIFO_DOWNS reset value = 0x88
 *      bit[7] acc_fifo_filt_data
 *      bit[6:4] acc_fifo_downs
 *      bit[3] gyr_fifo_filt_data
 *      bit[2:0] gyr_fifo_downs
 */
/* 0x46 FIFO_CONFIG_0 reset value = 0x80
 *      bit[7:0] fifo_water_mark
 */
/* 0x47 FIFO_CONFIG_1 reset value = 0x10
 *      bit[7] fifo_gyr_en
 *      bit[6] fifo_acc_en
 *      bit[5] fifo_mag_en
 *      bit[4] fifo_header_en
 *      bit[3] fifo_tag_int1_en
 *      bit[2] fifo_tag_int2_en
 *      bit[1] fifo_time_en
 *      bit[0] (reserved)
 */
/* 0x4B MAG_IF_0 reset value = 0x20
 *      bit[7:1] i2c_device_addr
 */
/* 0x4C MAG_IF_1 reset value = 0x80
 *      bit[7] mag_manual_en
 *      bit[6] (reserved)
 *      bit[5:2] mag_offset
 *      bit[1:0] mag_rd_burst
 */
/* 0x4D MAG_IF_2 reset value = 0x42
 *      bit[7:0] read_addr
 */
/* 0x4E MAG_IF_3 reset value = 0x4C
 *      bit[7:0] write_addr
 */
/* 0x4F MAG_IF_4 bit[7:0] write data */
/* 0x50 INT_EN_0 
 *      bit[7] int_flat_en
 *      bit[6] int_orient_en
 *      bit[5] int_s_tap_en
 *      bit[4] int_d_tap_en
 *      bit[3] (reserved)
 *      bit[2] int_anymo_z_en
 *      bit[1] int_anymo_y_en
 *      bit[0] int_anymo_x_en
 */
/* 0x51 INT_EN_1
 *      bit[7] (reserved)
 *      bit[6] int_fwm_en
 *      bit[5] int_ffull_en
 *      bit[4] int_drdy_en
 *      bit[3] int_low_en
 *      bit[2] int_highg_z_en
 *      bit[1] int_highg_y_en
 *      bit[0] int_highg_x_en
 */
/* 0x52 INT_EN_2
 *      bit[7:4] (reserved)
 *      bit[3] int_step_det_en
 *      bit[2] int_nomoz_en
 *      bit[1] int_nomoy_en
 *      bit[0] int_nomox_en
 */
/* 0x53 INT_OUT_CTRL
 *      bit[7] int2_output_en
 *      bit[6] int2_od
 *      bit[5] int2_lvl
 *      bit[4] int2_edge_ctrl
 *      bit[3] int1_output_en
 *      bit[2] int1_od
 *      bit[1] int1_lvl
 *      bit[0] itn1_edge_ctrl
 */
/* 0x54 INT_LATCH
 *      bit[7:6] (reserved)
 *      bit[5] int2_input_en
 *      bit[4] int1_input_en
 *      bit[3:0] int_latch
 */
/* 0x55 INT_MAP_0
 *      bit[7] int1_flat
 *      bit[6] int1_orient
 *      bit[5] int1_s_tap
 *      bit[4] int1_d_tap
 *      bit[3] int1_nomotion
 *      bit[2] int1_anymotion
 *      bit[1] int1_highg
 *      bit[0] int1_lowg_step
 */
/* 0x56 INT_MAP_1
 *      bit[7] int1_drdy
 *      bit[6] int1_fwm
 *      bit[5] int1_ffull
 *      bit[4] int1_pmu_trig
 *      bit[3] int2_drdy
 *      bit[2] itn2_fwm
 *      bit[1] int2_ffull
 *      bit[0] int2_pmu_trig
 */
/* 0x57 INT_MAP_2
 *      bit[7] int2_flat
 *      bit[6] int2_orient
 *      bit[5] int2_s_tap
 *      bit[4] int2_d_tap
 *      bit[3] int2_nomotion
 *      bit[2] int2_anymotion
 *      bit[1] int2_highg
 *      bit[0] int2_lowg_steop
 */
/* 0x58 INT_DATA_0
 *      bit[7] int_low_high_src
 *      bit[6:4] (reserved)
 *      bit[3] int_tap_src
 *      bit[2:0] (reserved)
 */
/* 0x59 INT_DATA_1
 *      bit[7] int_motion_src
 *      bit[6:0] (reseved)
 */
/* 0x5A INT_LOWHIGH_0 reset value = 0x07
 *      bit[7:0] int_low_dur
 */
/* 0x5B INT_LOWHIGH_1 reset value = 0x30
 *      bit[7:0] int_low_th
 */
/* 0x5C INT_LOWHIGH_2 reset value = 0x81
 *      bit[7:6] int_high_hy
 *      bit[5:3] (reserved)
 *      bit[2]   int_low_mode
 *      bit[1:0] int_low_hy
 */
/* 0x5D INT_LOWHIGH_3 reset value = 0x0B
 *      bit[7:0] int_high_dur
 */
/* 0x5E INT_LOWHIGH_4 reset value = 0xC0
 *      bit[7:0] int_high_th
 */
/* 0x5F INT_MOTION_0 
 *      bit[7:2] int_slo_nomo_dur
 *      bit[1:0] int_anym_dur
 */
/* 0x60 INT_MOTION_1 reset value = 0x14
 *      bit[7:0] int_anymo_th
 */
/* 0x61 INT_MOTION_2 reset value = 0x14
 *      bit[7:0] int_slo_nomo_th
 */
/* 0x62 INT_MOTION_3 reset value = 0x24
 *      bit[7:6] (reserved)
 *      bit[5:4] int_sig_mot_proof
 */
/* 0x63 INT_TAP_0 reset value = 0x04
 *      bit[7] int_tap_quiet
 *      bit[6] int_tap_shock
 *      bit[5:3] (reserved)
 *      bit[2:0] int_tap_dur
 */
/* 0x64 INT_TAP_1 reset value = 0x0A
 *      bit[7:5] (reserved)
 *      bit[4:0] int_tap_th
 */
/* 0x65 INT_ORIENT_0 reset value = 0x18
 *      bit[7:4] int_orient_hy
 *      bit[3:2] int_orient_blocking
 *      bit[1:0] int_orient_mode
 */
/* 0x66 INT_ORIENT_1 reset value = 0x48
 *      bit[7] int_orient_axes_ex
 *      bit[6] int_orient_ud_en
 *      bit[5:0] int_orient_theta
 */
/* 0x67 INT_FLAT_0 reset value = 0x08
 *      bit[7:6] (reserved)
 *      bit[5:0] int_flat_theta
 */
/* 0x68 INT_FLAT_1 reset value = 0x11
 *      bit[7:6] (reserved)
 *      bit[5:4] int_flat_hold
 *      bit[3:0] int_flat_hy
 */
/* 0x69 FOC_CONF 
 *      bit[7] (reserved)
 *      bit[6] foc_gyr_en
 *      bit[5:4] foc_acc_x
 *      bit[3:2] foc_acc_y
 *      bit[1:0] foc_acc_z
 */
/* 0x6A CONF
 *      bit[7:2] (reserved)
 *      bit[1]  nvm_prog_en
 *      bit[0] (reserved)
 */
/* 0x6B IF_CONF
 *      bit[7:6] (reserved)
 *      bit[5:4] if_mode
 *      bit[3:1] (reserved)
 *      bit[0] spi3
 */
/* 0x6C PMU_TRIGGER
 *      bit[7] (reserved)
 *      bit[6] wakeup_int
 *      bit[5] gyr_sleep_state
 *      bit[4:3] gyr_wakeup_trigger
 *      bit[2:0] gyr_sleep_trigger
 */
/* 0x6D SELF_TEST
 *      bit[7:5] (reserved)
 *      bit[4] gyr_self_test_enable
 *      bit[3] acc_self_test_amp
 *      bit[2] acc_self_test_sign
 *      bit[1:0] acc_self_test_enable
 */
/* 0x70 NV_CONF
 *      bit[7:4] (reserved)
 *      bit[3] u_spare_0
 *      bit[2] i2c_wdt_en
 *      bit[1] i2c_wdt_sel
 *      bit[0] i2c_spi_en
 */
/* 0x71 OFFSET_0 bit[7:0] off_acc_x */
/* 0x72 OFFSET_1 bit[7:0] off_acc_y */
/* 0x73 OFFSET_2 bit[7:0] off_acc_z */
/* 0x74 OFFSET_3 bit[7:0] off_gyr_x_7_0 */
/* 0x75 OFFSET_4 bit[7:0] off_gyr_y_7_0 */
/* 0x76 OFFSET_5 bit[7:0] off_gyr_z_7_0 */
/* 0x77 OFFSET_6
 *      bit[7] gyr_off_en
 *      bit[6] acc_off_en
 *      bit[5:4] off_gyr_Z_9_8
 *      bit[3:2] off_gyr_y_9_8
 *      bit[1:0] off_gyr_x_9_8
 */
/* 0x78 STEP_CNT_0 bit[7:0] step_cnt_7_0 */
/* 0x79 STEP_CNT_1 bit[7:0] step_cnt_15_8 */
/* 0x7A STEP_CONF_0 reset value = 0x15
 *      bit[7:5] (alpha)
 *      bit[4:3] min_threshold
 *      bit[2:0] steptime_min
 */
/* 0x7B STEP_CONF_1 reset value = 0x03
 *      bit[7:4] (reserved)
 *      bit[3] step_cnt_en
 *      bit[2:0] min_step_buf
 */
/* 0x7E CMD bit[7:0] cmd */

 
 

BMI160::BMI160(PinName sda, PinName scl, int addr) 
{
    m_i2c = new I2C(sda, scl) ;
    m_addr = (addr << 1) ;
    m_spi = 0 ;
    m_cs = 0 ;
    init() ;
    
    setCMD(0x10 | ACC_PMU_LOWPOWER) ;
    wait(0.1) ;
    setCMD(0x14 | GYR_PMU_SUSPEND) ;
    wait(0.1) ;
    setCMD(0x08 | MAG_PMU_SUSPEND) ;
    wait(0.1) ;
    setCMD(0x03) ; /* start_foc */
    wait(0.1) ;
}

BMI160::BMI160(PinName sck, PinName miso, PinName mosi, PinName cs) 
{
    m_cs = new DigitalOut(cs, 1) ;
    m_spi = new SPI(mosi, miso, sck) ;
    m_spi->format(8, 3) ;
    m_i2c = 0 ;
    m_addr = 0 ;
    init() ;
}

BMI160::~BMI160() 
{
    if (m_spi) { 
        delete m_spi ;
        delete m_cs ; 
    }
    if (m_i2c) { 
        delete m_i2c ; 
        m_addr = 0 ;
    }
}

void BMI160::i2c_readRegs(int addr, uint8_t *data, int len) 
{
    char t[1] = {addr} ;
    m_i2c->write(m_addr, t, 1, true) ;
    m_i2c->read(m_addr, (char*)data, len) ;
}

void BMI160::i2c_writeRegs(uint8_t *data, int len) 
{
   m_i2c->write(m_addr, (char *)data, len) ;
}

void BMI160::spi_readRegs(int addr, uint8_t *data, int len) 
{
    *m_cs = 0 ;
    m_spi->write(addr | 0x80) ;
    for (int i = 0 ; i < len ; i++ ) {    
      data[i] = m_spi->write((addr+i)|0x80) ; 
    } 
    m_spi->write(0x00) ; // to terminate read mode
    *m_cs = 1 ;
}

void BMI160::spi_writeRegs(uint8_t *data, int len) 
{
   *m_cs = 0 ;
   for (int i = 0 ; i < len-1 ; i++ ) {
      m_spi->write((data[0]+i)^0x80) ; /* register address */
      m_spi->write(data[i+1]) ; /* data to write */

   }
   *m_cs = 1 ;
}

void BMI160::readRegs(int addr, uint8_t *data, int len) 
{
    if (m_spi) {
        spi_readRegs(addr, data, len) ;
    } else if (m_i2c) {
        i2c_readRegs(addr, data, len) ;
    }
}

void BMI160::writeRegs(uint8_t *data, int len) 
{
    if (m_spi) {
        spi_writeRegs(data, len) ;
    } else if (m_i2c) {
        i2c_writeRegs(data, len) ;
    }
}

void BMI160::init(void)
{
    acc_range = getAccRange() ;
    gyr_range = getGyrRange() ;
}


void BMI160::setCMD(uint8_t cmd) 
{
    uint8_t data[2] ;
    data[0] = REG_CMD ;
    data[1] = cmd ;
    writeRegs(data, 2) ;
}

uint8_t BMI160::getStatus(void)
{
  uint8_t status ;
  readRegs(REG_STATUS, &status, 1) ;
  return(status) ;
}
  
uint8_t BMI160::getChipID(void) 
{
    uint8_t data ;
    readRegs(REG_CHIP_ID, &data, 1) ;
    return( data ) ;
}

uint8_t BMI160::getAccRange(void)
{
    uint8_t data = 0 ;
    uint8_t range = 0 ;
    readRegs(REG_ACC_RANGE, &data, 1) ;
    switch(data & 0x0F) {
    case 3: /* +/- 2g */
        range = 2 ; 
        break ;
    case 5: /* +/- 4g */
        range = 4 ;
        break ;
    case 8: /* +/- 8g */
        range = 8 ;
        break ;
    default:
        printf("illegal Acc Range %X detected\n", data & 0x0F) ;
        break ;
    }
    acc_range = range ;
    return(range) ;
}

int16_t BMI160::getGyrRange(void)
{
    uint8_t data = 0 ;
    int16_t range = 0 ;
    readRegs(REG_GYR_RANGE, &data, 1) ;
    switch(data & 0x07) {
    case 0:
        range = 2000 ;
        break ;
    case 1:
        range = 1000 ;
        break ;
    case 2:
        range = 500 ;
        break ;
    case 3:
        range = 250 ;
        break ;
    case 4:
        range = 125 ;
        break ;
    default:
        printf("illegal Gyr Range %d detected\n", data & 0x07) ;
        break ;
    }
    gyr_range = range ;
    return(range) ;
}

int16_t BMI160::getAccRawX(void) 
{
    uint8_t data[2] ;
    int16_t value = 0 ;
    readRegs(REG_DATA_14, data, 2) ;
    value = (data[1] << 8) | data[0] ;
    return( value ) ;
}

int16_t BMI160::getAccRawY(void) 
{
    uint8_t data[2] ;
    int16_t value = 0 ;
    readRegs(REG_DATA_16, data, 2) ;
    value = (data[1] << 8) | data[0] ;
    return( value ) ;
}

int16_t BMI160::getAccRawZ(void) 
{
    uint8_t data[2] ;
    int16_t value = 0 ;
    readRegs(REG_DATA_18, data, 2) ;
    value = (data[1] << 8) | data[0] ;
    return( value ) ;
}

void BMI160::getAccRaw(int16_t *value)
{
    uint8_t data[6] ;
    readRegs(REG_DATA_14, data, 6) ;
    value[0] = (data[1] << 8) | data[0] ;
    value[1] = (data[3] << 8) | data[2] ;
    value[2] = (data[5] << 8) | data[4] ;
} 

int16_t BMI160::getGyrRawX(void) 
{
    uint8_t data[2] ;
    int16_t value = 0 ;
    readRegs(REG_DATA_8, data, 2) ;
    value = (data[1] << 8) | data[0] ;
    return( value ) ;
}

int16_t BMI160::getGyrRawY(void) 
{
    uint8_t data[2] ;
    int16_t value = 0 ;
    readRegs(REG_DATA_10, data, 2) ;
    value = (data[1] << 8) | data[0] ;
    return( value ) ;
}

int16_t BMI160::getGyrRawZ(void) 
{
    uint8_t data[2] ;
    int16_t value = 0 ;
    readRegs(REG_DATA_12, data, 2) ;
    value = (data[1] << 8) | data[0] ;
    return( value ) ;
}

void BMI160::getGyrRaw(int16_t *value)
{
    uint8_t data[6] ;
    readRegs(REG_DATA_8, data, 6) ;
    value[0] = (data[1] << 8) | data[0] ;
    value[1] = (data[3] << 8) | data[2] ;
    value[2] = (data[5] << 8) | data[4] ;
} 

float BMI160::getAccX(void)
{
    float value ;
    value = acc_range * getAccRawX() / FLOAT_MAX_16BIT ;
    return(value) ;
}

float BMI160::getAccY(void)
{
    float value ;
    value = acc_range * getAccRawY() / FLOAT_MAX_16BIT ;
    return(value) ;
}

float BMI160::getAccZ(void)
{
    float value ;
    value = acc_range * getAccRawZ() / FLOAT_MAX_16BIT ;
    return(value) ;
}

void BMI160::getAcc(float *value)
{
    int16_t data[3] ;
    getAccRaw(data) ;
    value[0] = acc_range * data[0] / FLOAT_MAX_16BIT ;
    value[1] = acc_range * data[1] / FLOAT_MAX_16BIT ;
    value[2] = acc_range * data[2] / FLOAT_MAX_16BIT ;
}

float BMI160::getGyrX(void)
{
    float value ;
    value = gyr_range * getGyrRawX() / FLOAT_MAX_16BIT ;
    return(value) ;
}

float BMI160::getGyrY(void)
{
    float value ;
    value = gyr_range * getGyrRawY() / FLOAT_MAX_16BIT ;
    return(value) ;
}

float BMI160::getGyrZ(void)
{
    float value ;
    value = gyr_range * getGyrRawZ() / FLOAT_MAX_16BIT ;
    return(value) ;
}

void BMI160::getGyr(float *value)
{
    int16_t data[3] ;
    getGyrRaw(data) ;
    value[0] = gyr_range * data[0] / FLOAT_MAX_16BIT ;
    value[1] = gyr_range * data[1] / FLOAT_MAX_16BIT ;
    value[2] = gyr_range * data[2] / FLOAT_MAX_16BIT ;
}