/*
 * 
 * Copyright (c) 2020, Andres Gomez, Miromico AG
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "sys/rtimer.h"
#include "sensor-common.h"
#include "board-i2c.h"

#include "ti-lib.h"

#include "lis3dh.h"
#include "lis3dh-sensor.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/


#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#if !(CC26XX_UART_CONF_ENABLE)
#warning "running in debug configuration while serial is NOT enabled!"
#endif
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
/* Sensor selection/deselection */
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_1, LIS3DH_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()
/*---------------------------------------------------------------------------*/
/* Delay */
#define delay_ms(i) (ti_lib_cpu_delay(8000 * (i)))
/*---------------------------------------------------------------------------*/
#define SENSOR_STATE_DISABLED     0
#define SENSOR_STATE_BOOTING      1
#define SENSOR_STATE_ENABLED      2

static int state = SENSOR_STATE_DISABLED;
/*---------------------------------------------------------------------------*/
/* 3 16-byte words for all sensor readings */
#define SENSOR_DATA_BUF_SIZE   3

static uint16_t sensor_value[SENSOR_DATA_BUF_SIZE];
/*---------------------------------------------------------------------------*/
/*
 * Wait SENSOR_BOOT_DELAY ticks for the sensor to boot and
 * SENSOR_STARTUP_DELAY for readings to be ready
 * Gyro is a little slower than Acc
 */
#define SENSOR_BOOT_DELAY     10
#define SENSOR_STARTUP_DELAY  10

static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
/* Wait for the MPU to have data ready */
rtimer_clock_t t0;

int32_t ret;
uint8_t whoamI=0;
uint8_t i2c_buff[4];

/*
 * Wait timeout in rtimer ticks. This is just a random low number, since the
 * first time we read the sensor status, it should be ready to return data
 */
#define READING_WAIT_TIMEOUT 10

/**
 * \brief Read data from the accelerometer - X, Y, Z - 3 words
 * \return True if a valid reading could be taken, false otherwise
 */
static bool
acc_read(uint16_t *data)
{
  bool success;

  // if(interrupt_status & BIT_RAW_RDY_EN) 
  {
    /* Burst read of all accelerometer values */
    SENSOR_SELECT();
    success = sensor_common_read_reg(LIS3DH_OUT_X_L, (uint8_t *)data, DATA_SIZE);
    SENSOR_DESELECT();

    if(success) {
      // convert_to_le((uint8_t *)data, DATA_SIZE);
      PRINTF("Read Data\n");
    } else {
      PRINTF("Error reading data\n");
      sensor_common_set_error_data((uint8_t *)data, DATA_SIZE);
    }
  } 
  // else {
  //   /* Data not ready */
  //   success = false;
  // }

  return success; 
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

int32_t lis3dh_get(uint8_t *buff, uint8_t len){
  ret = 0;
  // board_i2c_select(BOARD_I2C_INTERFACE_1, LIS3DH_I2C_ADDRESS);
  // ret = sensor_common_read_reg(LIS3DH_WHO_AM_I, buff, 1);
  board_i2c_select(BOARD_I2C_INTERFACE_1, LIS3DH_I2C_ADDRESS);
  ret = sensor_common_read_reg(LIS3DH_WHO_AM_I, buff, len);
  board_i2c_deselect();

  return ret;
}

/**
  * @brief  DeviceWhoamI .[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_device_id_get(uint8_t *buff)
{
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_WHO_AM_I, buff, 1);
  SENSOR_DESELECT();

  return ret;
}
/**
  * @brief  Block Data Update.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_block_data_update_set(uint8_t val)
{
  bool success;
  int32_t ret;
  lis3dh_ctrl_reg4_t ctrl_reg4;

    /* Burst read of all accelerometer values */
    SENSOR_SELECT();
    success = sensor_common_read_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);
    SENSOR_DESELECT();

    if(success) {
      ctrl_reg4.bdu = val;
      success = sensor_common_write_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);
      ret = 0;
    } else {
      ret = -1;
    }

  return ret;
}

/**
  * @brief  Block Data Update.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_block_data_update_get(uint8_t *val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);
  SENSOR_DESELECT();
  *val = (uint8_t)ctrl_reg4.bdu;
  return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odr in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_rate_set(lis3dh_odr_t val)
{
  lis3dh_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG1, (uint8_t *)&ctrl_reg1,1);


  if (ret == 0) {
    ctrl_reg1.odr = (uint8_t)val;
    ret = sensor_common_write_reg(LIS3DH_CTRL_REG1,(uint8_t *)&ctrl_reg1,1);
  }

  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of odr in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_data_rate_get(lis3dh_odr_t *val)
{
  lis3dh_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG1, (uint8_t *)&ctrl_reg1,1);
  SENSOR_DESELECT();

  switch (ctrl_reg1.odr) {
    case LIS3DH_POWER_DOWN:
      *val = LIS3DH_POWER_DOWN;
      break;

    case LIS3DH_ODR_1Hz:
      *val = LIS3DH_ODR_1Hz;
      break;

    case LIS3DH_ODR_10Hz:
      *val = LIS3DH_ODR_10Hz;
      break;

    case LIS3DH_ODR_25Hz:
      *val = LIS3DH_ODR_25Hz;
      break;

    case LIS3DH_ODR_50Hz:
      *val = LIS3DH_ODR_50Hz;
      break;

    case LIS3DH_ODR_100Hz:
      *val = LIS3DH_ODR_100Hz;
      break;

    case LIS3DH_ODR_200Hz:
      *val = LIS3DH_ODR_200Hz;
      break;

    case LIS3DH_ODR_400Hz:
      *val = LIS3DH_ODR_400Hz;
      break;

    case LIS3DH_ODR_1kHz620_LP:
      *val = LIS3DH_ODR_1kHz620_LP;
      break;

    case LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP:
      *val = LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP;
      break;

    default:
      *val = LIS3DH_POWER_DOWN;
      break;
  }

  return ret;
}

/**
  * @brief  Full-scale configuration.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fs in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_full_scale_set(lis3dh_fs_t val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);

  if (ret == 0) {
    ctrl_reg4.fs = (uint8_t)val;
    ret = sensor_common_write_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);
  }

  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  Full-scale configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      get the values of fs in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_full_scale_get(lis3dh_fs_t *val)
{
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);
  SENSOR_DESELECT();

  switch (ctrl_reg4.fs) {
    case LIS3DH_2g:
      *val = LIS3DH_2g;
      break;

    case LIS3DH_4g:
      *val = LIS3DH_4g;
      break;

    case LIS3DH_8g:
      *val = LIS3DH_8g;
      break;

    case LIS3DH_16g:
      *val = LIS3DH_16g;
      break;

    default:
      *val = LIS3DH_2g;
      break;
  }

  return ret;
}


/**
  * @brief  Operating mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpen in reg CTRL_REG1
  *                  and HR in reg CTRL_REG4
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_operating_mode_set(lis3dh_op_md_t val)
{
  lis3dh_ctrl_reg1_t ctrl_reg1;
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG1, (uint8_t *)&ctrl_reg1,1);

  if (ret == 0) {
    ret = sensor_common_read_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);
  }

  if (ret == 0) {
    if ( val == LIS3DH_HR_12bit ) {
      ctrl_reg1.lpen = 0;
      ctrl_reg4.hr   = 1;
    }

    if (val == LIS3DH_NM_10bit) {
      ctrl_reg1.lpen = 0;
      ctrl_reg4.hr   = 0;
    }

    if (val == LIS3DH_LP_8bit) {
      ctrl_reg1.lpen = 1;
      ctrl_reg4.hr   = 0;
    }

    ret = sensor_common_write_reg(LIS3DH_CTRL_REG1, (uint8_t *)&ctrl_reg1,1);
  }

  if (ret == 0) {
    ret = sensor_common_write_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);
  }

  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  Operating mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lpen in reg CTRL_REG1
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_operating_mode_get(lis3dh_op_md_t *val)
{
  lis3dh_ctrl_reg1_t ctrl_reg1;
  lis3dh_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG1, (uint8_t *)&ctrl_reg1,1);

  if (ret == 0) {
    ret = sensor_common_read_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);

    if ( ctrl_reg1.lpen == PROPERTY_ENABLE ) {
      *val = LIS3DH_LP_8bit;
    }

    else if (ctrl_reg4.hr == PROPERTY_ENABLE ) {
      *val = LIS3DH_HR_12bit;
    }

    else {
      *val = LIS3DH_NM_10bit;
    }
  }

  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fth in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_watermark_set(uint8_t val)
{
  lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg,1);

  if (ret == 0) {
    fifo_ctrl_reg.fth = val;
    ret = sensor_common_write_reg(LIS3DH_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg,1);

  }

  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  FIFO watermark level selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fth in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_watermark_get(uint8_t *val)
{
  lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg,1);
  SENSOR_DESELECT();

  *val = (uint8_t)fifo_ctrl_reg.fth;
  return ret;
}

/**
  * @brief  FIFO mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fm in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_mode_set(lis3dh_fm_t val)
{
  lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg,1);


  if (ret == 0) {
    fifo_ctrl_reg.fm = (uint8_t)val;
    ret = sensor_common_write_reg(LIS3DH_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg,1);
  }

  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  FIFO mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fm in reg FIFO_CTRL_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_mode_get(lis3dh_fm_t *val)
{
  lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_FIFO_CTRL_REG, (uint8_t *)&fifo_ctrl_reg,1);
  SENSOR_DESELECT();

  switch (fifo_ctrl_reg.fm) {
    case LIS3DH_BYPASS_MODE:
      *val = LIS3DH_BYPASS_MODE;
      break;

    case LIS3DH_FIFO_MODE:
      *val = LIS3DH_FIFO_MODE;
      break;

    case LIS3DH_DYNAMIC_STREAM_MODE:
      *val = LIS3DH_DYNAMIC_STREAM_MODE;
      break;

    case LIS3DH_STREAM_TO_FIFO_MODE:
      *val = LIS3DH_STREAM_TO_FIFO_MODE;
      break;

    default:
      *val = LIS3DH_BYPASS_MODE;
      break;
  }

  return ret;
}


/**
  * @brief  FIFO enable.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_en in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_set(uint8_t val)
{
  lis3dh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG5, (uint8_t *)&ctrl_reg5,1);

  if (ret == 0) {
    ctrl_reg5.fifo_en = val;
    ret = sensor_common_write_reg(LIS3DH_CTRL_REG5, (uint8_t *)&ctrl_reg5,1);
  }

  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  FIFO enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fifo_en in reg CTRL_REG5
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_get(uint8_t *val)
{
  lis3dh_ctrl_reg5_t ctrl_reg5;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG5, (uint8_t *)&ctrl_reg5,1);
  SENSOR_DESELECT();

  *val = (uint8_t)ctrl_reg5.fifo_en;
  return ret;
}


/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
  state = SENSOR_STATE_ENABLED;
  sensors_changed(&lis3dh_sensor);

  int32_t ret;
  uint8_t whoamI=1;

  /*  Check device ID */
  lis3dh_device_id_get(&whoamI);
  if(ret == -1) {
    PRINTF("LIS WHO ERROR\n");
  }
  else {
    PRINTF("LIS is: %02X\n",whoamI);
  }


  /*  Enable Block Data Update */
  ret = lis3dh_block_data_update_set(PROPERTY_ENABLE);
  if(ret == -1) {
    PRINTF("LIS BUD ERROR\n");
  }
  else {
    PRINTF("LIS IS WORKING\n");
  }
  /* Set Output Data Rate to 25 hz */
  lis3dh_data_rate_set(LIS3DH_ODR_25Hz);
  /* Set full scale to 2 g */
  lis3dh_full_scale_set(LIS3DH_2g);
  /* Set operating mode to high resolution */
  lis3dh_operating_mode_set(LIS3DH_HR_12bit);
  /* Set FIFO watermark to 25 samples */
  lis3dh_fifo_watermark_set(25);
  /* Set FIFO mode to Stream mode: Accumulate samples and
   * override old data */
  lis3dh_fifo_mode_set(LIS3DH_DYNAMIC_STREAM_MODE);
  /* Enable FIFO */
  lis3dh_fifo_set(PROPERTY_ENABLE);

  
}
/*---------------------------------------------------------------------------*/
static void
initialise(void *not_used)
{
  ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
}
/*---------------------------------------------------------------------------*/
static void
power_up(void)
{
  ti_lib_gpio_set_dio(BOARD_IOID_MPU_POWER);
  state = SENSOR_STATE_BOOTING;

  ctimer_set(&startup_timer, SENSOR_BOOT_DELAY, initialise, NULL);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type LIS3DH_SENSOR_TYPE_ACC_[XYZ] or LIS3DH_SENSOR_TYPE_GYRO_[XYZ]
 * \return centi-G (ACC) or centi-Deg/Sec (Gyro)
 */
static int
value(int type)
{
  int rv;
  float converted_val = 0;

  if(state == SENSOR_STATE_DISABLED) {
    PRINTF("LIS: Sensor Disabled\n");
    return CC26XX_SENSOR_READING_ERROR;
  }

  memset(sensor_value, 0, sizeof(sensor_value));

  // t0 = RTIMER_NOW();

  // while(!int_status() &&
  //       (RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + READING_WAIT_TIMEOUT)));

  rv = acc_read(sensor_value);

  if(rv == 0) {
    return CC26XX_SENSOR_READING_ERROR;
  }

  PRINTF("LIS: ACC = 0x%04x 0x%04x 0x%04x = ",
          sensor_value[0], sensor_value[1], sensor_value[2]);

  // /* Convert */
  // if(type == LIS3DH_SENSOR_TYPE_ACC_X) {
  //   converted_val = acc_convert(sensor_value[0]);
  // } else if(type == LIS3DH_SENSOR_TYPE_ACC_Y) {
  //   converted_val = acc_convert(sensor_value[1]);
  // } else if(type == LIS3DH_SENSOR_TYPE_ACC_Z) {
  //   converted_val = acc_convert(sensor_value[2]);
  // }
  // rv = (int)(converted_val * 100);

  // PRINTF("%ld\n", (long int)(converted_val * 100));

  return rv;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the MPU9250 sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
static int
configure(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:
    ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_MPU_INT);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_MPU_INT, IOC_NO_IOPULL);
    ti_lib_ioc_io_hyst_set(BOARD_IOID_MPU_INT, IOC_HYST_ENABLE);

    ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_MPU_POWER);
    ti_lib_ioc_io_drv_strength_set(BOARD_IOID_MPU_POWER, IOC_CURRENT_4MA,
                                   IOC_STRENGTH_MAX);
    ti_lib_gpio_set_dio(BOARD_IOID_MPU_POWER);
    break;
  case SENSORS_ACTIVE:
    if(enable) {
      PRINTF("LIS: Enabling2\n");
      power_up();
      delay_ms(10);

      /*  Check device ID */
      lis3dh_device_id_get(&whoamI);
      if(ret == -1) {
        PRINTF("LIS WHO ERROR\n");
      }
      else {
        PRINTF("LIS is: %02X\n",whoamI);
      }

      state = SENSOR_STATE_BOOTING;
    } else {
      // PRINTF("LIS: Disabling\n");
      // if(HWREG(GPIO_BASE + GPIO_O_DOUT31_0) & BOARD_MPU_POWER) {
      //   /* Then check our state */
      //   ctimer_stop(&startup_timer);
      //   // sensor_sleep();
      //   while(ti_lib_i2c_master_busy(I2C0_BASE));
      //   state = SENSOR_STATE_DISABLED;
      //   ti_lib_gpio_set_dio(BOARD_IOID_MPU_POWER);
      // }
    }
    break;
  default:
    break;
  }
  return state;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 if the sensor is enabled
 */
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return state;
    break;
  default:
    break;
  }
  return SENSOR_STATE_DISABLED;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(lis3dh_sensor, "LIS3DH", value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
