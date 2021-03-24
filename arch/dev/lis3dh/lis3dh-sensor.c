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

// /* Sensor selection/deselection */
// #define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_1, LIS3H_I2C_ADDRESS)
// #define SENSOR_DESELECT()   board_i2c_deselect()

/*---------------------------------------------------------------------------*/
#define LIS_DATA_READY    0x01
#define LIS_MOVEMENT      0x40
/*---------------------------------------------------------------------------*/
/* Sensor selection/deselection */
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_1, LIS3DH_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()
/*---------------------------------------------------------------------------*/
/* Delay */
#define delay_ms(i) (ti_lib_cpu_delay(8000 * (i)))
/*---------------------------------------------------------------------------*/
static uint8_t lis_config;
static uint8_t acc_range;
static uint8_t acc_range_reg;
static uint8_t val;
static uint8_t interrupt_status;
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

/*
 * Wait timeout in rtimer ticks. This is just a random low number, since the
 * first time we read the sensor status, it should be ready to return data
 */
#define READING_WAIT_TIMEOUT 10
/*---------------------------------------------------------------------------*/
// /**
//  * \brief Place the MPU in low power mode
//  */
// static void
// sensor_sleep(void)
// {
//   SENSOR_SELECT();

//   val = LIS_SLEEP;
//   sensor_common_write_reg(PWR_MGMT_1, &val, 1);
//   SENSOR_DESELECT();
// }
// /*---------------------------------------------------------------------------*/
// /**
//  * \brief Exit low power mode
//  */
// static void
// sensor_wakeup(void)
// {
//   SENSOR_SELECT();
//   val = LIS_WAKE_UP;
//   sensor_common_write_reg(PWR_MGMT_1, &val, 1);

//   /* All axis initially disabled */
//   val = ALL_AXES;
//   sensor_common_write_reg(PWR_MGMT_2, &val, 1);
//   lis_config = 0;

//   /* Restore the range */
//   sensor_common_write_reg(ACCEL_CONFIG, &acc_range_reg, 1);

//   /* Clear interrupts */
//   sensor_common_read_reg(INT_STATUS, &val, 1);
//   SENSOR_DESELECT();
// }
// /*---------------------------------------------------------------------------*/
// /**
//  * \brief Select gyro and accelerometer axes
//  */
// static void
// select_axes(void)
// {
//   val = ~lis_config;
//   SENSOR_SELECT();
//   sensor_common_write_reg(PWR_MGMT_2, &val, 1);
//   SENSOR_DESELECT();
// }
// /*---------------------------------------------------------------------------*/
// static void
// convert_to_le(uint8_t *data, uint8_t len)
// {
//   int i;
//   for(i = 0; i < len; i += 2) {
//     uint8_t tmp;
//     tmp = data[i];
//     data[i] = data[i + 1];
//     data[i + 1] = tmp;
//   }
// }
// /*---------------------------------------------------------------------------*/
// /**
//  * \brief Set the range of the accelerometer
//  * \param new_range: ACC_RANGE_2G, ACC_RANGE_4G, ACC_RANGE_8G, ACC_RANGE_16G
//  * \return true if the write to the sensor succeeded
//  */
// static bool
// acc_set_range(uint8_t new_range)
// {
//   bool success;

//   if(new_range == acc_range) {
//     return true;
//   }

//   success = false;

//   acc_range_reg = (new_range << 3);

//   /* Apply the range */
//   SENSOR_SELECT();
//   success = sensor_common_write_reg(ACCEL_CONFIG, &acc_range_reg, 1);
//   SENSOR_DESELECT();

//   if(success) {
//     acc_range = new_range;
//   }

//   return success;
// }
// /*---------------------------------------------------------------------------*/
// /**
//  * \brief Check whether a data or wake on motion interrupt has occurred
//  * \return Return the interrupt status
//  *
//  * This driver does not use interrupts, however this function allows us to
//  * determine whether a new sensor reading is available
//  */
// static uint8_t
// int_status(void)
// {
//   SENSOR_SELECT();
//   sensor_common_read_reg(INT_STATUS, &interrupt_status, 1);
//   SENSOR_DESELECT();

//   return interrupt_status;
// }
/*---------------------------------------------------------------------------*/
/**
 * \brief Enable the MPU
 * \param axes: Gyro bitmap [0..2], X = 1, Y = 2, Z = 4. 0 = gyro off
 *              Acc  bitmap [3..5], X = 8, Y = 16, Z = 32. 0 = accelerometer off
 */
static void
enable_sensor(uint16_t axes)
{
  // if(lis_config == 0 && axes != 0) {
  //   /* Wake up the sensor if it was off */
  //   sensor_wakeup();
  // }

  // lis_config = axes;

  // if(lis_config != 0) {
  //   /* Enable gyro + accelerometer readout */
  //   select_axes();
  //   delay_ms(10);
  // } else if(lis_config == 0) {
  //   sensor_sleep();
  // }
}
/*---------------------------------------------------------------------------*/
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
// /**
//  * \brief Read data from the gyroscope - X, Y, Z - 3 words
//  * \return True if a valid reading could be taken, false otherwise
//  */
// static bool
// gyro_read(uint16_t *data)
// {
//   bool success;

//   if(interrupt_status & BIT_RAW_RDY_EN) {
//     /* Select this sensor */
//     SENSOR_SELECT();

//     /* Burst read of all gyroscope values */
//     success = sensor_common_read_reg(GYRO_XOUT_H, (uint8_t *)data, DATA_SIZE);

//     if(success) {
//       convert_to_le((uint8_t *)data, DATA_SIZE);
//     } else {
//       sensor_common_set_error_data((uint8_t *)data, DATA_SIZE);
//     }

//     SENSOR_DESELECT();
//   } else {
//     success = false;
//   }

//   return success;
// }
// /*---------------------------------------------------------------------------*/
// /**
//  * \brief Convert accelerometer raw reading to a value in G
//  * \param raw_data The raw accelerometer reading
//  * \return The converted value
//  */
// static float
// acc_convert(int16_t raw_data)
// {
//   float v = 0;

//   switch(acc_range) {
//   case ACC_RANGE_2G:
//     /* Calculate acceleration, unit G, range -2, +2 */
//     v = (raw_data * 1.0) / (32768 / 2);
//     break;
//   case ACC_RANGE_4G:
//     /* Calculate acceleration, unit G, range -4, +4 */
//     v = (raw_data * 1.0) / (32768 / 4);
//     break;
//   case ACC_RANGE_8G:
//     /* Calculate acceleration, unit G, range -8, +8 */
//     v = (raw_data * 1.0) / (32768 / 8);
//     break;
//   case ACC_RANGE_16G:
//     /* Calculate acceleration, unit G, range -16, +16 */
//     v = (raw_data * 1.0) / (32768 / 16);
//     break;
//   default:
//     v = 0;
//     break;
//   }

//   return v;
// }
/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
  state = SENSOR_STATE_ENABLED;
  sensors_changed(&lis3dh_sensor);
}
/*---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------*/
static void
initialise(void *not_used)
{
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
  // /* Set Output Data Rate to 25 hz */
  // lis3dh_data_rate_set(LIS3DH_ODR_25Hz);
  // /* Set full scale to 2 g */
  // lis3dh_full_scale_set(LIS3DH_2g);
  // /* Set operating mode to high resolution */
  // lis3dh_operating_mode_set(LIS3DH_HR_12bit);
  // /* Set FIFO watermark to 25 samples */
  // lis3dh_fifo_watermark_set(25);
  // /* Set FIFO mode to Stream mode: Accumulate samples and
  //  * override old data */
  // lis3dh_fifo_mode_set(LIS3DH_DYNAMIC_STREAM_MODE);
  // /* Enable FIFO */
  // lis3dh_fifo_set(PROPERTY_ENABLE);

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

  PRINTF("MPU: ACC = 0x%04x 0x%04x 0x%04x = ",
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
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_MPU_INT, IOC_IOPULL_DOWN);
    ti_lib_ioc_io_hyst_set(BOARD_IOID_MPU_INT, IOC_HYST_ENABLE);

    ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_MPU_POWER);
    ti_lib_ioc_io_drv_strength_set(BOARD_IOID_MPU_POWER, IOC_CURRENT_4MA,
                                   IOC_STRENGTH_MAX);
    ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);
    break;
  case SENSORS_ACTIVE:
    if( enable != 0 ) {
      PRINTF("LIS: Enabling\n");
      power_up();

      state = SENSOR_STATE_BOOTING;
    } else {
      PRINTF("LIS: Disabling\n");
      if(HWREG(GPIO_BASE + GPIO_O_DOUT31_0) & BOARD_MPU_POWER) {
        /* Then check our state */
        ctimer_stop(&startup_timer);
        // sensor_sleep();
        while(ti_lib_i2c_master_busy(I2C0_BASE));
        state = SENSOR_STATE_DISABLED;
        ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);
      }
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
