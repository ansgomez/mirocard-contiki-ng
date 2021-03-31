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
/* 3 16-byte words for all sensor readings */
#define SENSOR_DATA_BUF_SIZE   3

int16_t lis_buff[3];
static int16_t sensor_value[SENSOR_DATA_BUF_SIZE];
// static float acceleration_mg[SENSOR_DATA_BUF_SIZE];
/*---------------------------------------------------------------------------*/
static int state = SENSOR_STATE_DISABLED;
static int config = LIS_SENSOR_TYPE_FIFO;

/*---------------------------------------------------------------------------*/
/* Wait for the MPU to have data ready */
rtimer_clock_t t0;

uint8_t whoamI=0;
uint8_t i2c_buff[6];
uint16_t count=0;

/*
 * Wait timeout in rtimer ticks. This is just a random low number, since the
 * first time we read the sensor status, it should be ready to return data
 */
#define READING_WAIT_TIMEOUT 10
static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
int32_t lis3dh_get(uint8_t *buff, uint8_t len){
  int32_t ret;
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
  int32_t ret;
  lis3dh_ctrl_reg4_t ctrl_reg4;

  /* Burst read of all accelerometer values */
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);


  if(ret != -1) {
    ctrl_reg4.bdu = val;
    ret = sensor_common_write_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);
  } 
  
    SENSOR_DESELECT();

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

  if (ret != -1) {
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

  if (ret != -1) {
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

  if (ret != -1) {
    ret = sensor_common_read_reg(LIS3DH_CTRL_REG4, (uint8_t *)&ctrl_reg4,1);
  }

  if (ret != -1) {
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

  if (ret != -1) {
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

  if (ret != -1) {
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

  if (ret != -1) {
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


  if (ret != -1) {
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

  if (ret != -1) {
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

/**
  * @brief  FIFO status register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_status_get(lis3dh_fifo_src_reg_t *val)
{
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_FIFO_SRC_REG, (uint8_t *)val,1);
  SENSOR_DESELECT();
  return ret;
}
/**
  * @brief  FIFO stored data level.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fss in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_data_level_get(uint8_t *val)
{
  lis3dh_fifo_src_reg_t fifo_src_reg;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg,1);
  SENSOR_DESELECT();
  *val = (uint8_t)fifo_src_reg.fss;
  return ret;
}
/**
  * @brief  Empty FIFO status flag.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of empty in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_empty_flag_get(uint8_t *val)
{
  lis3dh_fifo_src_reg_t fifo_src_reg;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg,1);
  SENSOR_DESELECT();
  *val = (uint8_t)fifo_src_reg.empty;
  return ret;
}
/**
  * @brief  FIFO overrun status flag.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ovrn_fifo in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_ovr_flag_get(uint8_t *val)
{
  lis3dh_fifo_src_reg_t fifo_src_reg;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg,1);
  SENSOR_DESELECT();

  if(ret != -1) {
    *val = (uint8_t)fifo_src_reg.ovrn_fifo;
  }
  return ret;
}
/**
  * @brief  FIFO watermark status.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wtm in reg FIFO_SRC_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_fifo_fth_flag_get(uint8_t *val)
{
  lis3dh_fifo_src_reg_t fifo_src_reg;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg,1);
  SENSOR_DESELECT();

  if(ret != -1) {
    *val = (uint8_t)fifo_src_reg.wtm;
  }
  return ret;
}

/**
  * @brief  Acceleration set of data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of zyxda in reg STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_xl_data_ready_get(uint8_t *val)
{
  lis3dh_status_reg_t status_reg;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_STATUS_REG, (uint8_t *)&status_reg,1);
  SENSOR_DESELECT();

  if(ret != -1) {
    *val = status_reg.zyxda;
  }
  
  return ret;
}
/**
  * @brief  Acceleration set of data overrun.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of zyxor in reg STATUS_REG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_xl_data_ovr_get( uint8_t *val)
{
  lis3dh_status_reg_t status_reg;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_STATUS_REG, (uint8_t *)&status_reg,1);
  SENSOR_DESELECT();

  if(ret != -1) {
    *val = status_reg.zyxor;
  }

  return ret;
}

/**
  * @brief  User-defined threshold value for Tap/Double Tap event.[set]
  *         1 LSB = full scale/128
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_threshold_set(uint8_t val)
{
  lis3dh_click_ths_t click_ths;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CLICK_THS, (uint8_t *)&click_ths,1);

  if (ret == 0) {
    click_ths.ths = val;
    click_ths.lir_click = 0x01;
    ret = sensor_common_write_reg(LIS3DH_CLICK_THS, (uint8_t *)&click_ths,1);
  }
  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  User-defined threshold value for Tap/Double Tap event.[get]
  *         1 LSB = full scale/128
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_threshold_get(uint8_t *val)
{
  lis3dh_click_ths_t click_ths;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_CLICK_THS, (uint8_t *)&click_ths,1);
  SENSOR_DESELECT();
  *val = (uint8_t)click_ths.ths;
  return ret;
}

/**
  * @brief   If the LIR_Click bit is not set, the interrupt is kept high
  *          for the duration of the latency window.
  *          If the LIR_Click bit is set, the interrupt is kept high until the
  *          CLICK_SRC(39h) register is read.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of lir_click in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_notification_mode_set(stmdev_ctx_t *ctx,
                                         lis3dh_lir_click_t val)
{
  lis3dh_click_ths_t click_ths;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CLICK_THS, (uint8_t *)&click_ths,1);

  if (ret == 0) {
    click_ths.lir_click = (uint8_t)val;
    ret = sensor_common_write_reg(LIS3DH_CLICK_THS, (uint8_t *)&click_ths,1);
  }
  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief   If the LIR_Click bit is not set, the interrupt is kept high
  *          for the duration of the latency window.
  *          If the LIR_Click bit is set, the interrupt is kept high until the
  *          CLICK_SRC(39h) register is read.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of lir_click in reg CLICK_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_notification_mode_get(stmdev_ctx_t *ctx,
                                         lis3dh_lir_click_t *val)
{
  lis3dh_click_ths_t click_ths;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_CLICK_THS, (uint8_t *)&click_ths,1);
  SENSOR_DESELECT();

  switch (click_ths.lir_click) {
    case LIS3DH_TAP_PULSED:
      *val = LIS3DH_TAP_PULSED;
      break;

    case LIS3DH_TAP_LATCHED:
      *val = LIS3DH_TAP_LATCHED;
      break;

    default:
      *val = LIS3DH_TAP_PULSED;
      break;
  }

  return ret;
}

/**
  * @brief  The maximum time (1 LSB = 1/ODR) interval that can elapse
  *         between the start of the click-detection procedure and when the
  *         acceleration falls back below the threshold.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tli in reg TIME_LIMIT
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_shock_dur_set(uint8_t val)
{
  lis3dh_time_limit_t time_limit;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_TIME_LIMIT, (uint8_t *)&time_limit,1);

  if (ret == 0) {
    time_limit.tli = val;
    ret = sensor_common_write_reg(LIS3DH_TIME_LIMIT, (uint8_t *)&time_limit,1);
  }
  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  The maximum time (1 LSB = 1/ODR) interval that can elapse between
  *         the start of the click-detection procedure and when the
  *         acceleration falls back below the threshold.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tli in reg TIME_LIMIT
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_shock_dur_get(uint8_t *val)
{
  lis3dh_time_limit_t time_limit;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_TIME_LIMIT, (uint8_t *)&time_limit,1);
  SENSOR_DESELECT();
  *val = (uint8_t)time_limit.tli;
  return ret;
}

/**
  * @brief  The time (1 LSB = 1/ODR) interval that starts after the first
  *         click detection where the click-detection procedure is
  *         disabled, in cases where the device is configured for
  *         double-click detection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tla in reg TIME_LATENCY
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_quiet_dur_set(uint8_t val)
{
  lis3dh_time_latency_t time_latency;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_TIME_LATENCY, (uint8_t *)&time_latency,1);

  if (ret == 0) {
    time_latency.tla = val;
    ret = sensor_common_write_reg(LIS3DH_TIME_LATENCY, (uint8_t *)&time_latency,1);
  }
  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  The time (1 LSB = 1/ODR) interval that starts after the first
  *         click detection where the click-detection procedure is
  *         disabled, in cases where the device is configured for
  *         double-click detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tla in reg TIME_LATENCY
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_quiet_dur_get(uint8_t *val)
{
  lis3dh_time_latency_t time_latency;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_TIME_LATENCY, (uint8_t *)&time_latency,1);
  SENSOR_DESELECT();
  *val = (uint8_t)time_latency.tla;
  return ret;
}

/**
  * @brief  The maximum interval of time (1 LSB = 1/ODR) that can elapse
  *         after the end of the latency interval in which the click-detection
  *         procedure can start, in cases where the device is configured
  *         for double-click detection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tw in reg TIME_WINDOW
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_double_tap_timeout_set(uint8_t val)
{
  lis3dh_time_window_t time_window;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_TIME_WINDOW, (uint8_t *)&time_window,1);

  if (ret == 0) {
    time_window.tw = val;
    ret = sensor_common_write_reg(LIS3DH_TIME_WINDOW, (uint8_t *)&time_window,1);
  }
  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  The maximum interval of time (1 LSB = 1/ODR) that can elapse
  *         after the end of the latency interval in which the
  *         click-detection procedure can start, in cases where the device
  *         is configured for double-click detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tw in reg TIME_WINDOW
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_double_tap_timeout_get(uint8_t *val)
{
  lis3dh_time_window_t time_window;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_TIME_WINDOW, (uint8_t *)&time_window,1);
  SENSOR_DESELECT();
  *val = (uint8_t)time_window.tw;
  return ret;
}


/**
  * @brief  Int1 pin routing configuration register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CTRL_REG3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_pin_int1_config_set(lis3dh_ctrl_reg3_t *val)
{
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_write_reg( LIS3DH_CTRL_REG3, (uint8_t *)val,1);
  SENSOR_DESELECT();
  return ret;
}

/**
  * @brief  Int1 pin routing configuration register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CTRL_REG3
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_pin_int1_config_get(lis3dh_ctrl_reg3_t *val)
{
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_CTRL_REG3, (uint8_t *)val,1);
  SENSOR_DESELECT();
  return ret;
}


/**
  * @brief  Tap/Double Tap generator configuration register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CLICK_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_conf_set(lis3dh_click_cfg_t *val)
{
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_write_reg( LIS3DH_CLICK_CFG, (uint8_t *)val,1);
  SENSOR_DESELECT();
  return ret;
}

/**
  * @brief  Tap/Double Tap generator configuration register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CLICK_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_conf_get(lis3dh_click_cfg_t *val)
{
  int32_t ret; 
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_CLICK_CFG, (uint8_t *)val,1);
  SENSOR_DESELECT();
  return ret;
}


/**
  * @brief  Interrupt generator 1 configuration register.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register INT1_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_conf_set(lis3dh_int1_cfg_t *val)
{
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_write_reg( LIS3DH_INT1_CFG, (uint8_t *)val,1);
  SENSOR_DESELECT();
  return ret;
}

/**
  * @brief  Interrupt generator 1 configuration register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      register INT1_CFG
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_conf_get(lis3dh_int1_cfg_t *val)
{
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_INT1_CFG, (uint8_t *)val,1);
  SENSOR_DESELECT();
  return ret;
}

/**
  * @brief  Interrupt generator 1 source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Registers INT1_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_source_get(lis3dh_int1_src_t *val)
{
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_INT1_SRC, (uint8_t *)val,1);
  SENSOR_DESELECT();
  return ret;
}
/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 1.[set]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg INT1_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_threshold_set(uint8_t val)
{
  lis3dh_int1_ths_t int1_ths;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_INT1_THS, (uint8_t *)&int1_ths,1);

  if (ret == 0) {
    int1_ths.ths = val;
    ret = sensor_common_write_reg(LIS3DH_INT1_THS, (uint8_t *)&int1_ths,1);
  }
  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on
  *         generator 1.[get]
  *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of ths in reg INT1_THS
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_threshold_get(uint8_t *val)
{
  lis3dh_int1_ths_t int1_ths;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_INT1_THS, (uint8_t *)&int1_ths,1);
  SENSOR_DESELECT();
  *val = (uint8_t)int1_ths.ths;
  return ret;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d in reg INT1_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_duration_set(uint8_t val)
{
  lis3dh_int1_duration_t int1_duration;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_INT1_DURATION, (uint8_t *)&int1_duration,1);

  if (ret == 0) {
    int1_duration.d = val;
    ret = sensor_common_write_reg(LIS3DH_INT1_DURATION, (uint8_t *)&int1_duration,1);
  }
  SENSOR_DESELECT();

  return ret;
}

/**
  * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
  *         recognized.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of d in reg INT1_DURATION
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_int1_gen_duration_get(uint8_t *val)
{
  lis3dh_int1_duration_t int1_duration;
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_INT1_DURATION, (uint8_t *)&int1_duration,1);
  SENSOR_DESELECT();
  *val = (uint8_t)int1_duration.d;
  return ret;
}

/**
  * @brief  Tap/Double Tap generator source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CLICK_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_source_get(lis3dh_click_src_t *val)
{
  int32_t ret;
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_CLICK_SRC, (uint8_t *)val,1);
  SENSOR_DESELECT();
  return ret;
}

/**
  * @brief  Tap/Double Tap generator source register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      registers CLICK_SRC
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_tap_enable()
{
  lis3dh_click_src_t src;
  int32_t ret;

  SENSOR_SELECT();
  ret = sensor_common_read_reg(LIS3DH_CLICK_SRC, (uint8_t *)&src,1);

  if (ret == 0) {
    src.sclick = 0x01;
    ret = sensor_common_write_reg(LIS3DH_CLICK_SRC, (uint8_t *)&src,1);
  }
  SENSOR_DESELECT();

  return ret;
}


/**
  * @brief  Acceleration output value.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis3dh_acceleration_raw_get(int16_t *val)
{
  uint8_t buff[6]={0};
  int32_t ret=1;
  bool success;  
  
  SENSOR_SELECT();
  for(int i=0;i<6;i++) {
    success = sensor_common_read_reg( LIS3DH_OUT_X_L+i*(sizeof(uint8_t)), (uint8_t *)(buff+i*(sizeof(uint8_t))), 1);
  }
  // TODO: Change loop to multi-byte read.
  // success = sensor_common_read_reg_increment( LIS3DH_OUT_X_L, (uint8_t) 6, (uint8_t *)buff, 6);
  // success = sensor_common_read_reg( LIS3DH_OUT_Y_L, (uint8_t *)buff+2, 2);
  // success = sensor_common_read_reg( LIS3DH_OUT_Z_L, (uint8_t *)buff+4, 2);
  // board_i2c_write_single(LIS3DH_OUT_X_L);
  // board_i2c_read((uint8_t *)buff, 6);

  SENSOR_DESELECT();

  if(success) {
    val[0] = (int16_t)buff[1];
    val[0] = (val[0] * 256) +  (int16_t)buff[0];
    val[1] = (int16_t)buff[3];
    val[1] = (val[1] * 256) +  (int16_t)buff[2];
    val[2] = (int16_t)buff[5];
    val[2] = (val[2] * 256) +  (int16_t)buff[4];

#ifdef DEBUG_REGISTERS
    printf("RAW values: :");
    for(int i=0;i<6;i++) {
      // PRINTF("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
      //       acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      PRINTF("%d,",buff[i]);
    }
    PRINTF("\n");
#endif
  } else {
    PRINTF("Failed to get raw data\n");
  }
  return ret;
}

void lis_config_fifo_mode() {
  int32_t ret;

 /*  Enable Block Data Update */
  ret = lis3dh_block_data_update_set(PROPERTY_ENABLE);
  PRINTF((ret != -1)?"":"LIS BUD ERROR\n");
  // if(ret == -1) {
  //   PRINTF("LIS BUD ERROR\n");
  // }
  // else {
  //   PRINTF("LIS IS WORKING\n");
  // }
  /* Set Output Data Rate to 25 hz */
  ret = lis3dh_data_rate_set(LIS3DH_ODR_10Hz);
  PRINTF((ret!= -1)?"":"LIS BUD ERROR\n");
  /* Set full scale to 2 g */
  ret = lis3dh_full_scale_set(LIS3DH_2g);
  PRINTF((ret!= -1)?"":"LIS SCALE ERROR\n");
  /* Set operating mode to high resolution */
  ret = lis3dh_operating_mode_set(LIS3DH_NM_10bit);
  PRINTF((ret!= -1)?"":"LIS BIT ERROR\n");
  /* Set FIFO watermark to 25 samples */
  ret = lis3dh_fifo_watermark_set(2);
  PRINTF((ret!= -1)?"":"LIS WTM ERROR\n");
  /* Set FIFO mode to Stream mode: Accumulate samples and
   * override old data */
  // ret = lis3dh_fifo_mode_set(LIS3DH_BYPASS_MODE);
  // PRINTF((ret!= -1)?"":"LIS MODE ERROR1\n");
  // ret = lis3dh_fifo_mode_set(LIS3DH_DYNAMIC_STREAM_MODE);
  // PRINTF((ret!= -1)?"":"LIS MODE ERROR2\n");
  //  ret = lis3dh_fifo_mode_set(LIS3DH_BYPASS_MODE);
  // PRINTF((ret!= -1)?"":"LIS MODE ERROR1\n"); 
  ret = lis3dh_fifo_mode_set(LIS3DH_DYNAMIC_STREAM_MODE);
  PRINTF((ret!= -1)?"":"LIS MODE ERROR2\n");

  /* Enable FIFO */
  lis3dh_fifo_set(PROPERTY_ENABLE);
  PRINTF((ret!= -1)?"":"LIS FIFO ERROR\n");
}

void lis_config_tap_mode() {
  lis3dh_ctrl_reg3_t ctrl_reg3;
  lis3dh_click_cfg_t click_cfg;
  int32_t ret;

  /* Set Output Data Rate
   * The recommended accelerometer ODR for single and
   * double-click recognition is 400 Hz or higher */
  ret = lis3dh_data_rate_set(LIS3DH_ODR_400Hz);
  PRINTF((ret!= -1)?"":"LIS RATE ERROR1\n");
  /* Set full scale to 2 g */
  ret = lis3dh_full_scale_set(LIS3DH_2g);
  PRINTF((ret!= -1)?"":"LIS SCALE ERROR1\n");
  /* Set click threshold to 12h  -> 0.281 g
   * 1 LSB = full scale/128
   * Set TIME_LIMIT to 33h -> 127 ms.
   * 1 LSB = 1/ODR */
  ret = lis3dh_tap_threshold_set(0x24); //0x12
  PRINTF((ret!= -1)?"":"LIS THS ERROR\n");
  ret = lis3dh_shock_dur_set(0x44);
  PRINTF((ret!= -1)?"":"LIS SDUR ERROR\n");
  ret = lis3dh_quiet_dur_set(0xFF);
  PRINTF((ret!= -1)?"":"LIS QDUR ERROR\n");
  /* Enable Click interrupt on INT pin 1 */
  ret = lis3dh_pin_int1_config_get(&ctrl_reg3);
  PRINTF((ret!= -1)?"":"LIS INT1G ERROR\n");
  ctrl_reg3.i1_click = PROPERTY_ENABLE;
  ret = lis3dh_pin_int1_config_set(&ctrl_reg3);
  PRINTF((ret!= -1)?"":"LIS INT1S ERROR\n");
  ret = lis3dh_int1_gen_duration_set(0);
  PRINTF((ret!= -1)?"":"LIS INT1DUR ERROR\n");
  /* Enable single click on all axis */
  ret = lis3dh_tap_conf_get(&click_cfg);
  PRINTF((ret!= -1)?"":"LIS TAPG ERROR\n");
  click_cfg.xs = PROPERTY_ENABLE;
  click_cfg.ys = PROPERTY_ENABLE;
  click_cfg.zs = PROPERTY_ENABLE;
  ret = lis3dh_tap_conf_set(&click_cfg);
  PRINTF((ret!= -1)?"":"LIS TAPS ERROR\n");
  /* Set device in HR mode */
  ret = lis3dh_operating_mode_set(LIS3DH_NM_10bit);  
  PRINTF((ret!= -1)?"":"LIS MODE ERROR\n");
  ret = lis3dh_tap_enable();
  PRINTF((ret!= -1)?"":"LIS TAPE ERROR\n");
}

void lis_config_single_mode() {
  int32_t ret;

  /* Set Output Data Rate to 1Hz. */
  ret = lis3dh_data_rate_set( LIS3DH_ODR_1Hz);
  PRINTF((ret!= -1)?"":"LIS RATE ERROR1\n");
  /* Enable Block Data Update. */
  ret = lis3dh_block_data_update_set(PROPERTY_ENABLE);
  PRINTF((ret!= -1)?"":"LIS BUD ERROR1\n");
  /* Set full scale to 2g. */
  ret = lis3dh_full_scale_set(LIS3DH_2g);
  PRINTF((ret!= -1)?"":"LIS SCALW ERROR1\n");
  ret = lis3dh_fifo_mode_set(LIS3DH_BYPASS_MODE);
  PRINTF((ret!= -1)?"":"LIS MODE ERROR1\n"); 
  /* Set device in continuous mode with 12 bit resol. */
  ret = lis3dh_operating_mode_set(LIS3DH_NM_10bit);
  PRINTF((ret!= -1)?"":"LIS RES ERROR1\n");
  
}

/*---------------------------------------------------------------------------*/
static void
notify_ready(void *not_used)
{
  state = SENSOR_STATE_ENABLED;
  sensors_changed(&lis3dh_sensor);

  switch(config) {
    case LIS_SENSOR_TYPE_FIFO:
      lis_config_fifo_mode();
      break;
    case LIS_SENSOR_TYPE_SINGLE:
      lis_config_single_mode();
      break;
    case LIS_SENSOR_TYPE_TAP:
      lis_config_tap_mode();
    default:
      lis_config_fifo_mode();
      break;
  }

  PRINTF("LIS has been configured with type: %d\n", config);
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

void print_register(uint8_t addr) {
  uint8_t reg;
  // board_i2c_select(BOARD_I2C_INTERFACE_1, LIS3DH_I2C_ADDRESS);
  // ret = sensor_common_read_reg(LIS3DH_WHO_AM_I, buff, 1);
  board_i2c_select(BOARD_I2C_INTERFACE_1, LIS3DH_I2C_ADDRESS);
  sensor_common_read_reg(addr, &reg, 1);
  board_i2c_deselect();

  PRINTF("REG(%02X) = %02X\n",addr, reg);
}

static uint8_t
int_status(void)
{
  int32_t ret;
  lis3dh_click_src_t src;
  
  SENSOR_SELECT();
  ret = sensor_common_read_reg( LIS3DH_CLICK_SRC, (uint8_t *)&src,1);
  SENSOR_DESELECT();

  return src.sclick;
}

static bool acc_read_fifo () {
  bool success=false;
  int32_t ret;
  uint8_t flags;
  uint8_t num = 0;
  lis3dh_reg_t reg;   
  bool ready = false;

  /* Read output only if new value available */
  lis3dh_xl_data_ready_get( &reg.byte);

  // lis3dh_fifo_fth_flag_get(&num);
  // printf("FIFO threshold %u\n", num);

  // /* Check if FIFO level over threshold */
  // lis3dh_fifo_fth_flag_get( &flags);
  // printf("FIFO watermark %u\n", flags);

  /* Read number of sample in FIFO */
  // lis3dhh_fifo_full_flag_get( &num);
  lis3dh_fifo_data_level_get( &num );
  //printf("FIFO size %u\n", num);

  while (num-- > 0) {
    // memset(lis_buff, 0x00, 3 * sizeof(int16_t));
    // memset(acceleration_mg, 0x00, 3 * sizeof(int16_t));
    /* Read XL samples */
    ret=lis3dh_acceleration_raw_get( lis_buff);

    if(ret == 0) {
      success=false;
      break;
    }
    else{
      success=true;
    }

    // delay_ms(10);
    PRINTF("[%02X,",count);
    for(int i=0;i<3;i++) {
      // PRINTF("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
      //       acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      PRINTF("%04x,",lis_buff[i]);
    }
    PRINTF("]\n");

    count++;
  }

  return success;
}

static bool acc_read_single () {
  bool success;
  int32_t ret;

    // memset(lis_buff, 0x00, 3 * sizeof(int16_t));
  // memset(acceleration_mg, 0x00, 3 * sizeof(int16_t));
  /* Read XL samples */
  ret=lis3dh_acceleration_raw_get( lis_buff);

  if(ret == 0) {
    success=false;
  }
  else{
    success=true;
  }

  return success;
}

static bool acc_read_tap () {
  bool success;
  int32_t ret;
  lis3dh_click_src_t src;

  /*
  * Read INT pin 1 in polling mode
  * or read src status register
  */
  rtimer_clock_t   t0 = RTIMER_NOW();

  while(!int_status() &&
        (RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + READING_WAIT_TIMEOUT)));


  ret = lis3dh_tap_source_get(&src);
  if (src.sclick) {
    PRINTF("click detected : x %d, y %d, z %d, sign %d\n",src.x, src.y, src.z, src.sign);
  }

  if(ret==0) {
    success = false;
  }
  else {
    success = true;
  }

  return success;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Read data from the accelerometer - X, Y, Z - 3 words
 * \return True if a valid reading could be taken, false otherwise
 */
static bool
acc_read(int16_t *data)
{
  bool success=false;

#ifdef DEBUG_REGISTERS
  print_register(LIS3DH_CTRL_REG0);
  print_register(LIS3DH_CTRL_REG1);
  print_register(LIS3DH_CTRL_REG2);
  print_register(LIS3DH_CTRL_REG3);
  print_register(LIS3DH_CTRL_REG4);
  print_register(LIS3DH_CTRL_REG5);
  print_register(LIS3DH_CTRL_REG6);
  print_register(LIS3DH_STATUS_REG);
  print_register(LIS3DH_FIFO_CTRL_REG);
  print_register(LIS3DH_OUT_X_L);
  print_register(LIS3DH_OUT_X_H);
  print_register(LIS3DH_OUT_Y_L);
  print_register(LIS3DH_OUT_Y_H);
  print_register(LIS3DH_OUT_Z_L);
  print_register(LIS3DH_OUT_Z_H);
#endif 

  if(config == LIS_SENSOR_TYPE_FIFO) {
    success = acc_read_fifo();
  } else if (config == LIS_SENSOR_TYPE_SINGLE) {
    success = acc_read_single();
  } else if (config == LIS_SENSOR_TYPE_TAP) {
    success = acc_read_tap();
  } else {
    ("Unrecognized sensor type ACC_READ\n");
  }

  success=true;

  return success; 
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
    lis3dh_reg_t reg;

  if(state == SENSOR_STATE_DISABLED) {
    PRINTF("LIS: Sensor Disabled\n");
    return CC26XX_SENSOR_READING_ERROR;
  }

  /* Read output only if new value available */
  // lis3dh_xl_data_ready_get( &reg.byte);

  // if (reg.byte) {
  //   PRINTF("DATA ready\n");
  // } else {
  //   PRINTF("DATA not ready");
  // }

  rv = acc_read(sensor_value);

  return 1;
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
  int32_t ret;
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
    config = enable;

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
      PRINTF("LIS: Disabling\n");
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
