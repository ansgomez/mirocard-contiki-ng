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
#ifndef LIS3DH_SENSOR_H_
#define LIS3DH_SENSOR_H_
/*---------------------------------------------------------------------------*/
#include "lib/sensors.h"
/*---------------------------------------------------------------------------*/
/* Values PWR_MGMT_1 */
#define MPU_SLEEP                     0x4F  /* Sleep + stop all clocks */
#define MPU_WAKE_UP                   0x09  /* Disable temp. + intern osc */

/* 3 16-byte words for all sensor readings */
#define SENSOR_DATA_BUF_SIZE   3
/* Data sizes */
#define DATA_SIZE              6

/*
 * Wait SENSOR_BOOT_DELAY ticks for the sensor to boot and
 * SENSOR_STARTUP_DELAY for readings to be ready
 * Gyro is a little slower than Acc
 */
#define SENSOR_BOOT_DELAY     10
#define SENSOR_STARTUP_DELAY  10

/*---------------------------------------------------------------------------*/
#define LIS_SENSOR_TYPE_OFF         0
#define LIS_SENSOR_TYPE_FIFO        1
#define LIS_SENSOR_TYPE_TAP         2
#define LIS_SENSOR_TYPE_SINGLE      3

#define SENSOR_STATE_DISABLED     0
#define SENSOR_STATE_BOOTING      1
#define SENSOR_STATE_ENABLED      2

/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor lis3dh_sensor;
/*---------------------------------------------------------------------------*/



#ifndef DRV_BYTE_ORDER
#ifndef __BYTE_ORDER__

#define DRV_LITTLE_ENDIAN 1234
#define DRV_BIG_ENDIAN    4321

/** if _BYTE_ORDER is not defined, choose the endianness of your architecture
  * by uncommenting the define which fits your platform endianness
  */
//#define DRV_BYTE_ORDER    DRV_BIG_ENDIAN
#define DRV_BYTE_ORDER    DRV_LITTLE_ENDIAN

#else /* defined __BYTE_ORDER__ */

#define DRV_LITTLE_ENDIAN  __ORDER_LITTLE_ENDIAN__
#define DRV_BIG_ENDIAN     __ORDER_BIG_ENDIAN__
#define DRV_BYTE_ORDER     __BYTE_ORDER__

#endif /* __BYTE_ORDER__*/
#endif /* DRV_BYTE_ORDER */

/**
  * @}
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bit7       : 1;
  uint8_t bit6       : 1;
  uint8_t bit5       : 1;
  uint8_t bit4       : 1;
  uint8_t bit3       : 1;
  uint8_t bit2       : 1;
  uint8_t bit1       : 1;
  uint8_t bit0       : 1;
#endif /* DRV_BYTE_ORDER */
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)


/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
  *              You can create a sensor configuration by your own or using
  *              Unico / Unicleo tools available on STMicroelectronics
  *              web site.
  *
  * @{
  *
  */

typedef struct {
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup LIS3DH_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format if SA0=0 -> 31 if SA0=1 -> 33 **/
#define LIS3DH_I2C_ADD_L   0x18U
#define LIS3DH_I2C_ADD_H   0x19U

#define LIS3DH_I2C_ADDRESS   LIS3DH_I2C_ADD_H

/** Device Identification (Who am I) **/
#define LIS3DH_ID          0x33U

/**
  * @}
  *
  */

#define LIS3DH_STATUS_REG_AUX        0x07U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t _1da              : 1;
  uint8_t _2da              : 1;
  uint8_t _3da              : 1;
  uint8_t _321da            : 1;
  uint8_t _1or              : 1;
  uint8_t _2or              : 1;
  uint8_t _3or              : 1;
  uint8_t _321or            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t _321or            : 1;
  uint8_t _3or              : 1;
  uint8_t _2or              : 1;
  uint8_t _1or              : 1;
  uint8_t _321da            : 1;
  uint8_t _3da              : 1;
  uint8_t _2da              : 1;
  uint8_t _1da              : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_status_reg_aux_t;

#define LIS3DH_OUT_ADC1_L            0x08U
#define LIS3DH_OUT_ADC1_H            0x09U
#define LIS3DH_OUT_ADC2_L            0x0AU
#define LIS3DH_OUT_ADC2_H            0x0BU
#define LIS3DH_OUT_ADC3_L            0x0CU
#define LIS3DH_OUT_ADC3_H            0x0DU
#define LIS3DH_WHO_AM_I              0x0FU

#define LIS3DH_CTRL_REG0             0x1EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01       : 7;
  uint8_t sdo_pu_disc       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sdo_pu_disc       : 1;
  uint8_t not_used_01       : 7;
#endif /* DRV_BYTE_ORDER */
} lis3dh_ctrl_reg0_t;

#define LIS3DH_TEMP_CFG_REG          0x1FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01       : 6;
  uint8_t adc_pd            : 1;
  uint8_t temp_en           : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t temp_en           : 1;
  uint8_t adc_pd            : 1;
  uint8_t not_used_01       : 6;
#endif /* DRV_BYTE_ORDER */
} lis3dh_temp_cfg_reg_t;

#define LIS3DH_CTRL_REG1             0x20U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xen               : 1;
  uint8_t yen               : 1;
  uint8_t zen               : 1;
  uint8_t lpen              : 1;
  uint8_t odr               : 4;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr               : 4;
  uint8_t lpen              : 1;
  uint8_t zen               : 1;
  uint8_t yen               : 1;
  uint8_t xen               : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_ctrl_reg1_t;

#define LIS3DH_CTRL_REG2             0x21U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t hp                : 3; /* HPCLICK + HP_IA2 + HP_IA1 -> HP */
  uint8_t fds               : 1;
  uint8_t hpcf              : 2;
  uint8_t hpm               : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hpm               : 2;
  uint8_t hpcf              : 2;
  uint8_t fds               : 1;
  uint8_t hp                : 3; /* HPCLICK + HP_IA2 + HP_IA1 -> HP */
#endif /* DRV_BYTE_ORDER */
} lis3dh_ctrl_reg2_t;

#define LIS3DH_CTRL_REG3             0x22U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t i1_overrun        : 1;
  uint8_t i1_wtm            : 1;
  uint8_t i1_321da          : 1;
  uint8_t i1_zyxda          : 1;
  uint8_t i1_ia2            : 1;
  uint8_t i1_ia1            : 1;
  uint8_t i1_click          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t i1_click          : 1;
  uint8_t i1_ia1            : 1;
  uint8_t i1_ia2            : 1;
  uint8_t i1_zyxda          : 1;
  uint8_t i1_321da          : 1;
  uint8_t i1_wtm            : 1;
  uint8_t i1_overrun        : 1;
  uint8_t not_used_01       : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_ctrl_reg3_t;

#define LIS3DH_CTRL_REG4             0x23U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sim               : 1;
  uint8_t st                : 2;
  uint8_t hr                : 1;
  uint8_t fs                : 2;
  uint8_t ble               : 1;
  uint8_t bdu               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t bdu               : 1;
  uint8_t ble               : 1;
  uint8_t fs                : 2;
  uint8_t hr                : 1;
  uint8_t st                : 2;
  uint8_t sim               : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_ctrl_reg4_t;

#define LIS3DH_CTRL_REG5             0x24U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t d4d_int2          : 1;
  uint8_t lir_int2          : 1;
  uint8_t d4d_int1          : 1;
  uint8_t lir_int1          : 1;
  uint8_t not_used_01       : 2;
  uint8_t fifo_en           : 1;
  uint8_t boot              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t boot              : 1;
  uint8_t fifo_en           : 1;
  uint8_t not_used_01       : 2;
  uint8_t lir_int1          : 1;
  uint8_t d4d_int1          : 1;
  uint8_t lir_int2          : 1;
  uint8_t d4d_int2          : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_ctrl_reg5_t;

#define LIS3DH_CTRL_REG6            0x25U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t int_polarity      : 1;
  uint8_t not_used_02       : 1;
  uint8_t i2_act            : 1;
  uint8_t i2_boot           : 1;
  uint8_t i2_ia2            : 1;
  uint8_t i2_ia1            : 1;
  uint8_t i2_click          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t i2_click          : 1;
  uint8_t i2_ia1            : 1;
  uint8_t i2_ia2            : 1;
  uint8_t i2_boot           : 1;
  uint8_t i2_act            : 1;
  uint8_t not_used_02       : 1;
  uint8_t int_polarity      : 1;
  uint8_t not_used_01       : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_ctrl_reg6_t;

#define LIS3DH_REFERENCE            0x26U
#define LIS3DH_STATUS_REG           0x27U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xda               : 1;
  uint8_t yda               : 1;
  uint8_t zda               : 1;
  uint8_t zyxda             : 1;
  uint8_t _xor              : 1;
  uint8_t yor               : 1;
  uint8_t zor               : 1;
  uint8_t zyxor             : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t zyxor             : 1;
  uint8_t zor               : 1;
  uint8_t yor               : 1;
  uint8_t _xor              : 1;
  uint8_t zyxda             : 1;
  uint8_t zda               : 1;
  uint8_t yda               : 1;
  uint8_t xda               : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_status_reg_t;

#define LIS3DH_OUT_X_L              0x28U
#define LIS3DH_OUT_X_H              0x29U
#define LIS3DH_OUT_Y_L              0x2AU
#define LIS3DH_OUT_Y_H              0x2BU
#define LIS3DH_OUT_Z_L              0x2CU
#define LIS3DH_OUT_Z_H              0x2DU
#define LIS3DH_FIFO_CTRL_REG        0x2EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fth               : 5;
  uint8_t tr                : 1;
  uint8_t fm                : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t fm                : 2;
  uint8_t tr                : 1;
  uint8_t fth               : 5;
#endif /* DRV_BYTE_ORDER */
} lis3dh_fifo_ctrl_reg_t;

#define LIS3DH_FIFO_SRC_REG         0x2FU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fss               : 5;
  uint8_t empty             : 1;
  uint8_t ovrn_fifo         : 1;
  uint8_t wtm               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t wtm               : 1;
  uint8_t ovrn_fifo         : 1;
  uint8_t empty             : 1;
  uint8_t fss               : 5;
#endif /* DRV_BYTE_ORDER */
} lis3dh_fifo_src_reg_t;

#define LIS3DH_INT1_CFG             0x30U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlie              : 1;
  uint8_t xhie              : 1;
  uint8_t ylie              : 1;
  uint8_t yhie              : 1;
  uint8_t zlie              : 1;
  uint8_t zhie              : 1;
  uint8_t _6d               : 1;
  uint8_t aoi               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t aoi               : 1;
  uint8_t _6d               : 1;
  uint8_t zhie              : 1;
  uint8_t zlie              : 1;
  uint8_t yhie              : 1;
  uint8_t ylie              : 1;
  uint8_t xhie              : 1;
  uint8_t xlie              : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_int1_cfg_t;

#define LIS3DH_INT1_SRC             0x31U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                : 1;
  uint8_t xh                : 1;
  uint8_t yl                : 1;
  uint8_t yh                : 1;
  uint8_t zl                : 1;
  uint8_t zh                : 1;
  uint8_t ia                : 1;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ia                : 1;
  uint8_t zh                : 1;
  uint8_t zl                : 1;
  uint8_t yh                : 1;
  uint8_t yl                : 1;
  uint8_t xh                : 1;
  uint8_t xl                : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_int1_src_t;

#define LIS3DH_INT1_THS             0x32U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths               : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ths               : 7;
#endif /* DRV_BYTE_ORDER */
} lis3dh_int1_ths_t;

#define LIS3DH_INT1_DURATION        0x33U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t d                 : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t d                 : 7;
#endif /* DRV_BYTE_ORDER */
} lis3dh_int1_duration_t;

#define LIS3DH_INT2_CFG             0x34U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlie              : 1;
  uint8_t xhie              : 1;
  uint8_t ylie              : 1;
  uint8_t yhie              : 1;
  uint8_t zlie              : 1;
  uint8_t zhie              : 1;
  uint8_t _6d               : 1;
  uint8_t aoi               : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t aoi               : 1;
  uint8_t _6d               : 1;
  uint8_t zhie              : 1;
  uint8_t zlie              : 1;
  uint8_t yhie              : 1;
  uint8_t ylie              : 1;
  uint8_t xhie              : 1;
  uint8_t xlie              : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_int2_cfg_t;

#define LIS3DH_INT2_SRC             0x35U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xl                : 1;
  uint8_t xh                : 1;
  uint8_t yl                : 1;
  uint8_t yh                : 1;
  uint8_t zl                : 1;
  uint8_t zh                : 1;
  uint8_t ia                : 1;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ia                : 1;
  uint8_t zh                : 1;
  uint8_t zl                : 1;
  uint8_t yh                : 1;
  uint8_t yl                : 1;
  uint8_t xh                : 1;
  uint8_t xl                : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_int2_src_t;

#define LIS3DH_INT2_THS             0x36U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths               : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ths               : 7;
#endif /* DRV_BYTE_ORDER */
} lis3dh_int2_ths_t;

#define LIS3DH_INT2_DURATION        0x37U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t d                 : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t d                 : 7;
#endif /* DRV_BYTE_ORDER */
} lis3dh_int2_duration_t;

#define LIS3DH_CLICK_CFG            0x38U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xs                : 1;
  uint8_t xd                : 1;
  uint8_t ys                : 1;
  uint8_t yd                : 1;
  uint8_t zs                : 1;
  uint8_t zd                : 1;
  uint8_t not_used_01       : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 2;
  uint8_t zd                : 1;
  uint8_t zs                : 1;
  uint8_t yd                : 1;
  uint8_t ys                : 1;
  uint8_t xd                : 1;
  uint8_t xs                : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_click_cfg_t;

#define LIS3DH_CLICK_SRC            0x39U
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t x                 : 1;
  uint8_t y                 : 1;
  uint8_t z                 : 1;
  uint8_t sign              : 1;
  uint8_t sclick            : 1;
  uint8_t dclick            : 1;
  uint8_t ia                : 1;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t ia                : 1;
  uint8_t dclick            : 1;
  uint8_t sclick            : 1;
  uint8_t sign              : 1;
  uint8_t z                 : 1;
  uint8_t y                 : 1;
  uint8_t x                 : 1;
#endif /* DRV_BYTE_ORDER */
} lis3dh_click_src_t;

#define LIS3DH_CLICK_THS            0x3AU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ths               : 7;
  uint8_t lir_click         : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t lir_click         : 1;
  uint8_t ths               : 7;
#endif /* DRV_BYTE_ORDER */
} lis3dh_click_ths_t;

#define LIS3DH_TIME_LIMIT           0x3BU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t tli               : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t tli               : 7;
#endif /* DRV_BYTE_ORDER */
} lis3dh_time_limit_t;

#define LIS3DH_TIME_LATENCY         0x3CU
typedef struct {
  uint8_t tla               : 8;
} lis3dh_time_latency_t;

#define LIS3DH_TIME_WINDOW          0x3DU
typedef struct {
  uint8_t tw                : 8;
} lis3dh_time_window_t;

#define LIS3DH_ACT_THS              0x3EU
typedef struct {
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t acth              : 7;
  uint8_t not_used_01       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used_01       : 1;
  uint8_t acth              : 7;
#endif /* DRV_BYTE_ORDER */
} lis3dh_act_ths_t;

#define LIS3DH_ACT_DUR              0x3FU
typedef struct {
  uint8_t actd              : 8;
} lis3dh_act_dur_t;

/**
  * @defgroup LIS3DH_Register_Union
  * @brief    This union group all the registers that has a bitfield
  *           description.
  *           This union is useful but not need by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union {
  lis3dh_status_reg_aux_t status_reg_aux;
  lis3dh_ctrl_reg0_t      ctrl_reg0;
  lis3dh_temp_cfg_reg_t   temp_cfg_reg;
  lis3dh_ctrl_reg1_t      ctrl_reg1;
  lis3dh_ctrl_reg2_t      ctrl_reg2;
  lis3dh_ctrl_reg3_t      ctrl_reg3;
  lis3dh_ctrl_reg4_t      ctrl_reg4;
  lis3dh_ctrl_reg5_t      ctrl_reg5;
  lis3dh_ctrl_reg6_t      ctrl_reg6;
  lis3dh_status_reg_t     status_reg;
  lis3dh_fifo_ctrl_reg_t  fifo_ctrl_reg;
  lis3dh_fifo_src_reg_t   fifo_src_reg;
  lis3dh_int1_cfg_t       int1_cfg;
  lis3dh_int1_src_t       int1_src;
  lis3dh_int1_ths_t       int1_ths;
  lis3dh_int1_duration_t  int1_duration;
  lis3dh_int2_cfg_t       int2_cfg;
  lis3dh_int2_src_t       int2_src;
  lis3dh_int2_ths_t       int2_ths;
  lis3dh_int2_duration_t  int2_duration;
  lis3dh_click_cfg_t      click_cfg;
  lis3dh_click_src_t      click_src;
  lis3dh_click_ths_t      click_ths;
  lis3dh_time_limit_t     time_limit;
  lis3dh_time_latency_t   time_latency;
  lis3dh_time_window_t    time_window;
  lis3dh_act_ths_t        act_ths;
  lis3dh_act_dur_t        act_dur;
  bitwise_t                 bitwise;
  uint8_t                   byte;
} lis3dh_reg_t;


#endif /* LIS3DH_SENSOR_H_ */
/*---------------------------------------------------------------------------*/
