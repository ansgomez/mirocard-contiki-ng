/*
 * Copyright (c) 2020, Andres Gomez, Miromico AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*---------------------------------------------------------------------------*/
#ifndef LIS3DH_H_
#define LIS3DH_H_
/*---------------------------------------------------------------------------*/
#include "dev/spi.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#define LIS3DH_USE_CLOCKSTRETCH  0
#define LIS3DH_USE_FAHRENHEIT    0
/*---------------------------------------------------------------------------*/

// #ifndef LIS3DH_I2C_CONTROLLER

// #define LIS3DH_I2C_CONTROLLER          0xFF /* No controller */

// #define LIS3DH_I2C_PIN_SCL             GPIO_HAL_PIN_UNKNOWN
// #define LIS3DH_I2C_PIN_SDA             GPIO_HAL_PIN_UNKNOWN

// #define LIS3DH_I2C_ADDRESS             0x70

// #define LIS3DH_INTERFACE                BOARD_I2C_INTERFACE_0

// #endif /* LIS3DH_I2C_CONTROLLER */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include "lis3dh-sensor.h"


typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t *,
                                    uint16_t);
typedef int32_t (*stmdev_read_ptr) (void *, uint8_t, uint8_t *,
                                    uint16_t);

typedef struct {
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;



/**
  * @}
  *
  */

int32_t lis3dh_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data,
                        uint16_t len);
int32_t lis3dh_write_reg(stmdev_ctx_t *ctx, uint8_t reg,
                         uint8_t *data,
                         uint16_t len);

float lis3dh_from_fs2_hr_to_mg(int16_t lsb);
float lis3dh_from_fs4_hr_to_mg(int16_t lsb);
float lis3dh_from_fs8_hr_to_mg(int16_t lsb);
float lis3dh_from_fs16_hr_to_mg(int16_t lsb);
float lis3dh_from_lsb_hr_to_celsius(int16_t lsb);

float lis3dh_from_fs2_nm_to_mg(int16_t lsb);
float lis3dh_from_fs4_nm_to_mg(int16_t lsb);
float lis3dh_from_fs8_nm_to_mg(int16_t lsb);
float lis3dh_from_fs16_nm_to_mg(int16_t lsb);
float lis3dh_from_lsb_nm_to_celsius(int16_t lsb);

float lis3dh_from_fs2_lp_to_mg(int16_t lsb);
float lis3dh_from_fs4_lp_to_mg(int16_t lsb);
float lis3dh_from_fs8_lp_to_mg(int16_t lsb);
float lis3dh_from_fs16_lp_to_mg(int16_t lsb);
float lis3dh_from_lsb_lp_to_celsius(int16_t lsb);

int32_t lis3dh_temp_status_reg_get_stm(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lis3dh_temp_data_ready_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_temp_data_ovr_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_temperature_raw_get_stm(stmdev_ctx_t *ctx, int16_t *val);

int32_t lis3dh_adc_raw_get_stm(stmdev_ctx_t *ctx, int16_t *buff);

typedef enum {
  LIS3DH_AUX_DISABLE          = 0,
  LIS3DH_AUX_ON_TEMPERATURE   = 3,
  LIS3DH_AUX_ON_PADS          = 1,
} lis3dh_temp_en_t;
int32_t lis3dh_aux_adc_set_stm(stmdev_ctx_t *ctx, lis3dh_temp_en_t val);
int32_t lis3dh_aux_adc_get_stm(stmdev_ctx_t *ctx, lis3dh_temp_en_t *val);

typedef enum {
  LIS3DH_HR_12bit   = 0,
  LIS3DH_NM_10bit   = 1,
  LIS3DH_LP_8bit    = 2,
} lis3dh_op_md_t;
int32_t lis3dh_operating_mode_set_stm(stmdev_ctx_t *ctx,
                                  lis3dh_op_md_t val);
int32_t lis3dh_operating_mode_get_stm(stmdev_ctx_t *ctx,
                                  lis3dh_op_md_t *val);

typedef enum {
  LIS3DH_POWER_DOWN                      = 0x00,
  LIS3DH_ODR_1Hz                         = 0x01,
  LIS3DH_ODR_10Hz                        = 0x02,
  LIS3DH_ODR_25Hz                        = 0x03,
  LIS3DH_ODR_50Hz                        = 0x04,
  LIS3DH_ODR_100Hz                       = 0x05,
  LIS3DH_ODR_200Hz                       = 0x06,
  LIS3DH_ODR_400Hz                       = 0x07,
  LIS3DH_ODR_1kHz620_LP                  = 0x08,
  LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP    = 0x09,
} lis3dh_odr_t;
int32_t lis3dh_data_rate_set_stm(stmdev_ctx_t *ctx, lis3dh_odr_t val);
int32_t lis3dh_data_rate_get_stm(stmdev_ctx_t *ctx, lis3dh_odr_t *val);

int32_t lis3dh_high_pass_on_outputs_set_stm(stmdev_ctx_t *ctx,
                                        uint8_t val);
int32_t lis3dh_high_pass_on_outputs_get_stm(stmdev_ctx_t *ctx,
                                        uint8_t *val);

typedef enum {
  LIS3DH_AGGRESSIVE  = 0,
  LIS3DH_STRONG      = 1,
  LIS3DH_MEDIUM      = 2,
  LIS3DH_LIGHT       = 3,
} lis3dh_hpcf_t;
int32_t lis3dh_high_pass_bandwidth_set_stm(stmdev_ctx_t *ctx,
                                       lis3dh_hpcf_t val);
int32_t lis3dh_high_pass_bandwidth_get_stm(stmdev_ctx_t *ctx,
                                       lis3dh_hpcf_t *val);

typedef enum {
  LIS3DH_NORMAL_WITH_RST  = 0,
  LIS3DH_REFERENCE_MODE   = 1,
  LIS3DH_NORMAL           = 2,
  LIS3DH_AUTORST_ON_INT   = 3,
} lis3dh_hpm_t;
int32_t lis3dh_high_pass_mode_set_stm(stmdev_ctx_t *ctx,
                                  lis3dh_hpm_t val);
int32_t lis3dh_high_pass_mode_get_stm(stmdev_ctx_t *ctx,
                                  lis3dh_hpm_t *val);

typedef enum {
  LIS3DH_2g   = 0,
  LIS3DH_4g   = 1,
  LIS3DH_8g   = 2,
  LIS3DH_16g  = 3,
} lis3dh_fs_t;
int32_t lis3dh_full_scale_set_stm(stmdev_ctx_t *ctx, lis3dh_fs_t val);
int32_t lis3dh_full_scale_get_stm(stmdev_ctx_t *ctx, lis3dh_fs_t *val);

int32_t lis3dh_block_data_update_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_block_data_update_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_filter_reference_set_stm(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lis3dh_filter_reference_get_stm(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lis3dh_xl_data_ready_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_xl_data_ovr_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_acceleration_raw_get_stm(stmdev_ctx_t *ctx, int16_t *val);

int32_t lis3dh_device_id_get_stm(stmdev_ctx_t *ctx, uint8_t *buff);

typedef enum {
  LIS3DH_ST_DISABLE   = 0,
  LIS3DH_ST_POSITIVE  = 1,
  LIS3DH_ST_NEGATIVE  = 2,
} lis3dh_st_t;
int32_t lis3dh_self_test_set_stm(stmdev_ctx_t *ctx, lis3dh_st_t val);
int32_t lis3dh_self_test_get_stm(stmdev_ctx_t *ctx, lis3dh_st_t *val);

typedef enum {
  LIS3DH_LSB_AT_LOW_ADD = 0,
  LIS3DH_MSB_AT_LOW_ADD = 1,
} lis3dh_ble_t;
int32_t lis3dh_data_format_set_stm(stmdev_ctx_t *ctx, lis3dh_ble_t val);
int32_t lis3dh_data_format_get_stm(stmdev_ctx_t *ctx, lis3dh_ble_t *val);

int32_t lis3dh_boot_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_boot_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_status_get_stm(stmdev_ctx_t *ctx,
                          lis3dh_status_reg_t *val);

int32_t lis3dh_int1_gen_conf_set_stm(stmdev_ctx_t *ctx,
                                 lis3dh_int1_cfg_t *val);
int32_t lis3dh_int1_gen_conf_get_stm(stmdev_ctx_t *ctx,
                                 lis3dh_int1_cfg_t *val);

int32_t lis3dh_int1_gen_source_get_stm(stmdev_ctx_t *ctx,
                                   lis3dh_int1_src_t *val);

int32_t lis3dh_int1_gen_threshold_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_int1_gen_threshold_get_stm(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lis3dh_int1_gen_duration_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_int1_gen_duration_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_int2_gen_conf_set_stm(stmdev_ctx_t *ctx,
                                 lis3dh_int2_cfg_t *val);
int32_t lis3dh_int2_gen_conf_get_stm(stmdev_ctx_t *ctx,
                                 lis3dh_int2_cfg_t *val);

int32_t lis3dh_int2_gen_source_get_stm(stmdev_ctx_t *ctx,
                                   lis3dh_int2_src_t *val);

int32_t lis3dh_int2_gen_threshold_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_int2_gen_threshold_get_stm(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lis3dh_int2_gen_duration_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_int2_gen_duration_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS3DH_DISC_FROM_INT_GENERATOR  = 0,
  LIS3DH_ON_INT1_GEN              = 1,
  LIS3DH_ON_INT2_GEN              = 2,
  LIS3DH_ON_TAP_GEN               = 4,
  LIS3DH_ON_INT1_INT2_GEN         = 3,
  LIS3DH_ON_INT1_TAP_GEN          = 5,
  LIS3DH_ON_INT2_TAP_GEN          = 6,
  LIS3DH_ON_INT1_INT2_TAP_GEN     = 7,
} lis3dh_hp_t;
int32_t lis3dh_high_pass_int_conf_set_stm(stmdev_ctx_t *ctx,
                                      lis3dh_hp_t val);
int32_t lis3dh_high_pass_int_conf_get_stm(stmdev_ctx_t *ctx,
                                      lis3dh_hp_t *val);

int32_t lis3dh_pin_int1_config_set_stm(stmdev_ctx_t *ctx,
                                   lis3dh_ctrl_reg3_t *val);
int32_t lis3dh_pin_int1_config_get_stm(stmdev_ctx_t *ctx,
                                   lis3dh_ctrl_reg3_t *val);

int32_t lis3dh_int2_pin_detect_4d_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_int2_pin_detect_4d_get_stm(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum {
  LIS3DH_INT2_PULSED   = 0,
  LIS3DH_INT2_LATCHED  = 1,
} lis3dh_lir_int2_t;
int32_t lis3dh_int2_pin_notification_mode_set_stm(stmdev_ctx_t *ctx,
                                              lis3dh_lir_int2_t val);
int32_t lis3dh_int2_pin_notification_mode_get_stm(stmdev_ctx_t *ctx,
                                              lis3dh_lir_int2_t *val);

int32_t lis3dh_int1_pin_detect_4d_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_int1_pin_detect_4d_get_stm(stmdev_ctx_t *ctx,
                                      uint8_t *val);

typedef enum {
  LIS3DH_INT1_PULSED   = 0,
  LIS3DH_INT1_LATCHED  = 1,
} lis3dh_lir_int1_t;
int32_t lis3dh_int1_pin_notification_mode_set_stm(stmdev_ctx_t *ctx,
                                              lis3dh_lir_int1_t val);
int32_t lis3dh_int1_pin_notification_mode_get_stm(stmdev_ctx_t *ctx,
                                              lis3dh_lir_int1_t *val);

int32_t lis3dh_pin_int2_config_set_stm(stmdev_ctx_t *ctx,
                                   lis3dh_ctrl_reg6_t *val);
int32_t lis3dh_pin_int2_config_get_stm(stmdev_ctx_t *ctx,
                                   lis3dh_ctrl_reg6_t *val);

int32_t lis3dh_fifo_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_fifo_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_fifo_watermark_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_fifo_watermark_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS3DH_INT1_GEN = 0,
  LIS3DH_INT2_GEN = 1,
} lis3dh_tr_t;
int32_t lis3dh_fifo_trigger_event_set_stm(stmdev_ctx_t *ctx,
                                      lis3dh_tr_t val);
int32_t lis3dh_fifo_trigger_event_get_stm(stmdev_ctx_t *ctx,
                                      lis3dh_tr_t *val);

typedef enum {
  LIS3DH_BYPASS_MODE           = 0,
  LIS3DH_FIFO_MODE             = 1,
  LIS3DH_DYNAMIC_STREAM_MODE   = 2,
  LIS3DH_STREAM_TO_FIFO_MODE   = 3,
} lis3dh_fm_t;
int32_t lis3dh_fifo_mode_set_stm(stmdev_ctx_t *ctx, lis3dh_fm_t val);
int32_t lis3dh_fifo_mode_get_stm(stmdev_ctx_t *ctx, lis3dh_fm_t *val);

int32_t lis3dh_fifo_status_get_stm(stmdev_ctx_t *ctx,
                               lis3dh_fifo_src_reg_t *val);

int32_t lis3dh_fifo_data_level_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_fifo_empty_flag_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_fifo_ovr_flag_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_fifo_fth_flag_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_tap_conf_set_stm(stmdev_ctx_t *ctx,
                            lis3dh_click_cfg_t *val);
int32_t lis3dh_tap_conf_get_stm(stmdev_ctx_t *ctx,
                            lis3dh_click_cfg_t *val);

int32_t lis3dh_tap_source_get_stm(stmdev_ctx_t *ctx,
                              lis3dh_click_src_t *val);

int32_t lis3dh_tap_threshold_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_tap_threshold_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS3DH_TAP_PULSED   = 0,
  LIS3DH_TAP_LATCHED  = 1,
} lis3dh_lir_click_t;
int32_t lis3dh_tap_notification_mode_set_stm(stmdev_ctx_t *ctx,
                                         lis3dh_lir_click_t val);
int32_t lis3dh_tap_notification_mode_get_stm(stmdev_ctx_t *ctx,
                                         lis3dh_lir_click_t *val);

int32_t lis3dh_shock_dur_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_shock_dur_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_quiet_dur_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_quiet_dur_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_double_tap_timeout_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_double_tap_timeout_get_stm(stmdev_ctx_t *ctx,
                                      uint8_t *val);

int32_t lis3dh_act_threshold_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_act_threshold_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lis3dh_act_timeout_set_stm(stmdev_ctx_t *ctx, uint8_t val);
int32_t lis3dh_act_timeout_get_stm(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  LIS3DH_PULL_UP_DISCONNECT  = 0,
  LIS3DH_PULL_UP_CONNECT     = 1,
} lis3dh_sdo_pu_disc_t;
int32_t lis3dh_pin_sdo_sa0_mode_set_stm(stmdev_ctx_t *ctx,
                                    lis3dh_sdo_pu_disc_t val);
int32_t lis3dh_pin_sdo_sa0_mode_get_stm(stmdev_ctx_t *ctx,
                                    lis3dh_sdo_pu_disc_t *val);

typedef enum {
  LIS3DH_SPI_4_WIRE = 0,
  LIS3DH_SPI_3_WIRE = 1,
} lis3dh_sim_t;
int32_t lis3dh_spi_mode_set_stm(stmdev_ctx_t *ctx, lis3dh_sim_t val);
int32_t lis3dh_spi_mode_get_stm(stmdev_ctx_t *ctx, lis3dh_sim_t *val);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/*---------------------------------------------------------------------------*/
#endif /* LIS3DH_H_ */
/*---------------------------------------------------------------------------*/
