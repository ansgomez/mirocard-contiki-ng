/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
#ifndef POLICY_H_
#define POLICY_H_
/*---------------------------------------------------------------------------*/
#include "transient.h"
#include "policy-pool.h"
#ifndef POLICY_BUFFER_SIZE
#error "missing defines from policy pool."
#endif
/*---------------------------------------------------------------------------*/
#define POLICY_SELECTION_UNSET      0xFFFF
#define POLICY_POWER_LEVEL_INVALID  0xFF
/*---------------------------------------------------------------------------*/
/**
 * Initialize policy sampler, like HW random number generator
 */
void policy_init(void);
/**
 * Estimate the power level based on the last interval
 * 
 * @param interval The time interval since the last activation [in seconds].
 * @return The index of the power level closest
 */
uint8_t policy_estimate_power_level(uint32_t interval);
/**
 * Sample new data packet entries for the next transmission
 * 
 * @param power_level Power level (i.e. policy index) of the policy to use
 * @return Array of the generated sample indexes
 * 
 * @note The return array's size is defined by `POLICY_SELECTION_SIZE`.
 *       The array is overwritten in subsequent calls this function.
 */
const uint16_t* const policy_generate_selection(uint8_t power_level);
/**
 * Get the scaling factor of the base time unit of the given power level.
 * 
 * @param power_level Power level (i.e. policy index) of the policy to use
 * @return The scaling factor of the base delta for the given power level
 */
uint16_t policy_get_delta_factor(uint8_t power_level);
/**
 * Update aggregate values
 * 
 * @param buffer Buffer containing the aggregate values to update
 * @param value The current sensor value to use for updating the aggregates
 */
void policy_update_aggregates(transient_data_unit_t* const buffer,
                              const transient_data_unit_t* const value);
/*---------------------------------------------------------------------------*/
#endif /* POLICY_H_ */
/*---------------------------------------------------------------------------*/
