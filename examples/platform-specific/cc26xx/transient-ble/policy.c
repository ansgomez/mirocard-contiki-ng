/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich)
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
#include <stdint.h>
#include <stdio.h>

#include "contiki.h"
#include "lib/random.h"

#include "energy-model.h"
#include "policy-pool.h"
#include "policy.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#if !(CC26XX_UART_CONF_ENABLE)
#warning "running in debug configuration while serial is NOT enabled!"
#endif
#else
#define PRINTF(...)
#endif
/* ------------------------------------------------------------------------- */
/**
 * Base EWMA span for multiple aggregate values
 */
#define EWMA_BASE_ALPHA   ((POLICY_BUFFER_SIZE - POLICY_BUFFER_AGGREGATES) / 10)
/* ------------------------------------------------------------------------- */
/**
 * Buffer for storing the sampled policy indexes
 */
static uint16_t selection_indexes[POLICY_SELECTION_SIZE] = 
  {POLICY_SELECTION_UNSET};
/*---------------------------------------------------------------------------*/
void policy_init(void) {
  PRINTF("initialize random number generator\n");
  random_init(0);
}
/*---------------------------------------------------------------------------*/
uint8_t policy_estimate_power_level(uint32_t interval) {
  PRINTF("estimate power level for interval of %lu seconds\n", interval);

  // for zero interval return the maximum level
  if (interval == 0) {
    return POLICY_POWER_LEVELS - 1;
  }

  // calculate average input power from interval
  uint32_t power = ENERGY_PER_ACTIVATION / interval + ENERGY_SLEEP_POWER;

  // lookup closes energy level in the policy pool
  uint8_t power_level = POLICY_POWER_LEVEL_INVALID;
  uint32_t power_diff_min = 0xFFFFFFFF;
  for (uint8_t level = 0; level < POLICY_POWER_LEVELS; level++) {
    // get absolute difference in power
    uint32_t power_diff;
    if (policy_pool_power[level] > power) {
      power_diff = policy_pool_power[level] - power;
    } else {
      power_diff = power - policy_pool_power[level];
    }

    // for smaller difference update power level
    if (power_diff < power_diff_min) {
      power_level = level;
      power_diff_min = power_diff;
    }
  }

  // return the closest power level, i.e. its index
  return power_level;
}
/*---------------------------------------------------------------------------*/
const uint16_t* const policy_generate_selection(uint8_t power_level) {
    PRINTF("sample selection for power level %u\n", power_level);

  // sample indexes for each selection element
  for (uint8_t sel_index = 0; sel_index < POLICY_SELECTION_SIZE; sel_index++) {
    // reset selection index
    selection_indexes[sel_index] = POLICY_SELECTION_UNSET;

    // get a random number
    uint16_t random_sample = random_rand();
    PRINTF("rng sample: %u\n", random_sample);

    // decrement if highest value to avoid slecting largest index problems
    if (random_sample == RANDOM_RAND_MAX) {
      random_sample = random_sample - 1;
    }

    // traverse the CDF of the current bin
    for(uint16_t cdf_index = 0; cdf_index < POLICY_BUFFER_SIZE; cdf_index++) {
      /*
       * Set the selection index if the following conditions hold:
       * a) CDF value is for current bin (bin packing stored compressed format)
       * b) the random sample is smaller (sample specific distribution)
       * c) the selction has not been set (no break to keep runtime consistent)
       */
      if ((policy_pool_bin[power_level][cdf_index] == sel_index) &&
          (random_sample < policy_pool_cdf[power_level][cdf_index]) &&
          (selection_indexes[sel_index] == POLICY_SELECTION_UNSET)) {
        selection_indexes[sel_index] = cdf_index;
        // NOTE: *NO* early loop break for energy/runtime consistency reasons!
      }
    }
  }

  // return the sampled selection
  return selection_indexes;
}
/*---------------------------------------------------------------------------*/
uint16_t policy_get_delta_factor(uint8_t power_level) {
  return policy_pool_delta[power_level];
}
/*---------------------------------------------------------------------------*/
