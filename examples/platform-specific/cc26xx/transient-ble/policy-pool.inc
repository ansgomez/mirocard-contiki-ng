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
/**
 * This policy pool header was generated on ${GENERATION_TIME}, using file
 *   `${POLICY_FILE}`
 */
/*---------------------------------------------------------------------------*/
#ifndef POLICY_POOL_H_
#define POLICY_POOL_H_
/*---------------------------------------------------------------------------*/
/**
 * Number of power levels for which a policy was generated
 */
#define POLICY_POWER_LEVELS       ${POWER_LEVELS}
/**
 * Number of buffered samples the policy considers (aging and aggregate values)
 */
#define POLICY_BUFFER_SIZE        ${BUFFER_SIZE}
/**
 * Number of aggregates values in the policy
 */
#define POLICY_BUFFER_AGGREGATES  ${AGGREGATES}
/**
 * Number of values for which the selction was optimized
 */
#define POLICY_SELECTION_SIZE     ${SELECTION_SIZE}
/**
 * Selection base index of the first aggregate value
 */
#define POLICY_AGGREGATE_OFFSET   (POLICY_BUFFER_SIZE - POLICY_BUFFER_AGGREGATES)
/*---------------------------------------------------------------------------*/
/**
 * The individual power levels in nW for which the policies were optimized
 */
static const uint32_t policy_pool_power[POLICY_POWER_LEVELS] =
  ${POOL_POWER};
/**
 * The scaling factor of the base time unit for which the policies were optimized
 */
static const uint16_t policy_pool_delta[POLICY_POWER_LEVELS] =
  ${POOL_DELTA};
/**
 * The cumulative distribution pool of the optimal communication policies
 */
static const uint16_t policy_pool_cdf[POLICY_POWER_LEVELS][POLICY_BUFFER_SIZE] =
  ${POOL_CDF};
/**
 * The bin assignment of the pool of the optimal communication policies
 */
static const uint8_t policy_pool_bin[POLICY_POWER_LEVELS][POLICY_BUFFER_SIZE] =
  ${POOL_BIN};
/*---------------------------------------------------------------------------*/
#endif /* POLICY_POOL_H_ */
/*---------------------------------------------------------------------------*/
