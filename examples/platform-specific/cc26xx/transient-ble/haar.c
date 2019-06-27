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
/* ------------------------------------------------------------------------- */
#include <stdint.h>
#include <string.h>

#include "haar.h"
/*---------------------------------------------------------------------------*/
#define INV_SQRT2   7071 / 10000
// #define INV_SQRT2   707 / 1000
// #define INV_SQRT2   (0.707106781)
/*---------------------------------------------------------------------------*/
void haar_compress_inplace(int32_t *const data, uint32_t size) {
  // do not transform anything if size is not a power of two
  if ((size & (size - 1)) > 0) {
    return;
  }

  // increment index shift of the sum terms (array end aligned)
  for (uint32_t shift = 1; shift < size; shift = 2 * shift) {
    // calculate next level of diff and sum terms
    for (uint32_t i = shift - 1; i < size; i = i + 2 * shift) {
      int32_t diff = (data[i + shift] - data[i]) * INV_SQRT2;
      int32_t sum = (data[i + shift] + data[i]) * INV_SQRT2;
      data[i] = diff;
      data[i + shift] = sum;
    }
  }
}
/*---------------------------------------------------------------------------*/
void haar_decompress_inplace(int32_t *const data, uint32_t size) {
  // do not transform anything if size is not a power of two
  if ((size & (size - 1)) > 0) {
    return;
  }

  // decrement index shift of the sum terms (array end aligned)
  for (uint32_t shift = size / 2; shift > 0; shift = shift / 2) {
    // calculate current level of value and sum terms
    for (uint32_t i = shift - 1; i < size; i = i + 2 * shift) {
      int32_t val = (data[i + shift] - data[i]) * INV_SQRT2;
      int32_t sum = (data[i + shift] + data[i]) * INV_SQRT2;
      data[i] = val;
      data[i + shift] = sum;
    }
  }
}
/*---------------------------------------------------------------------------*/
