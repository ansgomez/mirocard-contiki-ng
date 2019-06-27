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
/**
 * Compile and run using:
 * gcc -std=c99 -Wall -Werror haar_standalone_test.c -o haar_standalone_test && ./haar_standalone_test
 */
/* ------------------------------------------------------------------------- */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* ------------------------------------------------------------------------- */
#define DEBUG 1
#define DATA_SIZE 1024
#define COEFF_SIZE 4
// #define INV_SQRT2   707 / 1000
#define INV_SQRT2 7071 / 10000
// #define INV_SQRT2   (0.707106781)
/*---------------------------------------------------------------------------*/
const int32_t data_orig[DATA_SIZE] = {
    2900, 2900, 3000, 3100, 3000, 3000, 2900, 2800, 2900, 2800, 2900, 2800,
    2700, 2700, 2700, 2600, 2500, 2500, 2500, 2600, 2500, 2600, 2600, 2700,
    2800, 2800, 2800, 2900, 2900, 2900, 3000, 3000, 3100, 3000, 3100, 3100,
    3200, 3100, 3000, 2900, 3000, 2900, 2900, 3000, 2900, 3000, 3000, 3100,
    3100, 3100, 3000, 2900, 2900, 3000, 3100, 3000, 3000, 3100, 3100, 3100,
    3100, 3100, 3000, 3100, 3200, 3300, 3300, 3400, 3300, 3300, 3400, 3400,
    3300, 3300, 3400, 3300, 3200, 3100, 3000, 3000, 3100, 3200, 3300, 3200,
    3300, 3200, 3100, 3000, 3100, 3000, 3100, 3100, 3200, 3200, 3200, 3300,
    3200, 3200, 3300, 3200, 3200, 3100, 3000, 3000, 2900, 3000, 2900, 2900,
    2800, 2700, 2800, 2800, 2700, 2800, 2700, 2700, 2800, 2900, 2800, 2700,
    2700, 2600, 2700, 2600, 2600, 2500, 2400, 2500, 2400, 2500, 2600, 2600,
    2700, 2800, 2900, 3000, 3000, 2900, 2800, 2700, 2700, 2600, 2500, 2400,
    2500, 2600, 2500, 2400, 2300, 2400, 2400, 2500, 2400, 2300, 2400, 2500,
    2500, 2500, 2500, 2500, 2600, 2700, 2800, 2700, 2600, 2600, 2600, 2600,
    2600, 2500, 2400, 2400, 2300, 2300, 2200, 2300, 2300, 2300, 2200, 2100,
    2200, 2300, 2400, 2400, 2400, 2500, 2500, 2500, 2600, 2500, 2500, 2400,
    2400, 2500, 2600, 2700, 2600, 2700, 2700, 2800, 2800, 2700, 2800, 2900,
    2900, 2900, 2900, 3000, 3000, 3000, 3000, 3100, 3100, 3000, 2900, 2800,
    2800, 2900, 2800, 2800, 2800, 2900, 2900, 2800, 2700, 2800, 2900, 2900,
    3000, 3100, 3000, 2900, 2900, 2900, 3000, 3000, 3100, 3200, 3200, 3200,
    3200, 3300, 3400, 3500, 3600, 3500, 3600, 3600, 3700, 3600, 3500, 3400,
    3400, 3300, 3300, 3300, 3200, 3300, 3400, 3400, 3300, 3300, 3200, 3100,
    3200, 3300, 3300, 3300, 3400, 3500, 3500, 3600, 3500, 3600, 3500, 3400,
    3400, 3400, 3400, 3500, 3500, 3400, 3400, 3400, 3300, 3200, 3300, 3400,
    3500, 3500, 3600, 3700, 3800, 3900, 3800, 3800, 3800, 3700, 3800, 3900,
    4000, 4100, 4100, 4200, 4200, 4200, 4100, 4200, 4300, 4200, 4300, 4200,
    4200, 4200, 4100, 4100, 4100, 4100, 4000, 4100, 4000, 4000, 4000, 4000,
    3900, 3800, 3800, 3700, 3800, 3900, 3900, 3800, 3700, 3800, 3700, 3600,
    3700, 3800, 3900, 3900, 3800, 3800, 3700, 3700, 3800, 3900, 3900, 3900,
    3900, 3800, 3700, 3600, 3500, 3400, 3300, 3200, 3300, 3300, 3300, 3400,
    3300, 3300, 3400, 3400, 3400, 3500, 3600, 3700, 3800, 3900, 3900, 3800,
    3900, 3900, 3900, 4000, 4000, 4100, 4000, 4100, 4100, 4100, 4100, 4200,
    4300, 4200, 4100, 4100, 4200, 4100, 4000, 4100, 4100, 4000, 4000, 3900,
    3800, 3900, 3900, 3900, 3900, 3900, 3900, 3900, 3800, 3800, 3800, 3900,
    3800, 3900, 4000, 4000, 4100, 4100, 4200, 4100, 4100, 4100, 4100, 4000,
    4000, 4000, 4100, 4100, 4200, 4100, 4200, 4300, 4400, 4300, 4200, 4200,
    4200, 4200, 4200, 4100, 4100, 4200, 4300, 4400, 4400, 4500, 4400, 4400,
    4400, 4400, 4500, 4400, 4400, 4300, 4200, 4300, 4300, 4400, 4300, 4300,
    4200, 4200, 4100, 4200, 4100, 4100, 4100, 4200, 4300, 4400, 4400, 4400,
    4500, 4600, 4700, 4700, 4800, 4900, 5000, 5100, 5200, 5200, 5300, 5300,
    5400, 5400, 5400, 5400, 5500, 5400, 5500, 5500, 5500, 5500, 5600, 5500,
    5600, 5700, 5700, 5700, 5700, 5800, 5700, 5600, 5600, 5500, 5400, 5300,
    5300, 5300, 5300, 5300, 5400, 5300, 5200, 5200, 5200, 5300, 5300, 5400,
    5500, 5500, 5600, 5700, 5700, 5600, 5700, 5800, 5900, 6000, 6100, 6200,
    6300, 6300, 6200, 6100, 6200, 6300, 6300, 6200, 6100, 6000, 5900, 5900,
    5800, 5700, 5800, 5900, 5800, 5800, 5700, 5700, 5600, 5700, 5700, 5800,
    5900, 6000, 6100, 6000, 5900, 6000, 5900, 5800, 5700, 5700, 5800, 5900,
    5900, 5800, 5700, 5800, 5800, 5800, 5900, 5800, 5700, 5700, 5700, 5700,
    5700, 5600, 5700, 5700, 5800, 5900, 5800, 5700, 5700, 5700, 5800, 5700,
    5800, 5900, 5800, 5900, 5900, 5900, 6000, 6100, 6100, 6000, 6000, 6000,
    6100, 6100, 6000, 5900, 6000, 6000, 6000, 6100, 6200, 6300, 6300, 6400,
    6400, 6300, 6400, 6400, 6300, 6400, 6400, 6300, 6400, 6400, 6500, 6500,
    6400, 6400, 6300, 6300, 6300, 6300, 6200, 6200, 6200, 6300, 6400, 6300,
    6200, 6300, 6400, 6300, 6200, 6300, 6200, 6200, 6200, 6300, 6300, 6300,
    6300, 6200, 6300, 6300, 6400, 6500, 6500, 6400, 6300, 6200, 6200, 6200,
    6300, 6400, 6400, 6300, 6400, 6500, 6400, 6400, 6500, 6500, 6500, 6500,
    6600, 6700, 6700, 6800, 6700, 6600, 6700, 6600, 6500, 6600, 6600, 6700,
    6800, 6900, 6800, 6900, 6800, 6700, 6600, 6700, 6700, 6700, 6800, 6700,
    6600, 6700, 6700, 6600, 6600, 6500, 6600, 6700, 6700, 6600, 6500, 6500,
    6400, 6400, 6400, 6300, 6400, 6400, 6500, 6400, 6400, 6500, 6400, 6300,
    6200, 6200, 6100, 6100, 6000, 6100, 6100, 6000, 5900, 6000, 6000, 6100,
    6000, 6100, 6200, 6100, 6000, 6000, 5900, 5900, 6000, 5900, 5900, 5800,
    5800, 5900, 5800, 5700, 5700, 5800, 5900, 5900, 6000, 6100, 6200, 6200,
    6300, 6300, 6400, 6500, 6400, 6400, 6500, 6400, 6500, 6600, 6600, 6500,
    6400, 6500, 6500, 6500, 6400, 6400, 6300, 6200, 6100, 6000, 6000, 6100,
    6000, 5900, 6000, 5900, 6000, 6000, 5900, 6000, 6000, 5900, 6000, 6100,
    6200, 6200, 6300, 6400, 6400, 6500, 6600, 6700, 6600, 6500, 6500, 6400,
    6500, 6400, 6300, 6200, 6200, 6200, 6200, 6300, 6400, 6300, 6300, 6200,
    6100, 6000, 6000, 6100, 6000, 5900, 6000, 5900, 6000, 6100, 6000, 6000,
    6000, 6000, 6100, 6100, 6100, 6000, 6100, 6100, 6200, 6200, 6300, 6200,
    6100, 6000, 5900, 5900, 5900, 5900, 5900, 5800, 5900, 5900, 5800, 5800,
    5900, 5800, 5900, 5800, 5700, 5700, 5700, 5700, 5700, 5600, 5700, 5700,
    5800, 5900, 6000, 6000, 5900, 6000, 6100, 6000, 6000, 5900, 5800, 5900,
    6000, 6100, 6100, 6000, 6000, 5900, 5800, 5700, 5800, 5900, 6000, 5900,
    5800, 5900, 6000, 6000, 6000, 6000, 6000, 6100, 6100, 6000, 5900, 6000,
    5900, 6000, 6000, 6100, 6200, 6200, 6300, 6400, 6500, 6600, 6500, 6400,
    6300, 6300, 6200, 6100, 6000, 5900, 5800, 5800, 5800, 5900, 5900, 5800,
    5900, 5900, 6000, 6000, 5900, 5900, 5800, 5900, 5900, 5900, 5900, 5900,
    6000, 5900, 5900, 5900, 6000, 6000, 6000, 6000, 6100, 6100, 6200, 6100,
    6200, 6300, 6300, 6200, 6300, 6200, 6300, 6400, 6500, 6400, 6300, 6400,
    6300, 6300, 6400, 6500, 6400, 6500, 6500, 6400, 6400, 6300, 6200, 6300,
    6200, 6300, 6300, 6300, 6200, 6300, 6200, 6100, 6000, 5900, 5900, 5900,
    6000, 5900, 6000, 5900, 5900, 6000, 5900, 5800, 5700, 5700, 5700, 5700,
    5600, 5500, 5600, 5600, 5700, 5700, 5800, 5800, 5800, 5900, 5800, 5700,
    5600, 5700, 5800, 5800, 5900, 5900, 6000, 5900, 5900, 5800, 5900, 6000,
    5900, 5900, 5900, 5800,
};
/*---------------------------------------------------------------------------*/
void haar_compress_inplace(int32_t *const data, uint32_t size) {
  // exit if size not power of two with all zero output
  if ((size & (size - 1)) > 0) {
    printf("ERROR: size not power of 2\n");
    return;
  }

  // increment index shift of the sum terms (array end aligned)
  for (uint32_t shift = 1; shift < size; shift = 2 * shift) {
#if DEBUG
    printf("s=%d\t", shift);
#endif
    // calculate next level of diff and sum terms
    for (uint32_t i = shift - 1; i < size; i = i + 2 * shift) {
      int32_t diff = (data[i + shift] - data[i]) * INV_SQRT2;
      int32_t sum = (data[i + shift] + data[i]) * INV_SQRT2;
      data[i] = diff;
      data[i + shift] = sum;
    }

#if DEBUG
    for (uint32_t i = 0; i < 16; i++) {
      printf("%d,\t", data[i]);
    }
    printf("...\n");
  }
#endif
}
/*---------------------------------------------------------------------------*/
void haar_decompress_inplace(int32_t *const data, uint32_t size) {
  // exit if size not power of two with all zero output
  if ((size & (size - 1)) > 0) {
    printf("ERROR: size not power of 2\n");
    return;
  }

  // decrement index shift of the sum terms (array end aligned)
  for (uint32_t shift = size / 2; shift > 0; shift = shift / 2) {
#if DEBUG
    printf("s=%d\t", shift);
#endif
    // calculate current level of value and sum terms
    for (uint32_t i = shift - 1; i < size; i = i + 2 * shift) {
      int32_t val = (data[i + shift] - data[i]) * INV_SQRT2;
      int32_t sum = (data[i + shift] + data[i]) * INV_SQRT2;
      data[i] = val;
      data[i + shift] = sum;
    }

#if DEBUG
    for (uint32_t i = 0; i < 16; i++) {
      printf("%d,\t", data[i]);
    }
    printf("...\n");
  }
#endif
}
/*---------------------------------------------------------------------------*/
int main(void) {
  printf("copy data...\n");
  int32_t data[DATA_SIZE];
  memcpy(data, data_orig, sizeof(data_orig));

  /*-------------------------------------------------------------------------*/
  // compression
  printf("compress inplace...\n");
  haar_compress_inplace(data, DATA_SIZE);

  printf("compression result:\n");
  printf("[");
  for (uint32_t i = 0; i < DATA_SIZE; i++) {
    printf("%d,", data[i]);
  }
  printf("]\n");
  /*-------------------------------------------------------------------------*/
  // select n most significant values
  int32_t coeff_value[COEFF_SIZE] = {0};
  uint32_t coeff_index[COEFF_SIZE] = {-1};

  // uint32_t i = DATA_SIZE;
  // while(i > 0) {
  //     i--;
  for (uint32_t i = 0; i < DATA_SIZE; i++) {
    // insert element in list, if larger than lowest selected value
    if (abs(data[i]) > abs(coeff_value[COEFF_SIZE - 1])) {
      // reorder selection list by swapping, until insertion position is found
      uint32_t j = COEFF_SIZE;
      while (j > 0) {
        j--;
        if (j > 0 && abs(data[i]) > abs(coeff_value[j - 1])) {
          // swap values and indexes
          coeff_value[j] = coeff_value[j - 1];
          coeff_index[j] = coeff_index[j - 1];
        } else {
          // insert value and index, break
          coeff_value[j] = data[i];
          coeff_index[j] = i;
          break;
        }
      }
    }
  }

  printf("N largest coefficient with index:\n");
  printf("{\n");
  for (uint32_t i = 0; i < COEFF_SIZE; i++) {
    printf("%d:\t%d,\n", coeff_index[i], coeff_value[i]);
  }
  printf("}\n");
  /*-------------------------------------------------------------------------*/
  // decompression
  printf("decompress inplace...\n");
  haar_decompress_inplace(data, DATA_SIZE);

  printf("result:\n");
  printf("[");
  for (uint32_t i = 0; i < DATA_SIZE; i++) {
    printf("%d,", data[i]);
  }
  printf("]\n");

  printf("error:\n");
  printf("[");
  for (uint32_t i = 0; i < DATA_SIZE; i++) {
    printf("%d,", data[i] - data_orig[i]);
  }
  printf("]\n");
}
/*---------------------------------------------------------------------------*/
