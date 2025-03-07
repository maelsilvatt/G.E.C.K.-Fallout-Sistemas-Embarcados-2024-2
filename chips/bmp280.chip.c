// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT
// Copyright 2023 Luis Miguel Capacho Valbuena

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

#define ID        0xD0
#define DIG_T1_L  0x88
#define DIG_T1_H  0x89
#define DIG_T2_L  0x8A  
#define DIG_T2_H  0x8B
#define DIG_T3_L  0x8C
#define DIG_T3_H  0x8D
#define TEMP_XLSB 0xFC
#define TEMP_LSB  0xFB
#define TEMP_MSB  0xFA


typedef struct {
  pin_t cs;
  spi_dev_t spi0;
  uint8_t  spi_buffer[1];
  uint32_t temp;
  uint8_t nbyte;
  uint8_t addr;
} chip_state_t;

void chip_pin_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t*)user_data;

  if(value == LOW) {
    chip->nbyte = 0;
    spi_start(chip->spi0, chip->spi_buffer, sizeof(chip->spi_buffer));
  } else {
    spi_stop(chip->spi0);
  }
}

static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count) {
  chip_state_t *chip = (chip_state_t*)user_data;

  if( pin_read(chip->cs) == LOW ) {
    if(chip->nbyte == 0){  
      chip->addr = chip->spi_buffer[0];
    }  
    switch(chip->addr) {
      case ID:
        chip->spi_buffer[0] = 0x58;
        break;
      case DIG_T1_L:
        chip->spi_buffer[0] = 0x70;
        break;  
      case DIG_T1_H:
        chip->spi_buffer[0] = 0x6B;
        break;  
      case DIG_T2_L:
        chip->spi_buffer[0] = 0x43;
        break;  
      case DIG_T2_H:
        chip->spi_buffer[0] = 0x67;
        break;  
      case DIG_T3_L:
        chip->spi_buffer[0] = 0x18;
        break;  
      case DIG_T3_H:
        chip->spi_buffer[0] = 0xC0;
        break;  
      case TEMP_XLSB:
        chip->spi_buffer[0] = (uint8_t)(attr_read(chip->temp) << 4);
        break; 
      case TEMP_LSB:
        chip->spi_buffer[0] = (uint8_t)(attr_read(chip->temp) >> 4);
        break; 
      case TEMP_MSB:
        chip->spi_buffer[0] = (uint8_t)(attr_read(chip->temp) >> 12);
        break; 
    }
    printf("%x\n", chip->spi_buffer[0]);
    spi_start(chip->spi0, chip->spi_buffer, sizeof(chip->spi_buffer));
    chip->addr++;  
    chip->nbyte++;
  }
}

void chip_init() {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->cs = pin_init("CS", INPUT);

  const pin_watch_config_t watch_config = {
    .edge = BOTH,
    .pin_change = chip_pin_change,
    .user_data = chip,
  };
  
  pin_watch(chip->cs, &watch_config);

  const spi_config_t spi_config = {
    .sck = pin_init("SCK", INPUT),
    .mosi = pin_init("SDI", INPUT),
    .miso = pin_init("SDO", OUTPUT),
    .mode = 0,
    .done = chip_spi_done,
    .user_data = chip,
  };

  chip->spi0 = spi_init(&spi_config);

  chip->temp = attr_init("temp", 0); 
  chip->addr = 0;
}
