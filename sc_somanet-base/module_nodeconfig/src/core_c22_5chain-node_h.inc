#pragma once
#include <flash.h>

/************
 * Core C22
 ************/

/* SPI Flash */
//extern in  port p_core_spiflash_miso;
//extern out port p_core_spiflash_ss;
//extern out port p_core_spiflash_clk;
//extern out port p_core_spiflash_mosi;

extern fl_SPIPorts p_spi_flash_0;
extern fl_SPIPorts p_spi_flash_1;
extern fl_SPIPorts p_spi_flash_2;
extern fl_SPIPorts p_spi_flash_3;
//extern fl_SPIPorts p_spi_flash_4;

/* RGB LED */
extern out port p_core_leds_0[3]; /* = { Red, Green, Blue } */
extern out port p_core_leds_1[3]; /* = { Red, Green, Blue } */
extern out port p_core_leds_2[3]; /* = { Red, Green, Blue } */
extern out port p_core_leds_3[3]; /* = { Red, Green, Blue } */
extern out port p_core_leds_4[3]; /* = { Red, Green, Blue } */
