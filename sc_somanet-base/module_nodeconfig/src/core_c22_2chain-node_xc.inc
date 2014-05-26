#pragma once

#include <xs1.h>
#include <platform.h>
#include <flash.h>

/************
 * Core C22
 ************/

 /* SPI port definitions */ 
on stdcore[0]: fl_SPIPorts p_spi_flash_0 = { PORT_SPI_MISO_0,
 								PORT_SPI_SS_0,
 								PORT_SPI_CLK_0,
 								PORT_SPI_MOSI_0,
 								XS1_CLKBLK_2 };
 								
on stdcore[4]: fl_SPIPorts p_spi_flash_1 = { PORT_SPI_MISO_1,
								PORT_SPI_SS_1,
								PORT_SPI_CLK_1,
								PORT_SPI_MOSI_1,
								XS1_CLKBLK_2 };


 								
/* RGB LED */
on stdcore[1]: out port p_core_leds_0[3] = { XS1_PORT_1M,   // Red 
					   						XS1_PORT_1L,   // Green 
					   						XS1_PORT_1K }; // Blue 
					   
on stdcore[5]: out port p_core_leds_1[3] = { XS1_PORT_1M,   // Red 
					   						XS1_PORT_1L,   // Green 
					   						XS1_PORT_1K }; // Blue 
