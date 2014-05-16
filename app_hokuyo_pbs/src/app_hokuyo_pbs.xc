/* app_hokuyo_pbs
 *
 * Test appliction for hokuyo PBS-03JN.
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Author: Viatcheslav Tretyakov <vtretyakov@synapticon.com>
 * Changes:
 */

#include <platform.h>
#include <xs1.h>
#include <print.h>


#include "uart_rx.h"
#include "uart_tx.h"

#include "hokuyo_pbs_driver.h"

#define MAX_BUFFER_SIZE   1024
#define BAUD_RATE 57600

buffered in port:1 p_rx = on tile[3] : XS1_PORT_1E;
out port p_tx = on tile[3] : XS1_PORT_1A;

on tile[3]: clock clk_adc  = XS1_CLKBLK_1;

/*---------------------------------------------------------------------------
 global variables
 ---------------------------------------------------------------------------*/
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))


void navigation_routine(chanend c_laser_data, laser_data_t &laser_data_packet);
void init_data(laser_data_t &laser_data_packet);


int main(void) {

	chan c_laser_data;

	/*UART communication channels*/
	chan c_chanTX, c_chanRX;


	par
	{
		on tile[1] :
		{
		    laser_data_t laser_data_packet;
		    init_data(laser_data_packet);
		    navigation_routine(c_laser_data, laser_data_packet);
		}
		on tile[3] :
		{
		    unsigned char tx_buffer[MAX_BUFFER_SIZE];
		    unsigned char rx_buffer[MAX_BUFFER_SIZE];
		    par {
		        uart_rx(p_rx, rx_buffer, ARRAY_SIZE(rx_buffer), BAUD_RATE, 7, UART_TX_PARITY_NONE, 1, c_chanRX);
		        uart_tx(p_tx, tx_buffer, ARRAY_SIZE(tx_buffer), BAUD_RATE, 7, UART_TX_PARITY_NONE, 1, c_chanTX);
		        run_scanner(c_chanRX, c_chanTX, c_laser_data);
		    }
		}

	}

	return 0;
}




void navigation_routine(chanend c_laser_data, laser_data_t &laser_data_packet)
{
	while(1)
	{

		get_laser_data(c_laser_data, laser_data_packet);

		printstr("id ");printint(laser_data_packet.id);printstr(" :");printint(laser_data_packet.ranges_mm[60]);printstrln(" mm");
	}
}

void init_data(laser_data_t &laser_data_packet)
{
    int i = 0;
    laser_data_packet.length = 0;
    laser_data_packet.id = 0;
    for (i; i < NUMBER_OF_RANGE_READINGS; i++)
    {
        laser_data_packet.ranges_mm[i] = 0;
    }
}
