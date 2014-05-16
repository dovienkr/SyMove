/* hokuyo_pbs_driver
 *
 * Hokuyo PBS-03JN driver
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Author: Viatcheslav Tretyakov <vtretyakov@synapticon.com>
 * Changes:
 */

#ifndef _hokuyo_pbs_driver_h_
#define _hokuyo_pbs_driver_h_

#include <stdint.h>
#include "uart_rx.h"

#define BAUD_RATE 57600
#define NUMBER_OF_RANGE_READINGS 121
#define MAX_RANGE_MM 3500

	// storage for outgoing data
	typedef struct t_laser_data
	{
		/** Number of readings in the scan */
		unsigned int length;
		/** A unique, increasing, ID for the scan */
		unsigned int id;
		/** Range readings [mm]. */
		unsigned int ranges_mm[NUMBER_OF_RANGE_READINGS];
	}laser_data_t;

	// Main function for device thread. Must run always in your main.
	void run_scanner(chanend c_uartRX, chanend c_uartTX, chanend c_laser_data);

	int send_distance_acquisition_request(chanend c_uartTX);
	int send_authentication_code(unsigned char pbsPackage[], chanend c_uartTX);
	int send_acquisition_link_code(chanend c_uartTX);
	unsigned int parse_incoming_package(unsigned int timeout, unsigned char pbsPackage[], chanend c_uartRX, uart_rx_client_state &rxState);
	void print_package(int length, unsigned char pbsPackage[]);
	void print_package_mm(int length, unsigned int pbsPackageMm[]);
	unsigned long calculate_crc(unsigned char pbsPackage[], unsigned long len);
	unsigned long reflect (unsigned long crc, int bitnum);
	int decode_PBS(unsigned char stuff[], unsigned char decodedstuff[], int length);
	int encode_PBS(unsigned char uncoded[], unsigned char encoded[], int length);
	int convert_to_mm(unsigned char pPbsDataHex[], unsigned int pPbsDataMm[], int length);
	unsigned int combine_two_chars_to_int(unsigned char MSB, unsigned char LSB);
	int write_buffer_uart(unsigned char chars[], unsigned int dwToWrite, chanend c_uartTX);
	//Laser data publisher (server). Publishes data from the main thread over the c_laser_data cannel.
	void publish_laser_data(unsigned int pPbsDataMm[], unsigned int lengthMm, unsigned int id, chanend c_laser_data);
	//Laser data subscriber (client). Must be called in you application thread.
	void get_laser_data(chanend c_laser_data, laser_data_t &laser_data_packet);

#endif /* _hokuyo_pbs_driver_h_ */
