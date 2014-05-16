/* hokuyo_pbs_driver
 *
 * Hokuyo PBS-03JN driver
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Author: Viatcheslav Tretyakov <vtretyakov@synapticon.com>
 * Changes:
 */

#include "hokuyo_pbs_driver.h"
#include "uart_tx.h"
#include <xs1.h>
#include <print.h>

#define NUMBER_OF_RANGE_READINGS 121
#define MAX_RANGE_MM 3500

/// Function for acquiring distance data from PBS
/** When distance data is needed this function is invoked. Note that the link
to the PBS must be previously set up*/
int send_distance_acquisition_request(chanend c_uartTX)
{
	// This is the distance acquisition command. Always looks like this
	static unsigned char distanceAcq[8] = {0x02,0x48,0x46,0x46,0x28,0x38,0x40,0x03};

	if(write_buffer_uart(distanceAcq, 8, c_uartTX)==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int send_authentication_code(unsigned char linkAuthCode[], chanend c_uartTX)
{
	int status=1;
	int i=0;
	unsigned char temp2;
	unsigned char temp1;
	unsigned long checksum, checksum2;
	// temp variables
	unsigned char linkAuthSend[15], linkAuthSend2[17], linkAuthUn[12], linkAuthEn[10], lenAuth;
	unsigned char linkAuthCodeCRC[2], linkAuthCodeEn[8];
	unsigned char linkAuthCodeClose[8], linkAuthCodeCloseSend[15],linkAuthCodeCloseSend2[17];

	// Calculating the cheksum needed to make the link auth code
	checksum =  calculate_crc(linkAuthCode, 8);

	temp1 = checksum & 0x00FF;
	temp2 = (checksum & 0xFF00) >> 8;

	// Calculate the new checksum with the link auth code in it
	linkAuthCodeEn[0] = 0xA0;
	linkAuthCodeEn[1] = 0x5A;
	linkAuthCodeEn[2] = 0x01;
	linkAuthCodeEn[3] = temp1;
	linkAuthCodeEn[4] = temp2;

	// the following is to test if the connection can be closed correctly
	linkAuthCodeClose[0] = 0xA0;
	linkAuthCodeClose[1] = 0x5A;
	linkAuthCodeClose[2] = 0x00;
	linkAuthCodeClose[3] = temp1;
	linkAuthCodeClose[4] = temp2;

	checksum = calculate_crc(linkAuthCodeEn,5);
	checksum2 = calculate_crc(linkAuthCodeClose,5);

	// This HAS to be reversed
	temp1 = checksum & 0x00FF;
	temp2 = (checksum & 0xFF00) >> 8;
	// Plug it in the last two bytes in reverse order
	linkAuthCodeEn[5] = temp1;
	linkAuthCodeEn[6] = temp2;

	// This is to calculate the close command
	temp1 = checksum2 & 0x00FF;
	temp2 = (checksum2 & 0xFF00) >> 8;
	linkAuthCodeClose[5] = temp1;
	linkAuthCodeClose[6] = temp2;

	lenAuth = encode_PBS(linkAuthCodeEn, linkAuthSend, 8);
	lenAuth = encode_PBS(linkAuthCodeClose, linkAuthCodeCloseSend, 8);

	linkAuthSend2[0] = 0x02;
	for(i=0;i<=lenAuth;i++)
	{
		linkAuthSend2[i+1] = linkAuthSend[i];
	}
	linkAuthSend2[i] = 0x03;

	linkAuthCodeCloseSend2[0]=0x02;
	for(i=0;i<=lenAuth;i++)
	{
		linkAuthCodeCloseSend2[i+1] = linkAuthCodeCloseSend[i];
	}
	linkAuthCodeCloseSend2[i] = 0x03;
	write_buffer_uart(linkAuthSend2, 12, c_uartTX);

	return status;
}

/// Function for sending acquisition of link code
/** When sent to the PBS the PBS responds with the link code */
int send_acquisition_link_code(chanend c_uartTX)
{
	// First to to send the first Acquisition of LINK auth code, this is always fixed
	// The following line is link Acq command that the pc has to send to the PBS.
  	static unsigned char linkAcqEn[8] = {0x02,0x48,0x26,0x44,0x58,0x34,0x30,0x03};
	write_buffer_uart(linkAcqEn, 8, c_uartTX);
	return 1;
}

/// Function for closing PBS
/** Terminates the link between PC and PBS*/
/* ToDo - implement the method
void close_PBS(chanend c_uartTX)
{
    printstrln("sending close command");
    write_buffer_uart(linkAuthCodeCloseSend2, 12, c_uartTX);
    printstrln("close command sent");
}
*/

unsigned int parse_incoming_package(unsigned int timeout, unsigned char pbsPackage[], chanend c_uartRX, uart_rx_client_state &rxState)
{
	int len = 0; //Number of bytes returned by decodePBS (defines the length of actual data in pbsPackage)
//	int lenMm = 0;
	int cnt = 0;//Number of bytes received (6-bit format)
	int IncommingPackage = 0;
//	unsigned int StartTime = get_tick_count();//Returns the time in milliseconds since last reboot (max 49 days)
	//Holder for  incoming byte from the serial receive routine
	unsigned char chPBS;
	timer t;
	unsigned time = 0;
	unsigned timer_is_enabled = 0;
	unsigned no_timeout = 1;
	unsigned delay = timeout * 10000;
	unsigned char pbsBuf[400];


	while(no_timeout)
	{
		//read one byte
		select
		{		//wait till something is coming and then read one byte
			case uart_rx_get_byte_byref(c_uartRX, rxState, chPBS):
				//start timer
				t :> time;
				timer_is_enabled = 1;
				//if data is comming
			//	printintln(rxState.received_bytes);
				if(rxState.received_bytes)
				{
					// 0x02 is the "new package" keyword
					if(chPBS==0x02)
					{
				//		printstrln("New package");
						//cnt = 0;
						rxState.received_bytes = 0;
						IncommingPackage = 1;//true
					}
					//0x03 is the "end of package keyword
					else if((chPBS==0x03) && (IncommingPackage == 1))
					{
				//		printstr("Decoding package of size ");printintln(rxState.received_bytes - 1);
				//		print_package(rxState.received_bytes - 1, pbsBuf);
						len = decode_PBS(pbsBuf, pbsPackage, rxState.received_bytes - 1);

						return len;
					}
					else
					{
						if (IncommingPackage == 1)
						{
							pbsBuf[rxState.received_bytes - 1] = chPBS;
							// Sorting of packages. We are only interested in link auth code, link auth, and Dist data
							if ( (rxState.received_bytes == 1) && !(chPBS==0x48) )
							{
								IncommingPackage = 0;
							}
						}
					}
				}
				else
				{
					return 0;
				}
				break;
			default:
				break;

		}//eof select1

		if(timer_is_enabled)
		{
			select
			{
				case t when timerafter(time+delay) :> void :
					for (int i=0; i < 100000; i++){

					}
//					printstr("timeout, buf: ");printintln(rxState.received_bytes);
					timer_is_enabled = 0;
					rxState.received_bytes = 0;
					no_timeout = 0;
					break;
				default:
					break;
			}//eof select2
		}//eif of

	}//eof while
	return 0;
}

/// Function for printing the decoded output of a package
/** Prints the output to the screen for debugging purposes */
void print_package(int length, unsigned char pbsPackage[])
{
	int i;

	printstrln(" Package: ");

	// Print decoded data to screen
    for(i=0; i<length; i++)
	{
	    printstr(" 0x"); printhex(pbsPackage[i]);
	}

   printstrln("\nTest length pbs_package: "); printint(length);printstrln("\n");
   printstrln("-----------------------------------------");printstrln("");

}

/// Function for printing the decoded output of a package in milimeters
/**	Prints the output to the screen for debugging purposes*/
void print_package_mm(int length, unsigned int pbsPackageMm[])
{
	int i;

	printstrln("Package in mm: ");

    // Print data in mm to screen*/
    for(i=0;i<length;i++){
	    	printstr("MM[");printint(i);printstr("]: ");printintln(pbsPackageMm[i]);
    }
printstrln("-----------------------------------------\n");

}

/// Function for calculating crc
/** Used for authentication code generation */
unsigned long calculate_crc(unsigned char pbsPackage[], unsigned long len)
{
	// CRC parameters (default values are for CRC-32):

	const int order = 16;
	const unsigned long polynom = 0x1021;
	const unsigned long crcxor = 0x0000;
	const int refin = 1;
	const int refout = 1;
	unsigned long crcinit_direct = 0;


	unsigned long crcmask;
	unsigned long crchighbit;
	unsigned long i, j, c, bit;
	unsigned long crc = crcinit_direct;

	crcmask = ((((unsigned long)1<<(order-1))-1)<<1)|1;
	crchighbit = (unsigned long)1<<(order-1);

	for (i=0; i<len; i++) {

		c = (unsigned long)pbsPackage[i];
		if (refin) c = reflect(c, 8);

		for (j=0x80; j; j>>=1) {

			bit = crc & crchighbit;
			crc<<= 1;
			if (c & j) bit^= crchighbit;
			if (bit) crc^= polynom;
		}
	}

	if (refout) crc=reflect(crc, order);
	crc^= crcxor;
	crc&= crcmask;

	return(crc);
}

/// Function for closing reflecting the lower 'bitnum' bits of crc
/** Used for authentication code generation */
unsigned long reflect (unsigned long crc, int bitnum)
{
    unsigned long i, j=1, crcout=0;

	for (i=(unsigned long)1<<(bitnum-1); i; i>>=1) {
	    if (crc & i) crcout|=j;
	    j<<= 1;
	}
	return (crcout);
}

/// Function for decoding PBS data
/** Decodes the PBS package according to the PBS/PC communication protocol */
int decode_PBS(unsigned char stuff[], unsigned char decodedstuff[], int length)
{
	int i,j,k;
	j=0;
	k=0;

	// First we clean up the list by subtracting the 0x20
	for (i=0; i<length; i++){

	if (((stuff[i] & 0x40) >> 6) == 1){
		stuff[i] = (stuff[i] & 0xBF);
		stuff[i] = (stuff[i] | 0x20);
		stuff[i] = stuff[i] << 2;
	}
	else{
		stuff[i] = stuff[i] & 0xDF;
		stuff[i] = stuff[i] << 2;
	}
	}


// Then we decode the list and put everything together
	for (i=0; i<=length; i++){

	if (j == 0){
		if(i==length-1){
		decodedstuff[k] = stuff[i] +((stuff[i+1] & 0xC0 >> 6));
		break;
		}

		//printf("the stuff for the first part %x \n",stuff[i]);
		decodedstuff[k] = stuff[i] + ((stuff[i+1] & 0xC0) >> 6);
		//printf("the decoded stuff for the first part %x \n",decodedstuff[k]);
		k=k+1;
	}

	if (j == 1){

		if(i==length-1){
		decodedstuff[k] = (((stuff[i] & 0x3C)) << 2)  + ((stuff[i+1] & 0xF0) >> 4);
		break;
		}

		decodedstuff[k] = (((stuff[i] & 0x3C)) << 2)  + ((stuff[i+1] & 0xF0) >> 4);
		//printf("the decoded stuff for the second part %x \n",decodedstuff[k]);
		k=k+1;
	}

	if (j == 2){
		if(i==length-1){
		decodedstuff[k] = ((stuff[i] & 0x0C) << 4) + ((stuff[i+1] & 0xFC) >> 2);
		break;
		}

		decodedstuff[k] = ((stuff[i] & 0x0C) << 4) + ((stuff[i+1] & 0xFC) >> 2);
		//printf("the decoded stuff for the third part %x \n",decodedstuff[k]);
		k=k+1;
	}

	j=j+1;

	if (j == 4){
		j = 0;
	}

	if(j ==3){}
	}

// Return the length of the decoded list
	return k;

}

/// Function for encoding data to be send to PBS
/** Encodes the PBS package according to the PBS/PC communication protocol */
int encode_PBS(unsigned char uncoded[], unsigned char encoded[], int length)
{
    int i,j,k;

    j=0;
    k=0;

    for (i=0; i <= length;i++){

		if (j==0){

			if(i==length){
				break;
			}
			encoded[k] = ((uncoded[i] & 0xFC) >> 2) + 0x20;
			k=k+1;

		}

		if (j==1){

			if(i==length){
				encoded[k] = ((uncoded[i-1] << 4) & 0x30) + 0x20;
				break;
			}
			encoded[k] = (((uncoded[i-1] & 0x03) << 4) + ((uncoded[i] & 0xF0) >> 4)) + 0x20;
			k=k+1;
		}

		if (j==2){

			if(i==length){
				encoded[k] = ((uncoded[i-1] << 2) & 0x3c) + 0x20;
				break;
			}
			encoded[k] = (((uncoded[i-1] & 0x0F) << 2) + ((uncoded[i] & 0xC0) >> 6)) + 0x20;
			k=k+1;
		}

		if (j==3){
			if(i==length){
				encoded[k] = ((uncoded[i-1] & 0x3F) + 0x20);
				break;
			}
			encoded[k] = (uncoded[i-1] & 0x3F) + 0x20;
			k=k+1;
		}

		j=j+1;

		if (j==4){
			j=0;
			i=i-1;
		}

    }
    return k;
}

/// Function for converting PBS data from hex values to mm
int convert_to_mm(unsigned char pPbsDataHex[], unsigned int pPbsDataMm[], int length)
{
	int i = 0, k=1, LengthConv =0, OffsetInit = 2, OffsetEnd = 4;

	for(i=OffsetInit+1 ; i<length-OffsetEnd ; i = i+2){
	       pPbsDataMm[i-k-OffsetInit] = ((unsigned int)combine_two_chars_to_int(pPbsDataHex[i],pPbsDataHex[i-1]));
	       LengthConv = i-k;
	       k++;
	}

//	printstr("Data converted to mm. Length of new package is ");
//	printint(LengthConv);printstrln(" shorts"); //printstrln(" shorts");
	return (LengthConv);
}

/// Function for combining two chars to an integer
unsigned int combine_two_chars_to_int(unsigned char MSB, unsigned char LSB)
{
	return (MSB<<8) | LSB;
}



/// Function for writing data to the uart tx
int write_buffer_uart(unsigned char chars[], unsigned int dwToWrite, chanend c_uartTX) //transmit data on Uart TX terminal
{
	for (int i = 0; i < dwToWrite; i++)
	{
		uart_tx_send_byte(c_uartTX, chars[i]); //send data to uart byte by byte
	}
	return 1;
}



// The main driver thread
void run_scanner(chanend c_uartRX, chanend c_uartTX, chanend c_laser_data)
{


	enum state_t {AcqLinkCode, LinkAuth, DataAcq, Error};
	enum state_t state = AcqLinkCode;

	//Number of bytes received over UART
	uart_rx_client_state rxState;

	//Sampletime
	timer t, t2;
	unsigned time = 0;
	unsigned time2 = 0;
	unsigned int sampletime = 30000000; //300 milliseconds

	// The time when 'Link auth request' was sent
	unsigned int LinkAuthSendTime = 0;

	// Timeout when listening for 'Acq. of link auth. code'-reply
	unsigned int AcqLinkCodeTimeout = 300; //milliseconds

	// Timeout when listening for 'Distance Data Acq.'-reply (which is he meas. data)
	unsigned int SendAcqDataTimeout = 300; //milliseconds

	//Time between the state is changed to "Send Link auth" in the state "Request Distance Data"
	unsigned int LinkAuthSendTimeout = 2000; //milliseconds.

	// Max number of retries when sending 'Link Auth.' request
	static int LinkAuthRetriesMAX = 4;

	// Counter for determining the maximal number of times in a row a wrong Data package can be returned before it is considered a failure
	static int wrongDataPackageCnt = 0;

	// Counter for determining the maximal number of times a link auth can be sent before the state is changed back to AcqLinkCode
	static int LinkAuthRetries = 0;

	// Number of times a request for Acq. Link Code has be sent
	static int AcqLinkCodeCnt = 0;

	// Counter for determining the number of wrong distance data packages.
	static int ReceiveDataFailCnt = 0;

	//The state must not be changed if af Distance Data request has not been relied
	int DistDataReceived = 0;

    unsigned int length, lengthMm;
    unsigned char pbsPackage[400];

    // Decoded data from PBS converted to mm
    unsigned int pbsPackageMm[1024];

	//Holder for Link Auth. Code received from PBS
	unsigned char linkAuthCode[9];


	//Condition for accepting package as data
	#define DATAPACKAGE_RECEIVED ((pbsPackage[0] == 0xA2) && (pbsPackage[1] == 0x69))

	//Condition for accepting package as link auth reply
	#define LINKAUTHREPLY_RECEIVED ((pbsPackage[0] == 0xA0) && (pbsPackage[1] == 0x5A))

	//Condition for link status: connected
	#define IS_CONNECTED pbsPackage[2] == 1

	//Condition for accepting package as Link Auth code
	#define AUTHCODE_RECEIVED ((pbsPackage[0] == 0xA0) && (pbsPackage[1] == 0x69))


	//A unique, increasing, ID for the scan
	unsigned int scan_id = 0;

	//UART initialization
	uart_rx_init(c_uartRX, rxState);
	uart_rx_set_baud_rate(c_uartRX, rxState, BAUD_RATE);
	uart_tx_set_baud_rate(c_uartTX, BAUD_RATE);


  //! The main loop
	while(1)
	{

			switch(state)
			{

			case AcqLinkCode:
				//Send a request for the Link Auth. code
				if(send_acquisition_link_code(c_uartTX))
				{
				//	printstr("Acquisition of LINK auth code transmitted, ");printintln(AcqLinkCodeCnt);
					AcqLinkCodeCnt++;
					//Now we must receive the Auth code. First we listen for new packages until timeout,
					//then we check if the package received really is the auth code

					length = parse_incoming_package(AcqLinkCodeTimeout, pbsPackage, c_uartRX, rxState);

					if(length)
					{
						//Check if received is auth code
						if(AUTHCODE_RECEIVED)
						{
							//printstrln(": package is link Acquisition Authentication Code, which is: ");
							//print_package(length, pbsPackage);
							//Save the Link Auth code

							for(int i=2 ; i<=10 ; i++)
							{
								linkAuthCode[i-2] = pbsPackage[i];
							}

							// Shift state to be able to send a link Acquisition Authentication Code reply

							state = LinkAuth;
							AcqLinkCodeCnt=0;

						}
						else
						{
//							printstr("Package was not a auth. code, but: ");
//							printhexln(pbsPackage[0]);
							//print_package(length, pbsPackage);
						}

					}
					else
					{
//						printstr("A valid package was not returned before the timeout ");printint(AcqLinkCodeTimeout);printstrln(" ms");
					}
				}
				else
				{
//					printstrln("Error when sending Acquisition of LINK auth code");
				}

				break;

			case LinkAuth:
				// Checking if Authentication Code reply has been sent
//				printstr("Sending Authentification Code reply ");printint(LinkAuthRetries);printstr("/");printintln(LinkAuthRetriesMAX);
				// The Link Authentication Code reply is sent

				if(send_authentication_code(linkAuthCode, c_uartTX))
				{
//					printstrln("Link Authentication Code reply was sent");
				}
				else
				{
//					printstrln("Something went wrong when sending link Authentication Code reply");
				}
				// Check if link is established until timeout occurs
				length = parse_incoming_package(AcqLinkCodeTimeout, pbsPackage, c_uartRX, rxState);
				if(length)
				{
					//Check if received is "link established" package
					if(LINKAUTHREPLY_RECEIVED)
					{
						t2 :> time2;
						LinkAuthSendTime = time2/100000;//Used to calc time since last Link Auth (must be sent at least each third seconed)
						// Check if the package contains connected or disconnected
						if(IS_CONNECTED)
						{
							LinkAuthRetries = 1;
							//We can now use distance data acquisition
							state = DataAcq;
							wrongDataPackageCnt = 0;
//							printstrln("Link connected");

						}
						else
						{
//							printstrln("package is 'Link level': DISCONNECTED");
							state = AcqLinkCode;
							LinkAuthRetries = 1;
						}

					}
					else
					{
//						printstr("Package was NOT 'link established', but ");
						// BEGIN HACK: Sometimes we receive data even when we should receive Link Auth...
						//we don't want to throw good data away...
						if(DATAPACKAGE_RECEIVED)
						{
//							printstrln("data. We publish it anyway...");
							DistDataReceived = 1;
//							printstr(" Data: ");

							lengthMm = convert_to_mm(pbsPackage, pbsPackageMm, length);
//							print_package_mm(lengthMm, pbsPackageMm);

							//publish the data
							if (lengthMm == NUMBER_OF_RANGE_READINGS)
							{
								publish_laser_data(pbsPackageMm, lengthMm, scan_id, c_laser_data);
							}

							scan_id++;
							//prevent data overflow
							if (scan_id > 4294967294)
								scan_id = 0;

//							printstr("Data package with an id ");printint(scan_id);printstrln(" is published");

							wrongDataPackageCnt = 0;
							// State is maintained

						}
						//END OF HACK
						else//we have received some strange package...
						{
//							printstrln("we have received some strange package...");
							//print_package(length, pbsPackage);
						}

						if(LinkAuthRetries==LinkAuthRetriesMAX)
						{
//							printstrln("MAX number of retries. Trying a request for the Link Auth. code");
							LinkAuthRetries = 1;
							state = AcqLinkCode;
						}
						else
						{
							if (state==LinkAuth)
							{
//								printstrln("Trying to send a Link Auth req again...");
								LinkAuthRetries++;
							}
							else
							{
								LinkAuthRetries = 0;
							}

						}

					}
				}
				break;

			case DataAcq: //State is "data acquisition". Edges to "Link Authorisation"
				//sampletime implementation
				t :> time;
				t when timerafter(time+sampletime) :> void;

				DistDataReceived = 0;
				//Now we send a request for distance data
				send_distance_acquisition_request(c_uartTX);

				// Check if distance data is received until timeout occurs
				length = parse_incoming_package(SendAcqDataTimeout, pbsPackage, c_uartRX, rxState);

				if(length)
				{
					//Check if received is data package
					if(DATAPACKAGE_RECEIVED)
					{
						DistDataReceived = 1;
//						printstr(" Data: ");
						lengthMm = convert_to_mm(pbsPackage, pbsPackageMm, length);
						//	print_package_mm(lengthMm, pbsPackageMm);

						//publish the data
						if (lengthMm == NUMBER_OF_RANGE_READINGS)
						{
							publish_laser_data(pbsPackageMm, lengthMm, scan_id, c_laser_data);
						}

//						printstr("Data package with an id ");printint(scan_id);printstrln(" is published");
						scan_id++;
						//prevent data overflow
						if (scan_id > 4294967294)
							scan_id = 0;

						wrongDataPackageCnt = 0;
						// State is maintained

					}
					else
					{
//						printstrln("package was NOT data. Trying again");
						//print_package(length, pbsPackage);
						if(wrongDataPackageCnt == 5)
						{
//							printstrln("To many wrong Data acquisition Retries in a row - Trying Link Auth.");
							state = LinkAuth;
						}
						wrongDataPackageCnt++;

					}
				}
				// If to many wrong packages has been received in a row, we change the state to Link Auth
				else
				{
					ReceiveDataFailCnt++;
					if(ReceiveDataFailCnt==3)
					{
						state = LinkAuth;
						ReceiveDataFailCnt = 0;
					}
				}

				// Check time since 'Link auth' package was sent
				t2 :> time2;
//				printstr("time since link auth was sent ");printuint(time2/100000 - LinkAuthSendTime);printstrln(" ms");
				if(((time2/100000 - LinkAuthSendTime) > LinkAuthSendTimeout) && (DistDataReceived == 1))
				{
					state = LinkAuth;
				}
				break;

			case Error:
//				printstrln("error!!!");
				break;
			}

	}
}

/// Function to send the laser data to a concumer over a channel
void publish_laser_data(unsigned int pbsPackageMm[], unsigned int lengthMm, unsigned int id, chanend c_laser_data)
{
	c_laser_data <: lengthMm;
	c_laser_data <: id;
	for(unsigned int i=0; i < lengthMm; i++)
	{
		if(pbsPackageMm[i] < MAX_RANGE_MM)
		{
			c_laser_data <: pbsPackageMm[i];
		}
		else
		{
			c_laser_data <: MAX_RANGE_MM;
		}

	}

}

/// Laser data concumer
void get_laser_data(chanend c_laser_data, laser_data_t &laser_data_packet)
{
	select
	{
		case c_laser_data :> laser_data_packet.length:
			c_laser_data :> laser_data_packet.id;
			for (unsigned i = 0; i < NUMBER_OF_RANGE_READINGS; i++)
			{
				c_laser_data :> laser_data_packet.ranges_mm[i];

			}
			break;
		default:
			break;
	}

}
