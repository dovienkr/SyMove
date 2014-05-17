/* app_hokuyo_pbs
 *
 * Test appliction for hokuyo PBS-03JN.
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Author: Viatcheslav Tretyakov <vtretyakov@synapticon.com>
 * Changes:
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <stdio.h>
#include <stdint.h>
#include <ioports.h>

#include <trigonometry.h>
#include <symove_conf.h>

#include <laser.h>
#include "hokuyo_pbs_driver.h"
#include "uart_rx.h"
#include "uart_tx.h"


#define MAX_BUFFER_SIZE   1024
#define BAUD_RATE 57600

#define USEC_STD       100
#define MSEC_STD    100000
#define SEC_STD  100000000

void wait_ms(int milliseconds, int core_id, timer t)
{
    unsigned int time;
    t :> time;


        t when timerafter(time + milliseconds * MSEC_STD) :> time;

    return;
}

/*---------------------------------------------------------------------------
 global variables
 ---------------------------------------------------------------------------*/
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

on stdcore[13]: out port p_core_leds_node_3[3] = { XS1_PORT_1M,   /* Red */
    XS1_PORT_1L,   /* Green */
XS1_PORT_1K }; /* Blue */

buffered in port:1 p_rx = on stdcore[12] : XS1_PORT_1F;
out port p_tx = on stdcore[12] : XS1_PORT_1G;



interface Points {
    void get(unsigned n, unsigned index[105],unsigned length[105]);
};

void navigation_routine(chanend c_laser_data, laser_data_t &laser_data_packet);
void init_data(laser_data_t &laser_data_packet);

//out port p_tx = on stdcore[3] : XS1_PORT_1J;
//out port p_rx = on stdcore[3] : XS1_PORT_1B;

void printPoints(unsigned points[105]){
    for(int i=0;i<105;i+10){
        if(points[i]<500){
            printstr("0 ");
        }else if(points[i]<1000){
            printstr("1 ");
        }else if(points[i]<1500){
            printstr("2 ");
        }else if(points[i]<2000){
            printstr("3 ");
        }else if(points[i]<2500){
            printstr("4 ");
        }else if(points[i]<3000){
            printstr("5 ");
        }else{
            printstr("6 ");
        }
    }
    printstrln("");
}

unsigned isMovementPossible(double vx, double vy, interface Points server dangerousPoints){

    int pointsX[105], pointsY[105];
    select{
        case dangerousPoints.get(unsigned n,unsigned index[105],unsigned length[105]):
             if(n>0){
                unsigned angle;
                unsigned i;
                for(int i=0;i<n;i++){
                  angle  = index[i];
                  pointsX[i] = (int) length[i] * COS[angle];
                  pointsY[i] = (int) length[i] * SIN[angle];
                 //  pointsX[i] = 0;
                 //  pointsY[i] = 0;

                }
               // printintln(pointsY[0]);
             }

                        break;
        }

    return 0;
}

int main(void) {

      //laser channels
      chan c_laser_data;
      chan c_chanTX, c_chanRX;

      interface Points iDangerousPoints;
      interface LaserData iLaserData;
      chan cGetDangerousPoints;

	par
	{
         // on stdcore[10] :

          on stdcore[12] :
               {
                   unsigned char tx_buffer[MAX_BUFFER_SIZE];
                   unsigned char rx_buffer[MAX_BUFFER_SIZE];

                   par{
                       uart_rx(p_rx, rx_buffer, ARRAY_SIZE(rx_buffer), BAUD_RATE, 7, UART_TX_PARITY_NONE, 1, c_chanRX);
                       uart_tx(p_tx, tx_buffer, ARRAY_SIZE(tx_buffer), BAUD_RATE, 7, UART_TX_PARITY_NONE, 1, c_chanTX);
                       {
                          timer t0;
                          wait_ms(700,12,t0);
                           //run_scanner(c_chanRX, c_chanTX, c_laser_data);
                          run_scanner(c_chanRX, c_chanTX, iLaserData);
                       }

                   }
               }

            on stdcore[13] :
               {
                   p_core_leds_node_3[0] <: 1;
                   p_core_leds_node_3[1] <: 0;
                   p_core_leds_node_3[2] <: 1;

                 //  laser_data_t laser_data_packet;
                //   init_data(laser_data_packet);

                   unsigned nOfDangerousPoints= 0;
                   unsigned dangerousPoints[105];
                   unsigned dangerousLengths[105];
                   unsigned get;

                   dangerousLengths[0] = 0;


                par
                {
                       {
                           while(1){
                               select
                               {
                                  case iLaserData.get(unsigned length[]):
                                        nOfDangerousPoints = 0;
                                        for(unsigned i=5;i<110;i++){ //firsts and last samples are zeros
                                            if(length[i]!=0 & length[i]<600){
                                                dangerousPoints[nOfDangerousPoints] = i-5; //offset, first samples are zeros
                                                dangerousLengths[nOfDangerousPoints++] = length[i];
                                            }
                                           // copyTest[i-5]=length[i];
                                        }
                                       // printPoints(copyTest);

                                       // printintln(length[60]);

                                        break;

                                   case cGetDangerousPoints :> get:

                                       iDangerousPoints.get(nOfDangerousPoints,dangerousPoints,dangerousLengths);
                                       break;
                               }

                             //  printintln(laser_data_packet.ranges_mm[60]);


                           }

                          // p_core_leds_node_3[0] <: 1;
                          // p_core_leds_node_3[1] <: 0;
                          // p_core_leds_node_3[2] <: 1;
                       }
                       {
                                     int vx= 0;
                                     int vy= 100;

                                     timer t0;

                                     int pointsX[105], pointsY[105];
                                     unsigned crashFLAG;
                                     unsigned angle;
                                     unsigned i;

                                    wait_ms(1500,10,t0);

                                    while(1)
                                    {
                                          wait_ms(300,10,t0);
                                          crashFLAG = 0;

                                          cGetDangerousPoints <: 1;

                                             select{
                                                 case iDangerousPoints.get(unsigned n,unsigned index[105],unsigned length[105]):
                                                      if(n>0){

                                                         for(unsigned i=0;i<n;i++){
                                                           angle  = index[i];
                                                           pointsX[i] = ((int) length[i] * COS[angle])-vx;
                                                           if(pointsX[i]<0)
                                                               pointsX[i] *= -1;

                                                           pointsY[i] = ((int) length[i] * SIN[angle])-vy;
                                                           if(pointsY[i]<0)
                                                               pointsY[i] *= -1;

                                                           if((pointsX[i] + pointsY[i])<500){
                                                              // printstrln("CRASH\n");
                                                               crashFLAG = 1;
                                                               break;
                                                           }

                                                          //  pointsX[i] = 0;
                                                          //  pointsY[i] = 0;

                                                         }
                                                      }

                                                         if(crashFLAG){
                                                             p_core_leds_node_3[0] <: 0;
                                                             p_core_leds_node_3[1] <: 1;
                                                             p_core_leds_node_3[2] <: 1;
                                                         }else{
                                                            p_core_leds_node_3[0] <: 1;
                                                            p_core_leds_node_3[1] <: 0;
                                                            p_core_leds_node_3[2] <: 1;
                                                         }

                                                        // printintln(pointsY[0]);


                                                                 break;
                                                 }
                                    }

                                 }

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
