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
#include <hall_server.h>
#include <pwm_service_inv.h>
#include <comm_loop_server.h>
#include <refclk.h>
#include <qei_server.h>
#include <bldc_motor_config.h>
#include <profile.h>
#include <position_ctrl_server.h>
#include <velocity_ctrl_server.h>
#include <drive_config.h>
#include <profile_control.h>
#include <internal_config.h>

#include <adc_server_ad7949.h>
#include <adc_client_ad7949.h>

#include <symove_conf.h>

#include <laser.h>
#include "hokuyo_pbs_driver.h"
#include "uart_rx.h"
#include "uart_tx.h"


#define MAX_BUFFER_SIZE   1024
#define BAUD_RATE 57600

//out port p_rx = on stdcore[3] : XS1_PORT_1E;
//out port p_tx = on stdcore[3] : XS1_PORT_1A;

/*---------------------------------------------------------------------------
 global variables
 ---------------------------------------------------------------------------*/
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

on stdcore[3]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[3]: clock clk_pwm = XS1_CLKBLK_REF;
/*on stdcore[7]: clock clk_adc_1 = XS1_CLKBLK_1;
on stdcore[7]: clock clk_pwm_1 = XS1_CLKBLK_REF;
on stdcore[11]: clock clk_adc_2 = XS1_CLKBLK_1;
on stdcore[11]: clock clk_pwm_2 = XS1_CLKBLK_REF;
on stdcore[15]: clock clk_adc_3 = XS1_CLKBLK_1;
on stdcore[15]: clock clk_pwm_3 = XS1_CLKBLK_REF;*/


void navigation_routine(chanend c_laser_data, laser_data_t &laser_data_packet);
void init_data(laser_data_t &laser_data_packet);


interface Wheel{
    void velocity(int vel);
};


void setVelocity(int vel, interface Wheel client wheel){
    wheel.velocity(vel);
}

int getVelocity(interface Wheel server wheel){
    select{
        case wheel.velocity(int vel):
                return vel;
    }
}


{int,int,int,int} convertVector2Vel(float vx, float vy, float w){

   // int wh_rpm[4] = {0,0,0,0};

    float wh1_rpm = (-1.0 * (-vx + vy - (L1 + L2) *  w) / WHEEL_RADIUS);
    float wh2_rpm = (-1.0 * (-vx - vy + (L1 + L2) * (-1.0) * w) / WHEEL_RADIUS);
    float wh3_rpm = ((-vx - vy - (L1 + L2) * (-1.0) * w) / WHEEL_RADIUS);
    float wh4_rpm = ((-vx + vy + (L1 + L2) * w) / WHEEL_RADIUS);

    wh1_rpm =  60*wh1_rpm/(2*PI); //RL
    wh2_rpm =  60*wh2_rpm/(2*PI); //FL
    wh3_rpm =  60*wh3_rpm/(2*PI); //RR
    wh4_rpm =  60*wh4_rpm/(2*PI); //FR

        return {(int)wh2_rpm,(int)wh4_rpm,(int)wh3_rpm,(int)wh1_rpm}; // FL, FR, RR, RL
    }

{float,float} convertJoystick2Vector(int xSignal, int ySignal){

    float vx, vy;

    if (xSignal < (X_ZERO - ZERO_ERROR)){
     vx = 1.0 * (X_ZERO - xSignal) / (X_ZERO - X_MIN);
     vx = vx * MAX_LINEAL_VELOCITY;
    }else if(xSignal > (X_ZERO + ZERO_ERROR)){
     vx = - 1.0 * (xSignal - X_ZERO) / (X_MAX - X_ZERO);
     vx = vx * MAX_LINEAL_VELOCITY;
    }else{
        vx = 0;
    }
    if (ySignal < (Y_ZERO - ZERO_ERROR)){
         vy =  1.0 * (Y_ZERO - ySignal) / (Y_ZERO - Y_MIN);
         vy = vy * MAX_LINEAL_VELOCITY;
        }else if(ySignal > (Y_ZERO+ ZERO_ERROR)){
         vy =  - 1.0 * (ySignal - Y_ZERO) / (Y_MAX- Y_ZERO);
         vy = vy * MAX_LINEAL_VELOCITY;
        }else{
            vy = 0;
        }

    return {vx, vy};
}

void leds(chanend ch, out port a, out port b, out port c){
int q;
    while(1){
        select{
                   case ch :> q:

                   a <: q;
                   b <: 1;
                   c <: 1;

                   break;
               }
    }
}


out port p_tx = on stdcore[3] : XS1_PORT_1J;
out port p_rx = on stdcore[3] : XS1_PORT_1B;

int main(void) {

    chan c_adc;

      chan c_qei_p1_0, c_qei_p2_0, c_qei_p3_0, c_qei_p4_0, c_qei_p5_0;                 // qei channels
      chan c_hall_p1_0, c_hall_p2_0, c_hall_p3_0, c_hall_p4_0, c_hall_p5_0;                // hall channels
      chan c_commutation_p1_0, c_commutation_p2_0, c_commutation_p3_0, c_signal_0;    // commutation channels
      chan c_pwm_ctrl_0, c_adctrig_0;                                               // pwm channels
      chan c_velocity_ctrl_0;       // position control channel
      chan c_watchdog_0; // watchdog channel

      chan c_qei_p1_1, c_qei_p2_1, c_qei_p3_1, c_qei_p4_1, c_qei_p5_1;                 // qei channels
      chan c_hall_p1_1, c_hall_p2_1, c_hall_p3_1, c_hall_p4_1, c_hall_p5_1;                // hall channels
      chan c_commutation_p1_1, c_commutation_p2_1, c_commutation_p3_1, c_signal_1;    // commutation channels
      chan c_pwm_ctrl_1, c_adctrig_1;                                               // pwm channels
      chan c_velocity_ctrl_1;       // position control channel
      chan c_watchdog_1; // watchdog channel

      chan c_qei_p1_2, c_qei_p2_2, c_qei_p3_2, c_qei_p4_2, c_qei_p5_2;                 // qei channels
      chan c_hall_p1_2, c_hall_p2_2, c_hall_p3_2, c_hall_p4_2, c_hall_p5_2;                // hall channels
      chan c_commutation_p1_2, c_commutation_p2_2, c_commutation_p3_2, c_signal_2;    // commutation channels
      chan c_pwm_ctrl_2, c_adctrig_2;                                               // pwm channels
      chan c_velocity_ctrl_2;       // position control channel
      chan c_watchdog_2; // watchdog channel

      chan c_qei_p1_3, c_qei_p2_3, c_qei_p3_3, c_qei_p4_3, c_qei_p5_3;                 // qei channels
      chan c_hall_p1_3, c_hall_p2_3, c_hall_p3_3, c_hall_p4_3, c_hall_p5_3;                // hall channels
      chan c_commutation_p1_3, c_commutation_p2_3, c_commutation_p3_3, c_signal_3;    // commutation channels
      chan c_pwm_ctrl_3, c_adctrig_3;                                               // pwm channels
      chan c_velocity_ctrl_3;       // position control channel
      chan c_watchdog_3; // watchdog channel

      //laser channels
      chan c_laser_data;
      chan c_chanTX, c_chanRX;

  /*    interface Initialization initRR;
      interface Initialization initRL;
      interface Initialization initFR;
      interface Initialization initFL;*/

      interface Wheel iWheelFL;
      interface Wheel iWheelFR;
      interface Wheel iWheelRR;
      interface Wheel iWheelRL;

      chan c_leed;
      chan new_data;


	par
	{
		on stdcore[0] :
		{


		  /*  c_leed <: 0;


		        laser_data_t laser_data_packet;
		             init_data(laser_data_packet);
		             //get_laser_data(c_laser_data, laser_data_packet, new_data);
		             		             while(1){
		                 select
		                 {
		                     case c_laser_data :> laser_data_packet.length:
		                         c_laser_data :> laser_data_packet.id;
		                         for (unsigned i = 0; i < NUMBER_OF_RANGE_READINGS; i++)
		                         {
		                             c_laser_data :> laser_data_packet.ranges_mm[i];

		                         }


		                        if(laser_data_packet.ranges_mm[60]==0 | laser_data_packet.ranges_mm[60]>1000){
		                      //      printintln(laser_data_packet.ranges_mm[60]);

		                        }else{
		                                        c_leed <: 1;
		                                     //   printstrln("It works!");
		                        }
		                        break;
		                     default:
		                         break;
		                 }*/

		/*        c_leed <: 0;
	            int q;
	            for(int i= 0;i<9999999;i++){
	                    q++;
                   }



	            while(laser_data_packet.ranges_mm[60]==0 | laser_data_packet.ranges_mm[60]>1000){
	                while(1){
	                    case new_data :>q:

	                }
	              //
	                q = 0;
	                           //get_laser_data(c_laser_data, laser_data_packet);
	                           c_leed <: 1;
	                           for(int i= 0;i<999999;i++){
	                               q++;
	                           }
	                           c_leed <: 0;
	                          printintln(laser_data_packet.ranges_mm[60]);
	                    }

	                    printstrln("It works!");




		    }
*/




          //  printintln(laser_data_packet.ranges_mm[60]);

          //


		  //  navigation_routine(c_laser_data, laser_data_packet);
		}
		on stdcore[1] :
		        {
		        //    leds(c_leed, p_core_leds[0], p_core_leds[1], p_core_leds[2]);
	            }


		on stdcore[3] :
		{
		      unsigned char tx_buffer[MAX_BUFFER_SIZE];
		      unsigned char rx_buffer[MAX_BUFFER_SIZE];

		    par {

		   //     uart_rx(p_rx, rx_buffer, ARRAY_SIZE(rx_buffer), BAUD_RATE, 7, UART_TX_PARITY_NONE, 1, c_chanRX);
		   //     uart_tx(p_tx, tx_buffer, ARRAY_SIZE(tx_buffer), BAUD_RATE, 7, UART_TX_PARITY_NONE, 1, c_chanTX);
		     //   {
		     //   for(int i=0;i<10000000;i++){

		     //   }

		        //p_rx <: 1;
                // p_ifm_ext_d0 <: 1;
                // p_ifm_ext_d1 <: 1;
		        // p_ifm_ext_d2 <: 1;
		         //p_ifm_ext_d3 <: 1;
		      while(1){
		          p_tx <: 1;
		        p_ifm_shared_leds_wden <: 0xa;
		      }
		      while(1);

		     //   run_scanner(c_chanRX, c_chanTX, c_laser_data);
		        }

		      //  {
		            // //1010

		            //while(1);
		       // }
		    }


		}

	/*	 on stdcore[4] :
		                {
		        //              printstrln("core 5\n");
		                }
		        on stdcore[5] :
		                {
		            //          printstrln("core 6\n");
		                        //Test Profile Position function
		                        //profile_velocity_test_1(c_velocity_ctrl[1]);
		                }
		        on stdcore[6] :
		                {

		            //          printstrln("core 7\n");
		                par
		                {
		                        //Velocity ramp generation
		                        {
		                            int vel;

		                  //         initializeWheel(initFR);

		                            while(1){
		                              vel = getVelocity(iWheelFR);
		                             set_profile_velocity(vel, ACCELERATION, DECELERATION, MAX_PROFILE_VELOCITY, c_velocity_ctrl_1);
		                              }
		                         }

		                      //Velocity Control Loop
		                        {
		                            ctrl_par velocity_ctrl_params;
		                            filter_par sensor_filter_params;
		                            hall_par hall_params;
		                            qei_par qei_params;

		                            init_velocity_control_param(velocity_ctrl_params);
		                            init_sensor_filter_param(sensor_filter_params);
		                            init_hall_param(hall_params);
		                            init_qei_param(qei_params);

		                            velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params,
		                                 qei_params, SENSOR_USED, c_hall_p2_1, c_qei_p1_1, c_velocity_ctrl_1, c_commutation_p2_1);
		                        }
		                }
		                }

		        on stdcore[7] :
		                {
		                    //  printstrln("core 8\n");
		                        par
		                            {
		                                //ADC Server for Joystick
		                                adc_ad7949(c_adc, clk_adc_1, p_ifm_adc_sclk_conv_mosib_mosia_node_1, p_ifm_adc_misoa_node_1, p_ifm_adc_misob_node_1);

		                                 //PWM Loop
		                                do_pwm_inv_triggered(c_pwm_ctrl_1, c_adctrig_1, p_ifm_dummy_port_node_1,
		                                        p_ifm_motor_hi_node_1, p_ifm_motor_lo_node_1, clk_pwm_1);

		                                 // Motor Commutation loop
		                                {
		                                    hall_par hall_params;
		                                    qei_par qei_params;
		                                    commutation_par commutation_params;

		                                    init_hall_param(hall_params);
		                                    init_qei_param(qei_params);
		                                    init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);             // initialize commutation parameters
		                                    commutation_sinusoidal(c_hall_p1_1,  c_qei_p2_1, c_signal_1, c_watchdog_1,
		                                            c_commutation_p1_1, c_commutation_p2_1, c_commutation_p3_1,
		                                            c_pwm_ctrl_1, hall_params, qei_params, commutation_params, p_ifm_esf_rstn_pwml_pwmh_node_1, p_ifm_coastn_node_1);
		                                }

		                                // Watchdog Server
		                                run_watchdog(c_watchdog_1, p_ifm_wd_tick_node_1, p_ifm_shared_leds_wden_node_1);

		                                // Hall Server
		                                {
		                                    hall_par hall_params;
		                                    init_hall_param(hall_params);
		                                    run_hall(c_hall_p1_1, c_hall_p2_1, c_hall_p3_1, c_hall_p4_1, c_hall_p5_1, p_ifm_hall_node_1, hall_params); // channel priority 1,2..5
		                                }
		                            }
		                }
		             on stdcore[8] :
		                        {


		                        }
		         on stdcore[9] :
		                        {
		                        //      printstrln("core 10\n");
		                            //  profile_velocity_test_1(c_velocity_ctrl[2]);
		                        }
		         on stdcore[10] :
		                        {

		                //              printstrln("core 11\n");
		                         par {
		                                //Velocity ramp generation
		                                {   int vel;
		                     //              initializeWheel(initRR);

		                                while(1){
		                                    vel = getVelocity(iWheelRR);
		                                    set_profile_velocity(vel, ACCELERATION, DECELERATION, MAX_PROFILE_VELOCITY, c_velocity_ctrl_2);
		                                    }
		                                }

		                                 //Velocity Control Loop
		                                {
		                                    ctrl_par velocity_ctrl_params;
		                                    filter_par sensor_filter_params;
		                                    hall_par hall_params;
		                                    qei_par qei_params;

		                                    init_velocity_control_param(velocity_ctrl_params);
		                                    init_sensor_filter_param(sensor_filter_params);
		                                    init_hall_param(hall_params);
		                                    init_qei_param(qei_params);

		                                    velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params,
		                                         qei_params, SENSOR_USED, c_hall_p2_2, c_qei_p1_2, c_velocity_ctrl_2, c_commutation_p2_2);
		                                }
		                         }
		                        }
		         on stdcore[11] :
		                        {
		                            //  printstrln("core 12\n");
		                                par
		                                {
		                                     //PWM Loop
		                                    do_pwm_inv_triggered(c_pwm_ctrl_2, c_adctrig_2, p_ifm_dummy_port_node_2,
		                                            p_ifm_motor_hi_node_2, p_ifm_motor_lo_node_2, clk_pwm_2);

		                                     // Motor Commutation loop
		                                    {
		                                        hall_par hall_params;
		                                        qei_par qei_params;
		                                        commutation_par commutation_params;

		                                        init_hall_param(hall_params);
		                                        init_qei_param(qei_params);
		                                        init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);             // initialize commutation parameters
		                                        commutation_sinusoidal(c_hall_p1_2,  c_qei_p2_2, c_signal_2, c_watchdog_2,
		                                                c_commutation_p1_2, c_commutation_p2_2, c_commutation_p3_2,
		                                                c_pwm_ctrl_2, hall_params, qei_params, commutation_params, p_ifm_esf_rstn_pwml_pwmh_node_2, p_ifm_coastn_node_2);
		                                    }

		                                    // Watchdog Server
		                                    run_watchdog(c_watchdog_2, p_ifm_wd_tick_node_2, p_ifm_shared_leds_wden_node_2);

		                                    // Hall Server
		                                    {
		                                        hall_par hall_params;
		                                        init_hall_param(hall_params);
		                                        run_hall(c_hall_p1_2, c_hall_p2_2, c_hall_p3_2, c_hall_p4_2, c_hall_p5_2, p_ifm_hall_node_2, hall_params); // channel priority 1,2..5
		                                    }

		                                }
		                        }

		         on stdcore[12] :
		                            {





		                           //         printstrln("core 13\n");
		                            }
		         on stdcore[13] :
		                            {
		                          //           printstrln("core 14\n");



		                            }
		         on stdcore[14] :
		                            {
		                    //                printstrln("core 15\n");

		                     par
		                         {
		                                     {  int vel;
		                       //                 initializeWheel(initRL);

		                                        while(1){
		                                             vel = getVelocity(iWheelRL);
		                                             set_profile_velocity(vel, ACCELERATION, DECELERATION, MAX_PROFILE_VELOCITY, c_velocity_ctrl_3);
		                                         }
		                                     }
		                                    //Velocity Control Loop
		                                    {
		                                        ctrl_par velocity_ctrl_params;
		                                        filter_par sensor_filter_params;
		                                        hall_par hall_params;
		                                        qei_par qei_params;

		                                        init_velocity_control_param(velocity_ctrl_params);
		                                        init_sensor_filter_param(sensor_filter_params);
		                                        init_hall_param(hall_params);
		                                        init_qei_param(qei_params);

		                                        velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params,
		                                             qei_params, SENSOR_USED, c_hall_p2_3, c_qei_p1_3, c_velocity_ctrl_3, c_commutation_p2_3);
		                                    }

		                            }
		                            }
		         on stdcore[15] :
		                            {
		                          //          printstrln("core 16\n");
		                                    par
		                                    {
		                                         //PWM Loop
		                                        do_pwm_inv_triggered(c_pwm_ctrl_3, c_adctrig_3, p_ifm_dummy_port_node_3,
		                                                p_ifm_motor_hi_node_3, p_ifm_motor_lo_node_3, clk_pwm_3);

		                                         // Motor Commutation loop
		                                        {
		                                            hall_par hall_params;
		                                            qei_par qei_params;
		                                            commutation_par commutation_params;

		                                            init_hall_param(hall_params);
		                                            init_qei_param(qei_params);
		                                            init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);             // initialize commutation parameters
		                                            commutation_sinusoidal(c_hall_p1_3,  c_qei_p2_3, c_signal_3, c_watchdog_3,
		                                                    c_commutation_p1_3, c_commutation_p2_3, c_commutation_p3_3,
		                                                    c_pwm_ctrl_3, hall_params, qei_params, commutation_params, p_ifm_esf_rstn_pwml_pwmh_node_3, p_ifm_coastn_node_3);
		                                        }

		                                        // Watchdog Server
		                                        run_watchdog(c_watchdog_3, p_ifm_wd_tick_node_3, p_ifm_shared_leds_wden_node_3);

		                                        // Hall Server
		                                        {
		                                            hall_par hall_params;
		                                            init_hall_param(hall_params);
		                                            run_hall(c_hall_p1_3, c_hall_p2_3, c_hall_p3_3, c_hall_p4_3, c_hall_p5_3, p_ifm_hall_node_3, hall_params); // channel priority 1,2..5
		                                        }
		                                    }
		                            }*/




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
