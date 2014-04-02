
/**
 *
 * \file test_position-ctrl.xc
 *
 * \brief Main project file
 *  Test illustrates usage of profile position control
 *
 *
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 * Author: Pavan Kanajar <pkanajar@synapticon.com> & Martin Schwarz <mschwarz@synapticon.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
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

#include <symove_conf.h>

on stdcore[3]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[3]: clock clk_pwm = XS1_CLKBLK_REF;
on stdcore[7]: clock clk_adc_1 = XS1_CLKBLK_1;
on stdcore[7]: clock clk_pwm_1 = XS1_CLKBLK_REF;
on stdcore[11]: clock clk_adc_2 = XS1_CLKBLK_1;
on stdcore[11]: clock clk_pwm_2 = XS1_CLKBLK_REF;
on stdcore[15]: clock clk_adc_3 = XS1_CLKBLK_1;
on stdcore[15]: clock clk_pwm_3 = XS1_CLKBLK_REF;

interface Initialization {
    void init(int i);
};

interface Wheel{
    void velocity(int vel);
};

void init(interface Initialization server initW){
       select {
       case initW.init(int i):
         printstrln("Wheel Initialized");
         break;
     }
}

void initializeWheel(interface Initialization client i){
    i.init(1);
}

void setVelocity(int vel, interface Wheel client wheel){
    wheel.velocity(vel);
}

int getVelocity(interface Wheel server wheel){
    select{
        case wheel.velocity(int vel):
                return vel;
    }
}

{int,int,int,int} convertInst2Vel(int vx, int vy, int w){

    int v[4] = {0,0,0,0};
    float l1 = L1 /100;
    float l2 = L1 /100;
    float wheel_radius = WHEEL_RADIUS /100;

    v[0] = (int) (-1.0 * (-vx + vy - (l1 + l2) *  w) / wheel_radius);
    v[1] = (int)(-1.0 * (-vx - vy + (l1 + l2) * (-1.0) * w) / wheel_radius);
    v[2] = (int)((-vx - vy - (l1 + l2) * (-1.0) * w) / wheel_radius);
    v[3] = (int)((-vx + vy + (l1 + l2) * w) / wheel_radius);

        return {v[0],v[1],v[2],v[3]};
    }



int main(void)
{
	// Motor control channels
//	chan c_qei_p1[N_NODES], c_qei_p2[N_NODES], c_qei_p3[N_NODES], c_qei_p4[N_NODES], c_qei_p5[N_NODES];					// qei channels
//	chan c_hall_p1[N_NODES], c_hall_p2[N_NODES], c_hall_p3[N_NODES], c_hall_p4[N_NODES], c_hall_p5[N_NODES];				// hall channels
//	chan c_commutation_p1[N_NODES], c_commutation_p2[N_NODES], c_commutation_p3[N_NODES], c_signal[N_NODES];	// commutation channels
//	chan c_pwm_ctrl[N_NODES], c_adctrig[N_NODES];												// pwm channels
//	chan c_velocity_ctrl[N_NODES];       // position control channel
//	chan c_watchdog[N_NODES]; // watchdog channel

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

	interface Initialization initRR;
	interface Initialization initRL;
	interface Initialization initFR;
	interface Initialization initFL;

	interface Wheel iWheelFL;
	interface Wheel iWheelFR;
	interface Wheel iWheelRR;
	interface Wheel iWheelRL;

	par
	{
		on stdcore[0] :
				{
		             //   timer t0,t1,t2,t3;

		                timer t;

		                init(initRR);
		                init(initFL);
		                init(initFR);
		                init(initRL);

		                /* while(1){
		                setVelocity(2000,iWheelFR);

		                wait_ms(2000, 0, t);

		                setVelocity(0,iWheelFR);
		                setVelocity(2000,iWheelRL);

		                wait_ms(2000, 0, t);

		                setVelocity(0,iWheelRL);
		                setVelocity(2000,iWheelRR);

		                wait_ms(2000, 0, t);

		                setVelocity(0,iWheelRR); */

		                }
				}
		on stdcore[1] :
				{

				}
		on stdcore[2] :
				{

	//					printstrln("core 3\n");

		        par {

		                //Velocity ramp generation
		                {

		                    initializeWheel(initFL);
		                }

	                    /*Velocity Control Loop*/
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
                                 qei_params, SENSOR_USED, c_hall_p2_0, c_qei_p1_0, c_velocity_ctrl_0, c_commutation_p2_0);
                        }

		        }

				}
		/***********************************************************
		 * IFM_CORE
		 ***********************************************************/
		on stdcore[3] :
				{
			//			printstrln("core 4\n");

						par
								{
						        // PWM Loop
						        do_pwm_inv_triggered(c_pwm_ctrl_0, c_adctrig_0, p_ifm_dummy_port_node_0,
								p_ifm_motor_hi_node_0, p_ifm_motor_lo_node_0, clk_pwm);

						// Motor Commutation loop
						{
							hall_par hall_params;
							qei_par qei_params;
							commutation_par commutation_params;
							init_hall_param(hall_params);
							init_qei_param(qei_params);
							init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); 			// initialize commutation parameters
							commutation_sinusoidal(c_hall_p1_0,  c_qei_p2_0, c_signal_0, c_watchdog_0,
									c_commutation_p1_0, c_commutation_p2_0, c_commutation_p3_0,
									c_pwm_ctrl_0, hall_params, qei_params, commutation_params, p_ifm_esf_rstn_pwml_pwmh_node_0, p_ifm_coastn_node_0);
						}

						// Watchdog Server
						run_watchdog(c_watchdog_0, p_ifm_wd_tick_node_0, p_ifm_shared_leds_wden_node_0);

						// Hall Server
						{
							hall_par hall_params;
							init_hall_param(hall_params);
							run_hall(c_hall_p1_0, c_hall_p2_0, c_hall_p3_0, c_hall_p4_0, c_hall_p5_0, p_ifm_hall_node_0, hall_params); // channel priority 1,2..5
						}

						 //QEI Server
						{
							qei_par qei_params;
							init_qei_param(qei_params);
							run_qei(c_qei_p1_0, c_qei_p2_0, c_qei_p3_0, c_qei_p4_0, c_qei_p5_0, p_ifm_encoder_node_0, qei_params);  	// channel priority 1,2..5
						}
								}
				}

		 on stdcore[4] :
				{
		//				printstrln("core 5\n");
				}
		on stdcore[5] :
				{
			//			printstrln("core 6\n");
						//Test Profile Position function
						//profile_velocity_test_1(c_velocity_ctrl[1]);
				}
		on stdcore[6] :
				{

			//			printstrln("core 7\n");
		        par
		        {
                        //Velocity ramp generation
                        {
                            int vel;

                           initializeWheel(initFR);

                            while(1){
                              vel = getVelocity(iWheelFR);
                             set_profile_velocity(vel, 2000, 1000, MAX_PROFILE_VELOCITY, c_velocity_ctrl_1);
                              }
                         }

                      /*Velocity Control Loop*/
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
					//	printstrln("core 8\n");
						par
							{
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
									init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); 			// initialize commutation parameters
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

								// QEI Server
								{
									qei_par qei_params;
									init_qei_param(qei_params);
									run_qei(c_qei_p1_1, c_qei_p2_1, c_qei_p3_1, c_qei_p4_1, c_qei_p5_1, p_ifm_encoder_node_1, qei_params);  	// channel priority 1,2..5
								}
							}
				}
		 on stdcore[8] :
						{
						//		printstrln("core 9\n");




						}
		 on stdcore[9] :
						{
						//		printstrln("core 10\n");
							//	profile_velocity_test_1(c_velocity_ctrl[2]);
						}
		 on stdcore[10] :
						{

				//				printstrln("core 11\n");
		                 par {
		                        //Velocity ramp generation
		                        {   int vel;
		                            initializeWheel(initRR);

		                        while(1){
                                    vel = getVelocity(iWheelRR);
                                    set_profile_velocity(vel, 2000, 1000, MAX_PROFILE_VELOCITY, c_velocity_ctrl_2);
		                            }
		                        }

                                 /*Velocity Control Loop*/
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
							//	printstrln("core 12\n");
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

                                    // QEI Server
                                    {
                                        qei_par qei_params;
                                        init_qei_param(qei_params);
                                        run_qei(c_qei_p1_2, c_qei_p2_2, c_qei_p3_2, c_qei_p4_2, c_qei_p5_2, p_ifm_encoder_node_2, qei_params);     // channel priority 1,2..5
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
	                                    initializeWheel(initRL);

	                                    while(1){
	                                         vel = getVelocity(iWheelRL);
	                                         set_profile_velocity(vel, 2000, 1000, MAX_PROFILE_VELOCITY, c_velocity_ctrl_3);
	                                     }
	                                 }
                                    /*Velocity Control Loop*/
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

                                        // QEI Server
                                        {
                                            qei_par qei_params;
                                            init_qei_param(qei_params);
                                            run_qei(c_qei_p1_3, c_qei_p2_3, c_qei_p3_3, c_qei_p4_3, c_qei_p5_3, p_ifm_encoder_node_3, qei_params);     // channel priority 1,2..5
                                        }
                                    }
	                        }

	}
	return 0;
}
