
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
#include <drive_config.h>
#include <profile_control.h>
#include <internal_config.h>


on stdcore[3]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[3]: clock clk_pwm = XS1_CLKBLK_REF;
on stdcore[7]: clock clk_adc_1 = XS1_CLKBLK_1;
on stdcore[7]: clock clk_pwm_1 = XS1_CLKBLK_REF;
on stdcore[11]: clock clk_adc_2 = XS1_CLKBLK_1;
on stdcore[11]: clock clk_pwm_2 = XS1_CLKBLK_REF;
on stdcore[15]: clock clk_adc_3 = XS1_CLKBLK_1;
on stdcore[15]: clock clk_pwm_3 = XS1_CLKBLK_REF;

#define N_NODES 4



/* Test Profile Position function */

void position_profile_test(chanend c_position_ctrl)
{
	int target_position = 350;			// degree
	int velocity 		= 4000;			// rpm
	int acceleration 	= 350;			// rpm/s
	int deceleration 	= 350;     		// rpm/s

	init_position_profile_limits(GEAR_RATIO, MAX_ACCELERATION, MAX_PROFILE_VELOCITY);

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	while(1){
	set_profile_position(target_position, velocity, acceleration, deceleration, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT, c_position_ctrl);
	target_position = 0; 	//degree
	set_profile_position(target_position, velocity, acceleration, deceleration, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT, c_position_ctrl);
	target_position = 350;
	}
}

 //Test Profile Position function
void position_profile_test_1(chanend c_position_ctrl)
{
	int target_position = 350;			// degree
	int velocity 		= 3000;			// rpm
	int acceleration 	= 350;			// rpm/s
	int deceleration 	= 350;     		// rpm/s

	init_position_profile_limits(GEAR_RATIO, MAX_ACCELERATION, MAX_PROFILE_VELOCITY);

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	while(1){
	set_profile_position(target_position, velocity, acceleration, deceleration, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT, c_position_ctrl);
	target_position = 350;
	set_profile_position(target_position, velocity, acceleration, deceleration, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT, c_position_ctrl);
	target_position = 0; 	//degree
	}
}




int main(void)
{
	// Motor control channels
	chan c_qei_p1[N_NODES], c_qei_p2[N_NODES], c_qei_p3[N_NODES], c_qei_p4[N_NODES], c_qei_p5[N_NODES];					// qei channels
	chan c_hall_p1[N_NODES], c_hall_p2[N_NODES], c_hall_p3[N_NODES], c_hall_p4[N_NODES], c_hall_p5[N_NODES];				// hall channels
	chan c_commutation_p1[N_NODES], c_commutation_p2[N_NODES], c_commutation_p3[N_NODES], c_signal[N_NODES];	// commutation channels
	chan c_pwm_ctrl[N_NODES], c_adctrig[N_NODES];												// pwm channels
	chan c_position_ctrl[N_NODES];													// position control channel
	chan c_watchdog[N_NODES]; 														// watchdog channel
//
	chan c_qei_p1_1, c_qei_p2_1, c_qei_p3_1, c_qei_p4_1, c_qei_p5_1;					// qei channels
	chan c_hall_p1_1, c_hall_p2_1, c_hall_p3_1, c_hall_p4_1, c_hall_p5_1;				// hall channels
	chan c_commutation_p1_1, c_commutation_p2_1, c_commutation_p3_1, c_signal_1;	// commutation channels
	chan c_pwm_ctrl_1, c_adctrig_1;												// pwm channels
	chan c_position_ctrl_1;													// position control channel
	chan c_watchdog_1; 														// watchdog channel
//



	par
	{
		on stdcore[0] :
				{
						printstrln("core 1\n");
				}
		on stdcore[1] :
				{
						printstrln("core 2\n");
						//Test Profile Position function
						position_profile_test(c_position_ctrl[0]);		  	// test PPM on slave side
				}
		on stdcore[2] :
				{
						printstrln("core 3\n");

						//Position Control Loop
						{
							 ctrl_par position_ctrl_params;
							 hall_par hall_params;
							 qei_par qei_params;

							 init_position_control_param(position_ctrl_params);
							 init_hall_param(hall_params);
							 init_qei_param(qei_params);

							 position_control(position_ctrl_params, hall_params, qei_params, SENSOR_USED, c_hall_p2[0],
									 c_qei_p1[0], c_position_ctrl[0], c_commutation_p3[0]);
						}
				}
		/***********************************************************
		 * IFM_CORE
		 ***********************************************************/
		on stdcore[3] :
				{
						printstrln("core 4\n");

						par
								{
						// PWM Loop
						do_pwm_inv_triggered(c_pwm_ctrl[0], c_adctrig[0], p_ifm_dummy_port_node_0,
								p_ifm_motor_hi_node_0, p_ifm_motor_lo_node_0, clk_pwm);

						// Motor Commutation loop
						{
							hall_par hall_params;
							qei_par qei_params;
							commutation_par commutation_params;
							init_hall_param(hall_params);
							init_qei_param(qei_params);
							init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); 			// initialize commutation parameters
							commutation_sinusoidal(c_hall_p1[0],  c_qei_p2[0], c_signal[0], c_watchdog[0],
									c_commutation_p1[0], c_commutation_p2[0], c_commutation_p3[0],
									c_pwm_ctrl[0], hall_params, qei_params, commutation_params, p_ifm_esf_rstn_pwml_pwmh_node_0, p_ifm_coastn_node_0);
						}

						// Watchdog Server
						run_watchdog(c_watchdog[0], p_ifm_wd_tick_node_0, p_ifm_shared_leds_wden_node_0);

						// Hall Server
						{
							hall_par hall_params;
							init_hall_param(hall_params);
							run_hall(c_hall_p1[0], c_hall_p2[0], c_hall_p3[0], c_hall_p4[0], c_hall_p5[0], p_ifm_hall_node_0, hall_params); // channel priority 1,2..5
						}

						 //QEI Server
						{
							qei_par qei_params;
							init_qei_param(qei_params);
							run_qei(c_qei_p1[0], c_qei_p2[0], c_qei_p3[0], c_qei_p4[0], c_qei_p5[0], p_ifm_encoder_node_0, qei_params);  	// channel priority 1,2..5
						}
								}
				}

		 on stdcore[4] :
				{
						printstrln("core 5\n");
				}
		on stdcore[5] :
				{
						printstrln("core 6\n");
						//Test Profile Position function
						position_profile_test_1(c_position_ctrl[1]);		  	// test PPM on slave side
				}
		on stdcore[6] :
				{
						printstrln("core 7\n");
						//Position Control Loop
									{
							 ctrl_par position_ctrl_params;
							 hall_par hall_params;
							 qei_par qei_params;

							 init_position_control_param(position_ctrl_params);
							 init_hall_param(hall_params);
							 init_qei_param(qei_params);

							 position_control(position_ctrl_params, hall_params, qei_params, SENSOR_USED, c_hall_p2[1],
									 c_qei_p1[1], c_position_ctrl[1], c_commutation_p3[1]);
						}
				}
		on stdcore[7] :
				{
						printstrln("core 8\n");
						par
							{
								 //PWM Loop
								do_pwm_inv_triggered(c_pwm_ctrl[1], c_adctrig[1], p_ifm_dummy_port_node_1,
										p_ifm_motor_hi_node_1, p_ifm_motor_lo_node_1, clk_pwm_1);

								 // Motor Commutation loop
								{
									hall_par hall_params;
									qei_par qei_params;
									commutation_par commutation_params;

									init_hall_param(hall_params);
									init_qei_param(qei_params);
									init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); 			// initialize commutation parameters
									commutation_sinusoidal(c_hall_p1[1],  c_qei_p2[1], c_signal[1], c_watchdog[1],
											c_commutation_p1[1], c_commutation_p2[1], c_commutation_p3[1],
											c_pwm_ctrl[1], hall_params, qei_params, commutation_params, p_ifm_esf_rstn_pwml_pwmh_node_1, p_ifm_coastn_node_1);
								}

								// Watchdog Server
								run_watchdog(c_watchdog[1], p_ifm_wd_tick_node_1, p_ifm_shared_leds_wden_node_1);

								// Hall Server
								{
									hall_par hall_params;
									init_hall_param(hall_params);
									run_hall(c_hall_p1[1], c_hall_p2[1], c_hall_p3[1], c_hall_p4[1], c_hall_p5[1], p_ifm_hall_node_1, hall_params); // channel priority 1,2..5
								}

								// QEI Server
								{
									qei_par qei_params;
									init_qei_param(qei_params);
									run_qei(c_qei_p1[1], c_qei_p2[1], c_qei_p3[1], c_qei_p4[1], c_qei_p5[1], p_ifm_encoder_node_1, qei_params);  	// channel priority 1,2..5
								}
							}
				}
		 on stdcore[8] :
						{
								printstrln("core 9\n");
						}
		 on stdcore[9] :
						{
								printstrln("core 10\n");
								position_profile_test(c_position_ctrl[2]);            // test PPM on slave side
						}
		 on stdcore[10] :
						{
								printstrln("core 11\n");
							    //Position Control Loop
							    {
                                     ctrl_par position_ctrl_params;
                                     hall_par hall_params;
                                     qei_par qei_params;

                                     init_position_control_param(position_ctrl_params);
                                     init_hall_param(hall_params);
                                     init_qei_param(qei_params);

                                     position_control(position_ctrl_params, hall_params, qei_params, SENSOR_USED, c_hall_p2[2],
                                             c_qei_p1[2], c_position_ctrl[2], c_commutation_p3[2]);
							     }
						}
		 on stdcore[11] :
						{
								printstrln("core 12\n");
								par
                                {
                                     //PWM Loop
                                    do_pwm_inv_triggered(c_pwm_ctrl[2], c_adctrig[2], p_ifm_dummy_port_node_2,
                                            p_ifm_motor_hi_node_2, p_ifm_motor_lo_node_2, clk_pwm_2);

                                     // Motor Commutation loop
                                    {
                                        hall_par hall_params;
                                        qei_par qei_params;
                                        commutation_par commutation_params;

                                        init_hall_param(hall_params);
                                        init_qei_param(qei_params);
                                        init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);             // initialize commutation parameters
                                        commutation_sinusoidal(c_hall_p1[2],  c_qei_p2[2], c_signal[2], c_watchdog[2],
                                                c_commutation_p1[2], c_commutation_p2[2], c_commutation_p3[2],
                                                c_pwm_ctrl[2], hall_params, qei_params, commutation_params, p_ifm_esf_rstn_pwml_pwmh_node_2, p_ifm_coastn_node_2);
                                    }

                                    // Watchdog Server
                                    run_watchdog(c_watchdog[2], p_ifm_wd_tick_node_2, p_ifm_shared_leds_wden_node_2);

                                    // Hall Server
                                    {
                                        hall_par hall_params;
                                        init_hall_param(hall_params);
                                        run_hall(c_hall_p1[2], c_hall_p2[2], c_hall_p3[2], c_hall_p4[2], c_hall_p5[2], p_ifm_hall_node_2, hall_params); // channel priority 1,2..5
                                    }

                                    // QEI Server
                                    {
                                        qei_par qei_params;
                                        init_qei_param(qei_params);
                                        run_qei(c_qei_p1[2], c_qei_p2[2], c_qei_p3[2], c_qei_p4[2], c_qei_p5[2], p_ifm_encoder_node_2, qei_params);     // channel priority 1,2..5
                                    }
                                }
						}

	     on stdcore[12] :
	                        {
	                                printstrln("core 13\n");
	                        }
	     on stdcore[13] :
	                        {
	                                printstrln("core 14\n");
	                                position_profile_test(c_position_ctrl[3]);            // test PPM on slave side
	                        }
	     on stdcore[14] :
	                        {
	                                printstrln("core 15\n");
	                                //Position Control Loop
                                    {
                                         ctrl_par position_ctrl_params;
                                         hall_par hall_params;
                                         qei_par qei_params;

                                         init_position_control_param(position_ctrl_params);
                                         init_hall_param(hall_params);
                                         init_qei_param(qei_params);

                                         position_control(position_ctrl_params, hall_params, qei_params, SENSOR_USED, c_hall_p2[3],
                                                 c_qei_p1[3], c_position_ctrl[3], c_commutation_p3[3]);
                                     }
	                        }
	     on stdcore[15] :
	                        {
	                                printstrln("core 16\n");
	                                par
                                    {
                                         //PWM Loop
                                        do_pwm_inv_triggered(c_pwm_ctrl[3], c_adctrig[3], p_ifm_dummy_port_node_3,
                                                p_ifm_motor_hi_node_3, p_ifm_motor_lo_node_3, clk_pwm_3);

                                         // Motor Commutation loop
                                        {
                                            hall_par hall_params;
                                            qei_par qei_params;
                                            commutation_par commutation_params;

                                            init_hall_param(hall_params);
                                            init_qei_param(qei_params);
                                            init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);             // initialize commutation parameters
                                            commutation_sinusoidal(c_hall_p1[3],  c_qei_p2[3], c_signal[3], c_watchdog[3],
                                                    c_commutation_p1[3], c_commutation_p2[3], c_commutation_p3[3],
                                                    c_pwm_ctrl[3], hall_params, qei_params, commutation_params, p_ifm_esf_rstn_pwml_pwmh_node_3, p_ifm_coastn_node_3);
                                        }

                                        // Watchdog Server
                                        run_watchdog(c_watchdog[3], p_ifm_wd_tick_node_3, p_ifm_shared_leds_wden_node_3);

                                        // Hall Server
                                        {
                                            hall_par hall_params;
                                            init_hall_param(hall_params);
                                            run_hall(c_hall_p1[3], c_hall_p2[3], c_hall_p3[3], c_hall_p4[3], c_hall_p5[3], p_ifm_hall_node_3, hall_params); // channel priority 1,2..5
                                        }

                                        // QEI Server
                                        {
                                            qei_par qei_params;
                                            init_qei_param(qei_params);
                                            run_qei(c_qei_p1[3], c_qei_p2[3], c_qei_p3[3], c_qei_p4[3], c_qei_p5[3], p_ifm_encoder_node_3, qei_params);     // channel priority 1,2..5
                                        }
                                    }
	                        }

	}
	return 0;
}
