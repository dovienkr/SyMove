
/**
 *
 * \file test_position-ctrl.xc
 *
 * \brief Main project file
 *  Test illustrates usage of profile position control
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

#include <adc_server_ad7949.h>
#include <adc_client_ad7949.h>

#include <symove_conf.h>

#include <laser.h>
#include "hokuyo_pbs_driver.h"
#include "uart_rx.h"
#include "uart_tx.h"

on stdcore[3]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[3]: clock clk_pwm = XS1_CLKBLK_REF;
on stdcore[7]: clock clk_adc_1 = XS1_CLKBLK_1;
on stdcore[7]: clock clk_pwm_1 = XS1_CLKBLK_REF;


//buffered in port:1 p_rx = on stdcore[12] : XS1_PORT_1F;
//out port p_tx = on stdcore[12] : XS1_PORT_1G;

interface Initialization {
    void init(int i);
};

interface Wheel{
    void velocity(int vel);
};

void init(interface Initialization server initW){
       select {
       case initW.init(int i):
     //    printstrln("Wheel Initialized");
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


int main(void)
{
	// Motor control channels
//	chan c_qei_p1[N_NODES], c_qei_p2[N_NODES], c_qei_p3[N_NODES], c_qei_p4[N_NODES], c_qei_p5[N_NODES];					// qei channels
//	chan c_hall_p1[N_NODES], c_hall_p2[N_NODES], c_hall_p3[N_NODES], c_hall_p4[N_NODES], c_hall_p5[N_NODES];				// hall channels
//	chan c_commutation_p1[N_NODES], c_commutation_p2[N_NODES], c_commutation_p3[N_NODES], c_signal[N_NODES];	// commutation channels
//	chan c_pwm_ctrl[N_NODES], c_adctrig[N_NODES];												// pwm channels
//	chan c_velocity_ctrl[N_NODES];       // position control channel
//	chan c_watchdog[N_NODES]; // watchdog channel

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


    //laser channels
    chan c_laser_data;
    chan c_chanTX, c_chanRX;

    chan c_start;
    chan c_getJoystick;
    chan c_va,c_vb,c_vc,c_vd,c_vx,c_vy;
    chan c_getDangerousPoints;
    chan c_movementPossible;

	interface Initialization initRR;
	interface Initialization initRL;
	interface Initialization initFR;
	interface Initialization initFL;

	interface Wheel iWheelFL;
	interface Wheel iWheelFR;

    par
	{
		on stdcore[0] :
		{
	           // init(initFL);
	    //        init(initFR);
	    //        init(initRR);
	    //        init(initRL);

		    printstrln("aaaa");
		    timer t0;
            int a,b,c,d;

             unsigned get;
             select{
                 case c_start :> get:
                     break;
             }


                while(1){

                       c_getJoystick <: 1;
               //        for(int i=0;i<1;i++){
                           select{

                               case c_va :> a:
                                   c_vb :> b;
                                   c_vc :> c;
                                   c_vd :> d;
                                   break;
                               case c_movementPossible :> get:
                                   break;

                    //       }
                       }

                    setVelocity(a,iWheelFL);
                    setVelocity(b,iWheelFR);

                    wait_ms(1, 0, t0);
                }
		}

		on stdcore[1] :
				{
		//    p_core_leds[0] <: 1;
		//    p_core_leds[1] <: 0;
		//    p_core_leds[2] <: 1;
		//    while(1);

				}
		on stdcore[2] :
				{

	//					printstrln("core 3\n");

		        par {

		                //Velocity ramp generation
		            	                {
		                    int vel;

		            //        initializeWheel(initFL);

		                  while(1){
                                vel = getVelocity(iWheelFL);
                                set_profile_velocity(vel, ACCELERATION, DECELERATION, MAX_PROFILE_VELOCITY, c_velocity_ctrl_0);
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
                                 qei_params, SENSOR_USED, c_hall_p2_0, c_qei_p1_0, c_velocity_ctrl_0, c_commutation_p2_0);
                        }

		        }

				}
		/***********************************************************
		 * IFM_CORE
		 ***********************************************************/
		on stdcore[3] :
		{
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
                            c_pwm_ctrl_0, hall_params, qei_params, commutation_params,
                            p_ifm_esf_rstn_pwml_pwmh_node_0, p_ifm_coastn_node_0);


                }

                // Watchdog Server
                run_watchdog(c_watchdog_0, p_ifm_wd_tick_node_0, p_ifm_shared_leds_wden_node_0);

                // Hall Server
                {
                    hall_par hall_params;
                    init_hall_param(hall_params);
                    run_hall(c_hall_p1_0, c_hall_p2_0, c_hall_p3_0, c_hall_p4_0, c_hall_p5_0, p_ifm_hall_node_0, hall_params); // channel priority 1,2..5
                }
            }
        }

		on stdcore[4] :
				{
                    int joystick_x, joystick_y;
                    float vx, vy;
                    int a,b,c,d;
                    unsigned get;

                    for(int i= 0; i<20;i++){

                        {joystick_x , joystick_y} = get_adc_external_potentiometer_ad7949(c_adc);

                       // wait_ms(10, 0, t0);
                    }

                    while(1){
                        select{
                            case c_getJoystick :> get:
                            {joystick_y,joystick_x} = get_adc_external_potentiometer_ad7949(c_adc); //y=port1, x=port2
                            {vx, vy} = convertJoystick2Vector(joystick_x, joystick_y);

                      //      c_vx <: vx;
                      //      c_vy <: vy;

                            {a,b,c,d} = convertVector2Vel(vy,vx,0); //front

                            c_va <: a;
                            c_vb <: b;
                            c_vc <: c;
                            c_vd <: d;
                            break;
                        }
                    }
				}
		on stdcore[5] :
				{
		                int vx, vy;
		                unsigned nOfDangerousPoints;
		                unsigned dangerousPoints[NUMBER_OF_RANGE_READINGS];
		                unsigned dangerousLengths[NUMBER_OF_RANGE_READINGS];

                        while(1){
                            select{
                                case c_vx :> vx:
                                    c_vy :> vy;

                                    //c_getDangerousPoints <: 1;
                                    select{
                                        case c_getDangerousPoints :> nOfDangerousPoints:
                                            for(int i=0;i<nOfDangerousPoints;i++){
                                                c_getDangerousPoints :> dangerousPoints[i];
                                                c_getDangerousPoints :> dangerousLengths[i];
                                            }
                                            break;
                                    }

                                    //return = isMovementPossible();
                                    //c_movementPossible <: return;
                                    break;
                            }
                        }
				}
		on stdcore[6] :
				{

			//			printstrln("core 7\n");
		        par
		        {
                        //Velocity ramp generation
                        {
                            int vel;

                        //   initializeWheel(initFR);

                            while(1){
                              vel = getVelocity(iWheelFR);
                             set_profile_velocity(vel, ACCELERATION, DECELERATION, MAX_PROFILE_VELOCITY, c_velocity_ctrl_1);
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
							}
				}

	}
	return 0;
}
