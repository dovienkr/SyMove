/**
*
* \file test_position-ctrl.xc
*
* \brief Main project file
*  Test illustrates usage of profile position control
*
*/

//motor control headers
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
//#include <position_ctrl_server.h>
#include <velocity_ctrl_server.h>
#include <drive_config.h>
#include <profile_control.h>
#include <internal_config.h>
//#include "include/interfaces.h"

//joystick headers
#include <adc_server_ad7949.h>
#include <adc_client_ad7949.h>

#include <symove_conf.h>

//Range finder headers
#include <laser.h>
#include "hokuyo_pbs_driver.h"
#include "uart_rx.h"
#include "uart_tx.h"

#include <trigonometry.h>

//fw update headers
#include <flash_somanet.h>
#include <ethercat.h>


//websockets
#include "websocketHandler.h"
#include <stdlib.h>
#include <string.h>
#include "motor_logic.h"
#include "rover_manager.h"
#include "websocket_logic.h"

on stdcore[3]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[3]: clock clk_pwm = XS1_CLKBLK_REF;
on stdcore[7]: clock clk_adc_1 = XS1_CLKBLK_1;
on stdcore[7]: clock clk_pwm_1 = XS1_CLKBLK_REF;
on stdcore[11]: clock clk_adc_2 = XS1_CLKBLK_1;
on stdcore[11]: clock clk_pwm_2 = XS1_CLKBLK_REF;
on stdcore[15]: clock clk_adc_3 = XS1_CLKBLK_1;
on stdcore[15]: clock clk_pwm_3 = XS1_CLKBLK_REF;

on stdcore[16]: out port reset = XS1_PORT_1F;


/*on stdcore[13]: out port p_core_leds_node_3[3] = { XS1_PORT_1M,   // Red
    XS1_PORT_1L,   // Green
XS1_PORT_1K }; // Blue */

buffered in port:1 p_rx = on stdcore[12] : XS1_PORT_1F;
out port p_tx = on stdcore[12] : XS1_PORT_1G;

/*interface Initialization {
    void init(int i);
};*/

/*interface MovingPermit{
    void permit(unsigned p);
};*/

interface Points {
    void get(unsigned n, unsigned index[105],unsigned length[105]);
};

interface MovingPermit{
    unsigned obtain(void);
};

interface ObstacleReport{
    void state(unsigned obstacles);
};

interface Wheel{
    void velocity(int vel);
};

interface Velocities{
    void get(int a, int b, int c, int d);
};

interface Vector{
  void get(float vx, float vy);
};

interface RawVector{
  void get(int vx, int vy);
};


int getVelocity(interface Wheel server wheel){
    select{
        case wheel.velocity(int vel):
        return vel;
    }
}


void vectorManager (chanend xy, interface RawVector client iRawVector)
{
    int x = 0;
    int y = 0;
    int c = 0;
    //int motor_left = 0;
    //int motor_right = 0;
    //int driveMode = 0;
    timer safetyTimer;
    unsigned time;
    unsigned safety_time = 200 * MSEC_STD;


    unsigned inited = 0;


    while (1)
    {
        select
        {
            case xy :> x:
            {

                //Read second value
                xy :> y;

                // Reset timer
                safetyTimer :> time;
                time += safety_time;

                break;
            }
           /* case giveMeVector :> c:
            {    giveMeVector <: x;
                 //giveMeVector <: y;

                break;
            }*/
            case safetyTimer when timerafter(time) :> void:
            {
                //Safety function -> Turns of motors after 250ms
               // motor_left = 0;
               // motor_right = 0;
                x = 0;
                y = 0;

                // Reset timer
                safetyTimer :> time;
                time += safety_time;
                break;
            }

            default:
                break;
        }

        iRawVector.get(x,y);

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

        if (xSignal < (X_ZERO - ZERO_ERROR))
        {
            vx = 1.0 * (X_ZERO - xSignal) / (X_ZERO - X_MIN);
            vx = vx * MAX_LINEAL_VELOCITY;
            } else if(xSignal > (X_ZERO + ZERO_ERROR)){
            vx = - 1.0 * (xSignal - X_ZERO) / (X_MAX - X_ZERO);
            vx = vx * MAX_LINEAL_VELOCITY;
        }
        else
        {
            vx = 0;
        }
        if (ySignal < (Y_ZERO - ZERO_ERROR)){
            vy = - 1.0 * (Y_ZERO - ySignal) / (Y_ZERO - Y_MIN);
            vy = vy * MAX_LINEAL_VELOCITY;
            }else if(ySignal > (Y_ZERO+ ZERO_ERROR)){
            vy =  1.0 * (ySignal - Y_ZERO) / (Y_MAX- Y_ZERO);
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
        //    chan c_qei_p1[N_NODES], c_qei_p2[N_NODES], c_qei_p3[N_NODES], c_qei_p4[N_NODES], c_qei_p5[N_NODES];                    // qei channels
        //    chan c_hall_p1[N_NODES], c_hall_p2[N_NODES], c_hall_p3[N_NODES], c_hall_p4[N_NODES], c_hall_p5[N_NODES];                // hall channels
        //    chan c_commutation_p1[N_NODES], c_commutation_p2[N_NODES], c_commutation_p3[N_NODES], c_signal[N_NODES];    // commutation channels
        //    chan c_pwm_ctrl[N_NODES], c_adctrig[N_NODES];                                                // pwm channels
        //    chan c_velocity_ctrl[N_NODES];       // position control channel
        //    chan c_watchdog[N_NODES]; // watchdog channel

        chan c_adc;

        chan c_qei_p1_0, c_qei_p2_0;                 // qei channels
        chan c_hall_p1_0, c_hall_p2_0, c_hall_p3_0, c_hall_p4_0, c_hall_p5_0;                // hall channels
        chan c_commutation_p1_0, c_commutation_p2_0, c_commutation_p3_0, c_signal_0;    // commutation channels
        chan c_pwm_ctrl_0, c_adctrig_0;                                               // pwm channels
        chan c_velocity_ctrl_0;       // position control channel
        chan c_watchdog_0; // watchdog channel

        chan c_qei_p1_1, c_qei_p2_1;                 // qei channels
        chan c_hall_p1_1, c_hall_p2_1, c_hall_p3_1, c_hall_p4_1, c_hall_p5_1;                // hall channels
        chan c_commutation_p1_1, c_commutation_p2_1, c_commutation_p3_1, c_signal_1;    // commutation channels
        chan c_pwm_ctrl_1, c_adctrig_1;                                               // pwm channels
        chan c_velocity_ctrl_1;       // position control channel
        chan c_watchdog_1; // watchdog channel

        chan c_qei_p1_2, c_qei_p2_2;                 // qei channels
        chan c_hall_p1_2, c_hall_p2_2, c_hall_p3_2, c_hall_p4_2, c_hall_p5_2;                // hall channels
        chan c_commutation_p1_2, c_commutation_p2_2, c_commutation_p3_2, c_signal_2;    // commutation channels
        chan c_pwm_ctrl_2, c_adctrig_2;                                               // pwm channels
        chan c_velocity_ctrl_2;       // position control channel
        chan c_watchdog_2; // watchdog channel

        chan c_qei_p1_3, c_qei_p2_3;                 // qei channels
        chan c_hall_p1_3, c_hall_p2_3, c_hall_p3_3, c_hall_p4_3, c_hall_p5_3;                // hall channels
        chan c_commutation_p1_3, c_commutation_p2_3, c_commutation_p3_3, c_signal_3;    // commutation channels
        chan c_pwm_ctrl_3, c_adctrig_3;                                               // pwm channels
        chan c_velocity_ctrl_3;       // position control channel
        chan c_watchdog_3; // watchdog channel

        //laser channels
        chan c_chanTX, c_chanRX;

            chan c_getJoystick;


        interface Wheel iWheelFL;
        interface Wheel iWheelFR;
        interface Wheel iWheelRR;
        interface Wheel iWheelRL;

        interface Velocities iVelocities;
        interface LaserData iLaserData;

        interface Vector iVector;
        interface RawVector iRawVector;
        interface MovingPermit iMovingPermit;
        interface ObstacleReport iObstacleReport;

        chan coe_in;        // CAN from module_ethercat to consumer
        chan coe_out;       // CAN from consumer to module_ethercat
        chan eoe_in;        // Ethernet from module_ethercat to consumer
        chan eoe_out;       // Ethernet from consumer to module_ethercat
        chan eoe_sig;
        chan foe_in;        // File from module_ethercat to consumer
        chan foe_out;       // File from consumer to module_ethercat
        chan pdo_in;
        chan pdo_out;
        chan sig_1;
        chan c_flash_data;
        chan c_nodes[4];   // only upto 17 nodes on dx can have firmware updated - with current implementation structure

        //websockets channels
        chan dataRcv;
        chan dataSend; // Only needed when sending data via websockets to the server
        chan giveMeVector, init;
        chan ddrive;
        chan xy;
        chan c_adc1;



        par
        {
            on stdcore[0] :
            {

                timer t0;

                wait_ms(3000,0,t0); //delay for evetything to get started


              //    unsigned get;
              //  select{
              //     case c_start :> get:
              //      break;
              //  }

                //    p_core_leds[0] <: 1;
                //    p_core_leds[1] <: 0;
                int newA,newB,newC,newD,newP;
                while(1){

                    c_getJoystick <: 1;

                         select{
                            case iVelocities.get(int a, int b, int c, int d):
                                iWheelFL.velocity(a);
                                iWheelFR.velocity(b);
                                iWheelRR.velocity(c);
                                iWheelRL.velocity(d);

                            break;

                        }
                        wait_ms(50, 0, t0);
                    }

            }

            on stdcore[0] :
            {
                par{
                    {
                        ecat_init();
                        ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out, foe_in, pdo_out, pdo_in);
                    }
                    {
                        firmware_update_loop(p_spi_flash_0, foe_out, foe_in, c_flash_data, c_nodes, sig_1); // firmware update over ethercat
                    }
                }
            }

            on stdcore[1] :
            {
                p_core_leds_0[0] <: 1; //red
                p_core_leds_0[1] <: 1; //green
                p_core_leds_0[2] <: 0; //blue
                while(1);
            }
            on stdcore[2] :
            {

                //                    printstrln("core 3\n");

                timer t0;
              //
                wait_ms(2000,2,t0); //delay for evetything to get started


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

            // IFM_CORE

            on stdcore[3] :
            {

               timer t0;
               wait_ms(2000,3,t0); //delay for evetything to get started

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
                        init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);             // initialize commutation parameters
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
                firmware_update_dx(p_spi_flash_1, c_nodes[0], 2);
            }
            on stdcore[5] :
            {

               // int joystick_x, joystick_y;
               // float vx, vy;
                int a,b,c,d;
               // unsigned get;
                timer t0;
                unsigned obstacleState = 0;
                wait_ms(500, 5, t0); //delay for ADC server to get initialised

             //   for(int i= 0; i<20;i++){

             //       {joystick_x , joystick_y} = get_adc_external_potentiometer_ad7949(c_adc);

             //   }
                //  printstrln("entro al loop");
                while(1){
                    select{
                        case iVector.get(float vx, float vy): //c_getJoystick :> get:
                     //   {joystick_y,joystick_x} = get_adc_external_potentiometer_ad7949(c_adc); //y=port1, x=port2

                        //     printintln(joystick_y);
                        //     printintln(joystick_x);

                    //    {vx, vy} = convertJoystick2Vector(joystick_x, joystick_y);

                      //  iVector.get(12*vx,12*vy); // send the vector to obtain moving permit
                        //      c_vx <: vx;
                        //      c_vy <: vy;
                        if(vy<0){
                            {a,b,c,d} = convertVector2Vel(0,0,3*(vx+1));
                            iVelocities.get(a,b,c,d);
                        }else{

                            obstacleState = iMovingPermit.obtain();
                            if(obstacleState==0){
                                {a,b,c,d} = convertVector2Vel(vy,vx,0); //front
                                iVelocities.get(a,b,c,d);
                            }else{
                                iVelocities.get(0,0,0,0);
                            }
                        }
                        break;
                    }
                }
            }
            on stdcore[6] :
            {
                timer t0;
                wait_ms(2000,2,t0); //delay for evetything to get started

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
                timer t0;
                wait_ms(2000,3,t0); //delay for evetything to get started

                //    printstrln("core 8\n");
                par
                {
                    //ADC Server for Joystick
                   // adc_ad7949(c_adc, clk_adc_1, p_ifm_adc_sclk_conv_mosib_mosia_node_1, p_ifm_adc_misoa_node_1, p_ifm_adc_misob_node_1);

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
                firmware_update_dx(p_spi_flash_2, c_nodes[1], 3);
            }
            on stdcore[9] :
            {
                //        printstrln("core 10\n");

            }
            on stdcore[10] :
            {
                timer t0;
                wait_ms(2000,2,t0); //delay for evetything to get started

                //                printstrln("core 11\n");
                par {
                    //Velocity ramp generation
                    {   int vel;
                        //       initializeWheel(initRR);

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
                //    printstrln("core 12\n");
                timer t0;
                wait_ms(2000,3,t0); //delay for evetything to get started

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

                unsigned char tx_buffer[MAX_BUFFER_SIZE];
                unsigned char rx_buffer[MAX_BUFFER_SIZE];
                par{
                    uart_rx(p_rx, rx_buffer, ARRAY_SIZE(rx_buffer), BAUD_RATE, 7, UART_TX_PARITY_NONE, 1, c_chanRX);
                    uart_tx(p_tx, tx_buffer, ARRAY_SIZE(tx_buffer), BAUD_RATE, 7, UART_TX_PARITY_NONE, 1, c_chanTX);
                    {
                        timer t0;
                        wait_ms(1000,12, t0); //delay for uarts to get initialised
                        while (1)
                        {
                            run_scanner(c_chanRX, c_chanTX, iLaserData);
                        }
                    }

                    firmware_update_dx(p_spi_flash_3, c_nodes[2], 4);

                }
            }
            on stdcore[13] :
            {

               p_core_leds_3[0] <: 1;
               p_core_leds_3[1] <: 0; //green
               p_core_leds_3[2] <: 1;

                unsigned nOfDangerousPoints= 0;
                unsigned dangerousPoints[105];

                par
                {
                    {
                        unsigned startFLAG = 0;
                        while(1){
                            select
                            {
                                case iLaserData.get(unsigned length[]):
                                    nOfDangerousPoints = 0;
                                for(unsigned i=5;i<110;i++){ //firsts and last samples are zeros
                                    if(length[i]!=0 & length[i]<600){
                                        dangerousPoints[nOfDangerousPoints++] = i-5; //offset, first samples are zeros
                                    //  dangerousLengths[nOfDangerousPoints++] = length[i];
                                    }
                                }
                                iObstacleReport.state(nOfDangerousPoints);

                                break;


                            }
                        }

               // p_core_leds_node_3[0] <: 1;
               // p_core_leds_node_3[1] <: 0;
               // p_core_leds_node_3[2] <: 1;

                    }
                    {


                                  timer t0;

                                  unsigned obstacleState=0;//, get;

                                 while(1)
                                 {

                                       select{

                                           case iObstacleReport.state(unsigned state):
                                               obstacleState = state;
                                               if(state>0){
                                                 p_core_leds_3[0] <: 0; //red
                                                 p_core_leds_3[1] <: 1;
                                                 p_core_leds_3[2] <: 1;

                                             }else{
                                                p_core_leds_3[0] <: 1;
                                                p_core_leds_3[1] <: 0; //green
                                                p_core_leds_3[2] <: 1;

                                             }
                                               break;

                                           case iMovingPermit.obtain() -> unsigned return_val:

                                           return_val = obstacleState;

                                               break;
                                       }
                                 }

                              }

                 }
}


            on stdcore[14] :
            {
                //                printstrln("core 15\n");
                timer t0;
                wait_ms(2000,2,t0); //delay for evetything to get started
                par
                {
                    {
                        int vel;
                        //       initializeWheel(initRL);

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
                timer t0;
                wait_ms(2000,3,t0); //delay for evetything to get started

                //printstrln("core 16\n");
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
                        commutation_sinusoidal(c_hall_p1_3,  c_qei_p2_3, c_signal_3,c_watchdog_3,
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
            }

            // Node 5
            on stdcore[16] :
            {

               // firmware_update_dx(p_spi_flash_4, c_nodes[3], 5);
                par{
                    websocketClient(dataRcv, dataSend);
                    receive(dataRcv, xy);
                    vectorManager(xy, iRawVector);
                    roverManager(dataSend, c_adc1);
                }
            }
            on stdcore[17] :
            {

                float x,y;
                //int px,py;
                unsigned c;
               // timer t0;

                while (1) {
                select{
                    case iRawVector.get(int px, int py):
                    {
                        if(px>600){
                            px = 600;
                        }else if(px<-600){
                            px = -600;
                        }

                        x = px / 100.0;

                        if(py>600){
                            py = 600;
                        }else if(py<-600){
                            py = -600;
                        }

                        y = py / 100.0;

                   //     c++;
                   //     if (px !=0 && c > 100 ){

                    // printint((int)x);
                    // printstr(" ");
                    // printintln((int)y);
                   //  c=0;
                        }
                     break;

                    case  c_getJoystick :> c:
                    {
                        iVector.get(x,y);
                        break;
                    }
                 }

                }



            }
            on stdcore[18] :{}
            on stdcore[19] :{}

        }
        return 0;
    }
