/*
 * symove_conf.h
 *
 *  Created on: Mar 24, 2014
 *      Author: atena
 */

#ifndef SYMOVE_CONF_H_
#define SYMOVE_CONF_H_

#define PI 3.14159
//CORE LABELING

//CORES MAIN LABELING
#define NODE0_CORE0 0
#define NODE0_CORE1 1
#define NODE0_CORE2 2
#define NODE0_CORE3 3

#define NODE1_CORE0 4
#define NODE1_CORE1 5
#define NODE1_CORE2 6
#define NODE1_CORE3 7

#define NODE2_CORE0 8
#define NODE2_CORE1 9
#define NODE2_CORE2 10
#define NODE2_CORE3 11

#define NODE3_CORE0 12
#define NODE3_CORE1 13
#define NODE3_CORE2 14
#define NODE3_CORE3 15

//CORES FUNCTIONAL LABELING
#define NODE0_IFM NODE0_CORE3
#define NODE0_CO_IFM NODE0_CORE2
#define NODE0_COM NODE0_CORE0
#define NODE0_CO_COM NODE0_CORE1

#define NODE1_IFM NODE1_CORE3
#define NODE1_CO_IFM NODE1_CORE2
#define NODE1_COM NODE1_CORE0
#define NODE1_CO_COM NODE1_CORE1

#define NODE2_IFM NODE2_CORE3
#define NODE2_CO_IFM NODE2_CORE2
#define NODE2_COM NODE2_CORE0
#define NODE2_CO_COM NODE2_CORE1

#define NODE3_IFM NODE3_CORE3
#define NODE3_CO_IFM NODE3_CORE2
#define NODE3_COM NODE3_CORE0
#define NODE3_CO_COM NODE3_CORE1

//NODEs CONFIGURATION
#define N_NODES 4

//MOTOR NUMERATION
#define FRONT_LEFT_MOTOR 0
#define FRONT_RIGHT_MOTOR 1
#define REAR_RIGHT_MOTOR 2
#define REAR_LEFT_MOTOR 3


//ROBOT FUNCTION LABELING
#define FRONT_LEFT_MOTOR_INTERFACE_CORE NODE0_IFM
#define FRONT_RIGHT_MOTOR_INTERFACE_CORE NODE1_IFM
#define REAR_RIGHT_MOTOR_INTERFACE_CORE NODE2_IFM
#define REAR_LEFT_MOTOR_INTERFACE_CORE NODE3_IFM

#define FRONT_LEFT_MOTOR_CONTROLLER_CORE NODE0_CO_IFM
#define FRONT_RIGHT_MOTOR_CONTROLLER_CORE NODE1_CO_IFM
#define REAR_RIGHT_MOTOR_CONTROLLER_CORE NODE2_CO_IFM
#define REAR_LEFT_MOTOR_CONTROLLER_CORE NODE3_CO_IFM

//ROBOT GEOMETRY
#define L1 0.125 //m
#define L2 0.115 // m
#define WHEEL_RADIUS 0.05 //m

//JOYSTICK PARAMETERS
#define X_MAX 16400
#define X_ZERO 10100
#define X_MIN 10

#define Y_MAX 16400
#define Y_ZERO 10640
#define Y_MIN 5

#define ZERO_ERROR 200

//VELOCITY PARAM
#define MAX_LINEAL_VELOCITY 6 //m/s
#define ACCELERATION 6000
#define DECELERATION 6000

#endif /* SYMOVE_CONF_H_ */
