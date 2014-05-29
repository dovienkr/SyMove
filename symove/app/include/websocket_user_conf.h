/**
 *
 * \file websocket_conf.h
 *
 * Configuration file for the SOMANET websocket client
 *
 *
 */

/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 * Author: Simon Fischinger <sfischinger@synapticon.com>
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
 */



/* Set to 1 to use DHCP to obtain an address within the local network */
#define WEBSOCKET_USE_DHCP 1


/* IP of the local websocket client within the local network
* (is ignored when DHCP is active) */
#define WEBSOCKET_CLIENT_IP_1stOctet 192
#define WEBSOCKET_CLIENT_IP_2ndOctet 168
#define WEBSOCKET_CLIENT_IP_3rdOctet 0
#define WEBSOCKET_CLIENT_IP_4thOctet 222

/* NETMASK of the local websocket client
* (is ignored when DHCP is active) */
#define WEBSOCKET_NETMASK_1stOctet 255
#define WEBSOCKET_NETMASK_2ndOctet 255
#define WEBSOCKET_NETMASK_3rdOctet 255
#define WEBSOCKET_NETMASK_4thOctet 0


/* GATEWAY in the local network
* (is ignored when DHCP is active) */
#define WEBSOCKET_GATEWAY_1stOctet 192
#define WEBSOCKET_GATEWAY_2ndOctet 168
#define WEBSOCKET_GATEWAY_3rdOctet 0
#define WEBSOCKET_GATEWAY_4thOctet 1


/* SERVER IP of the websocket server the application should connect to */
#define WEBSOCKET_SERVER_IP_1stOctet 85
#define WEBSOCKET_SERVER_IP_2ndOctet 214
#define WEBSOCKET_SERVER_IP_3rdOctet 233
#define WEBSOCKET_SERVER_IP_4thOctet 250

/* PORT of the websocket server */
#define WEBSOCKET_SERVER_PORT 1234



/* ============================== MACROS ================================== */
/* Macro for building the websocket header --> Leave untouched!! */
#define INT2STRING(INT) DO_INT2STRING(INT)
#define DO_INT2STRING(INT) #INT

#define WEBSOCKET_HEADER    "GET /rover HTTP/1.1\r\n"\
                            "Host: "\
                            INT2STRING(WEBSOCKET_SERVER_IP_1stOctet)\
                            "."\
                            INT2STRING(WEBSOCKET_SERVER_IP_2ndOctet)\
                            "."\
                            INT2STRING(WEBSOCKET_SERVER_IP_3rdOctet)\
                            "."\
                            INT2STRING(WEBSOCKET_SERVER_IP_4thOctet)\
                            ":"\
                            INT2STRING(WEBSOCKET_SERVER_PORT)\
                            "\r\n"\
                            "Upgrade: websocket\r\n"\
                            "Connection: Upgrade\r\n"\
                            "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"\
                            "Sec-WebSocket-Version: 13\r\n\r\n";
