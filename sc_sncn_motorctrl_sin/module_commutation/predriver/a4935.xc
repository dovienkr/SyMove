
/**
 *
 * \file a4935.xc
 *
 *	Driver header file for motor
 *
 *
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 * Author: Martin Schwarz <mschwarz@synapticon.com>
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
#include <a4935.h>
#include <print.h>

void a4935_init(int configuration,  out port p_ifm_esf_rstn_pwml_pwmh, out port p_ifm_coastn)
{
	timer timer1;
	unsigned int time1;

	configuration |= A4935_BIT_ESF; // add enable_stop_on_fault to config bits

	// set config pins and trigger reset
	p_ifm_esf_rstn_pwml_pwmh <: configuration;
	//p_ifm_esf_rstn_pwml_pwmh_node_0 <: configuration;

	timer1 :> time1;
	timer1 when timerafter(time1 + (4 * USEC_FAST/*TICKS_US*/)) :> time1; // hold reset for at least 3.5us

	// release reset
	p_ifm_esf_rstn_pwml_pwmh <: ( A4935_BIT_RSTN | configuration );
	//p_ifm_esf_rstn_pwml_pwmh_node_0 <: ( A4935_BIT_RSTN | configuration );

	// pause before enabling FETs after reset
	timer1 when timerafter(time1 + A4935_AFTER_RESET_DELAY) :> time1;

	// enable FETs redundant with watchdog
	p_ifm_coastn <: 1;
	//p_ifm_coastn_node_0 <: 1;
}
