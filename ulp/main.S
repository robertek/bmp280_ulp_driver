/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Robert David <robert.david@posteo.net>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "stack.S"


/* Define variables, which go into .bss section (zero-initialized data) */
.bss

.global prev_pres, prev_temp
prev_pres: .long 0
prev_temp: .long 0

.global t_diff, p_diff
t_diff: .long 0
p_diff: .long 0

.global period
period: .long 0

.global	counter
counter: .long 0

.global stack
stack: .skip 100
.global stackEnd
stackEnd: .long 0


/* Code goes into .text section */
.text
.global entry
entry:
	/* halt if counter++ < period */
	move r1,counter
	ld r0,r1,0
	add r0,r0,1
	st r0,r1,0
	move r1,period
	ld r2,r1,0
	sub r0,r2,r0
	jumpr waitNext,0,gt

	/* initialize the stack */
	move r3,stackEnd

	/* reset counter */
	move r1,counter
	move r0, 0
	st r0,r1,0

	/* read the BMP280 */
	psr
	jump readBMP

.global testTemp
testTemp:
	/* r0 = temp_msb << 8 | temp_lsb */
	move r1,temp_msb
	ld r0,r1,0
	lsh r0,r0,8
	move r1,temp_lsb
	ld r1,r1,0
	or r0,r1,r0

	/* save r0 on stack */
	push r0

	/* r0 = abs(r0 - prev_temp) */
	move r1,prev_temp
	ld r2,r1,0
	sub r0,r0,r2
	psr
	jump abs

	/* r0 = r0 - t_diff */
	move r1,t_diff
	ld r2,r1,0
	sub r0,r0,r2

	/* continue if r0 < 0 */ 
	jump testPres,OV

	/* store new prev_temp*/
	pop r0
	move r1,prev_temp
	st r0,r1,0

	jump wakeUp

testPres:
	/* pop the stack */
	add r3,r3,1

	/* r0 = pres_msb << 8 | pres_lsb */
	move r1,pres_msb
	ld r0,r1,0
	lsh r0,r0,8
	move r1,pres_lsb
	ld r1,r1,0
	or r0,r1,r0

	/* save r0 on stack */
	push r0

	/* r0 = abs(r0 - prev_pres) */
	move r1,prev_pres
	ld r2,r1,0
	sub r0,r0,r2
	psr
	jump abs

	/* r0 = r0 - p_diff */
	move r1,p_diff
	ld r2,r1,0
	sub r0,r0,r2

	/* waitNext if r0 < 0 */
	jump waitNext,OV

	/* store new prev_pres*/
	pop r0
	move r1,prev_pres
	st r0,r1,0

wakeUp:
	/* Check if the system can be woken up */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump wakeUp, eq

	/* Wake up the SoC, end program */
	wake

	/* Stop the wakeup timer so it does not restart ULP */
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)

waitNext:
	halt

/*
 * Compute abs value of R0
 */
abs:
	and r1,r0,0x8000
	jump noNegate,eq
	move r1,0
	sub r0,r1,r0
noNegate:
	ret
