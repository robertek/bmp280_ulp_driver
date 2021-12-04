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
	move r3,stackEnd

	/* halt if counter++ < period */
	move r1,counter
	ld r0,r1,0
	add r0,r0,1
	st r0,r1,0
	move r1,period
	ld r2,r1,0
	sub r0,r2,r0
	jumpr waitNext,0,gt

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

	/* wakeUp if r0 < 0 */ 
	jump testPres,OV
	jump wakeUp

testPres:
	/* r0 = pres_msb << 8 | pres_lsb */
	move r1,pres_msb
	ld r0,r1,0
	lsh r0,r0,8
	move r1,pres_lsb
	ld r1,r1,0
	or r0,r1,r0

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
