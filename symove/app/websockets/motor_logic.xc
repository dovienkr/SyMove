#include <platform.h>
//#include "dual_drive.h"
#include "print.h"

/* ================= Initializations ==================== */
// Bit mapping of 4-bit A4935 config port
//#define A4935_AFTER_RESET_DELAY (200 * MSEC_FAST/*TICKS_MS*/) // 200ms

/* 10NSEC_STD --> 1 */
#define USEC_STD       100
#define MSEC_STD    100000
#define SEC_STD  100000000

#if PLATFORM_REFERENCE_MHZ == 100
    #define USEC_FAST USEC_STD
    #define MSEC_FAST MSEC_STD
    #define SEC_FAST  SEC_STD
#else /* REFCLK_STD == 100 MHZ , REFCLK_FAST == 250 MHZ */
    /* 4NSEC_FAST --> 1 */
    #define USEC_FAST       250
    #define MSEC_FAST    250000
    #define SEC_FAST  250000000
#endif

#define PWM_PERIOD 2000
/*on stdcore[3]: bldc_bridge_t bridge=
        {
        PWM_PERIOD, //PWM PERIOD
        XS1_CLKBLK_2, //CLKBLK
        125, //CLK DIVIDEND
        5,  //CLK DIVISOR
        {
              XS1_PORT_1K, // Hi A
              XS1_PORT_1O, // Hi B
              XS1_PORT_1M,  // Hi C

              XS1_PORT_1L,   // Lo A
              XS1_PORT_1P,   // Lo B
              XS1_PORT_1N    // Lo C
        }
};
*/

//Maximum speed for the motors (used in driveMotors())
#define MAX_SPEED 1500

// ENABLE / DISABLE OF DUALDRIVE in general
#define DUALDRIVE_ACTIVE 1

//Region in which dual drive is active
#define DUALDRIVE_Y_POS_RANGE 7
#define DUALDRIVE_Y_NEG_RANGE -7

extern out port p_ifm_esf_rstn_pwml_pwmh;
extern out port p_ifm_coastn;

/* ========================================================= */


int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    int result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if (result > out_max)
        result = out_max;
    else if (result < out_min)
        result = out_min;

    return result;
}


