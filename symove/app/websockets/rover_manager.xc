#include <platform.h>
#include "adc_client_ad7949.h"
#include <string.h>
#include "websocketHandler.h"
#include <print.h>
#include <stdlib.h>
#include <string.h>

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


/* ===================== Helper functions ================================ */
void int2str(int number, char * buffer)
{
    int i = 0;

    unsigned isNeg = number < 0;

    // Make number positive in case it was a negative integer
    number = isNeg ? -number : number;

    // Create ASCII out of number
   while(number != 0)
   {
       buffer[i++] = number % 10 + '0';
       number = number / 10;
    }


   if(isNeg)
       buffer[i++] = '-';

   buffer[i] = '\0';

    //Turn number around
   for(int t = 0; t < i/2; t++)
   {
       buffer[t] ^= buffer[i-t-1];
       buffer[i-t-1] ^= buffer[t];
       buffer[t] ^= buffer[i-t-1];
   }
}
/* ===================================================================== */

/*
 * This function pings the WS server and handles the battery management (not yet implemented)
 * the pings are used by the server to recognize when a rover was turned off
 */
void roverManager(chanend dataSend, chanend c_adc)
{
    timer pingTimer, batteryTimer;
    unsigned timePingTimer, timeBatteryTimer;
    unsigned ping_time = 1000 * MSEC_STD;

    unsigned batteryVoltage = 0;
    static char batteryVoltageStr[8] = {'\0'};

    //Init timer
    pingTimer :> timePingTimer;
    timePingTimer += ping_time;


    while (1)
    {
        select
        {
            case pingTimer when timerafter(timePingTimer) :> void:
            {
                char data[] = "PING";
                websocketSend(data, strlen(data), dataSend);

                int a,b,c;
                for (int i = 0; i < 8; i++)
                {
                    a=0;
                    b=0;
                    c=0;
                    if (i == 1)
                        batteryVoltage = c;
                }

                // Only send voltage if there are at least 50 mV difference between current and last voltage
                if (batteryVoltage < 30000 && batteryVoltage > 0)
                {

                    // Assemble JSON string to send to the server
                    char json2send[18] = "{\"BattVal\":";
                    int2str(batteryVoltage, batteryVoltageStr);
                    strcat(json2send, batteryVoltageStr);
                    strcat(json2send, "}");
                    // Send string
                    websocketSend(json2send, strlen(json2send), dataSend);
                }

                //Timer reset
                pingTimer :> timePingTimer;
                timePingTimer += ping_time;


                break;
           }
        }
    }
}


