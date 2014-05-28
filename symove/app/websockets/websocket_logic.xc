#include <platform.h>
#include <string.h>
#include <stdlib.h>
#include "jsmn.h"


void parseJson(char jsonString[], chanend xy)
{
    char value[7] = "\0";
    unsafe
    {
        unsigned int *test;
        jsmn_parser p;
        jsmntok_t tokens[10];

        jsmn_init(&p);
        jsmn_parse(&p, jsonString, strlen(jsonString), tokens, 10);

        int i, j;
        int x, y;
        for (i = tokens[2].start, j = 0; i < tokens[2].end; i ++, j++){
          value[j] = jsonString[i];
          if (j == tokens[2].end - tokens[2].start - 1)
              value[j+1] = '\0';
        }
        x = atoi(value);

        for (i = tokens[4].start, j = 0; i < tokens[4].end; i ++, j++){
          value[j] = jsonString[i];
          if (j == tokens[4].end - tokens[4].start - 1)
              value[j+1] = '\0';
        }
        y = atoi(value);

        // Send the received values over the channel
        xy <: x;
        xy <: y;
    }
}

// Websocket receive function
void receive(chanend dataReceive, chanend xy)
{
    char buffer[125] = "\0";
    int len = 0;
    int cnt = 0;
    char data;

    int initialized = 0;

    while (1)
    {
        select {
            case dataReceive :> data : //break;

                if (len == 0)
                    len = (int) data;
                else
                {
                    buffer[cnt] = data;
                    cnt ++;

                    if (cnt == len)
                    {
                        buffer[cnt] = '\0';
                        parseJson(buffer, xy);
                        cnt = 0;
                        len = 0;
                    }

                }
            break;
        }
    }
}
