#ifndef _ZEN_H_
#define _ZEN_H_

#include <Arduino.h>
#include "device.h"
#include "machine.h"
#include "leg.h"
#include "cli.h"
extern Leg leg1;
extern Leg leg2;
extern Leg leg3;
extern Leg leg4;
Leg legs[4] = {leg1,leg2,leg3,leg4};

class Zen : public Device
{
public:
    Zen() {}
    void setup() {}
    void loop() {}
    void setupCli(void (*func)(int argc, char **argv))
    {
        cli.newCmd("m", func);
    }
    void onCmd(int arg_cnt, char **args)
    {
        if (!strcmp(args[0], "m"))
        {
            int n = String(args[1]).toInt();
            float ang = String(args[2]).toFloat();
            if( (n<0)||(n>7) ) return;
            legs[n/2].zenWrite(n%2,ang);
        }
    }
    void cliHelp()
    {
        DEBUG_SERIAL.println("-----------------------zen commands-----------------------");
        DEBUG_SERIAL.println("m [motor id] [angle]");
        DEBUG_SERIAL.println("-----------------------");
    }
    char *getName()
    {
        return "zen";
    }
};

Zen zen;

#endif