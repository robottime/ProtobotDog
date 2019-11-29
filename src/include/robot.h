#ifndef _ROBOT_H_
#define _ROBOT_H_

#include "device.h"
#include "machine.h"
#include "cli.h"
#include "switch.h"
#include "leg.h"
#include "dog.h"

enum device_t
{
  DOG,
  LEG1,
  LEG2,
  LEG3,
  LEG4,
  GYRO,
  DEVICE_NUM,
};

namespace robot {
  device_t cur_device = DOG;
  Device *devices[DEVICE_NUM] = {
    &dog,
    &leg1,
    &leg2,
    &leg3,
    &leg4,
    NULL,
  };
  char *device_names[DEVICE_NUM] = {
    "dog",
    "leg1",
    "leg2",
    "leg3",
    "leg4",
    "gyro",
  };

  void dispatchCmd(int arg_cnt, char **args)
  {
    if (!devices[cur_device]) {
      DEBUG_SERIAL.println("device is null");
      return;
    }
    devices[cur_device] -> onCmd(arg_cnt, args);
    delay(80);
    cli.setPath(devices[cur_device]->getName());
  }

  void changeDev(int arg_cnt, char **args) {
    if (arg_cnt < 1) {
      DEBUG_SERIAL.println("Invalid input, type \"help\" for more information.");
      return;
    }
    for (device_t d = device_t(0); d < DEVICE_NUM; d = device_t(d + 1)) {
      if (!strcmp(device_names[d], args[1])) {
        cur_device = d;
        if(devices[d]) cli.setPath(devices[d]->getName());
        else cli.setPath(args[1]);
        return;
      }
    }
    DEBUG_SERIAL.println("Invalid input, type \"help\" for more information.");
  }

  void help(int arg_cnt, char **args) {
    DEBUG_SERIAL.println("-----------------------global commands-----------------------");
    DEBUG_SERIAL.println("设备选择命令：cd [device]");
    DEBUG_SERIAL.println("-- [device] value:dog,leg1,leg2,leg3,leg4,gyro");
    DEBUG_SERIAL.println("-- e.g. cd leg1 => 选择设备：1号腿");
    if (!devices[cur_device])
    {
      DEBUG_SERIAL.println("device is null");
      return;
    }
    devices[cur_device]->cliHelp();
  }

  void setup()
  {
    setup_machines();
    if(devices[cur_device]) {
      cli.setup(&DEBUG_SERIAL, devices[cur_device]->getName());
    }
    else {
      cli.setup(&DEBUG_SERIAL);
    }
    dog_switch.setup();
    cli.newCmd("cd", changeDev);
    cli.newCmd("help", help);

    for (device_t d = device_t(0); d < DEVICE_NUM; d = device_t(d + 1))
    {
      if (!devices[d])
        continue;
      Device *device = devices[d];
      device->setup();
      device->setupCli(dispatchCmd);
      delay(100);
    }
  }

  void loop() {
    cli.read();
    dog_switch.read();
    if (!devices[cur_device])
    {
      return;
    }
    devices[cur_device]->loop();
  }

}

#endif