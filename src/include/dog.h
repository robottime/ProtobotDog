#ifndef _DOG_H_
#define _DOG_H_

#include <Arduino.h>
#include "device.h"
#include "machine.h"
#include "leg.h"
#include "cli.h"
#include "gait.h"

class Dog : public Device {
 public:
  Dog(float init_x = 0, float init_y = 1.6, char *name = "my dog") {
    strcat(_name, name);
    _init_x = init_x;
    _init_y = init_y;
    _connected = 0;
    _t_origin = 0;
    _calibrated = 0;
    _phase = 0;
  }
  void setup() {}
  void loop() {
    if (_t_origin && (millis() - _t_prev) > 30) {
      float x[4], y[4];
      long t = millis() - _t_origin;
      gait(t, 0, x[0], y[0], _gait[0]);
      gait(t, _phase, x[1], y[1], _gait[1]);
      gait(t, 0, x[2], y[2], _gait[2]);
      gait(t, _phase, x[3], y[3], _gait[3]);
      // for (int i = 0; i < 4; i++) {
      //   DEBUG_SERIAL << x[0] << ' ' << y[0] << ", ";
      // }
      // DEBUG_SERIAL.println();
      leg1.pos(x[0], y[0]);
      leg2.pos(x[1], y[1]);
      leg3.pos(x[2], y[2]);
      leg4.pos(x[3], y[3]);
      _t_prev = millis();
    }
  }
  void setupCli(void (*func)(int argc, char **argv)) {
    cli.newCmd("begin", func);
    cli.newCmd("init", func);
    cli.newCmd("walk", func);
    cli.newCmd("step", func);
    cli.newCmd("cal", func);
    cli.newCmd("rot", func);
  }
  void onCmd(int arg_cnt, char **args)
  {
    if (!strcmp(args[0], "begin"))
    {
      begin();
      return;
    }
    if (!_connected) {
      // DEBUG_SERIAL.println("在操作前需要先输入\"begin\"启动串口。");
      return;
    }
    if (!strcmp(args[0], "cal"))
    {
      if (_calibrated) return;
      _calibrated = 1;
      for (int axis = 0; axis < 2; axis++) {
        // leg1.axisCalibrate(axis);
        // delay(100);
        // leg2.axisCalibrate(axis);
        // delay(100);
        leg3.axisCalibrate(axis);
        delay(100);
        // leg4.axisCalibrate(axis);
        // delay(100);
      }
      return;
    }
    if (!strcmp(args[0], "walk"))
    {
      _t_origin = millis();
      _t_prev = _t_origin;
      _gait[0] = forwardGait2;
      _gait[1] = forwardGait2;
      _gait[2] = forwardGait;
      _gait[3] = forwardGait;
      _phase = 0.5;
      return;
    }
    if (!strcmp(args[0], "step"))
    {
      _t_origin = millis();
      _t_prev = _t_origin;
      _gait[0] = stepGait;
      _gait[1] = stepGait;
      _gait[2] = stepGait;
      _gait[3] = stepGait;
      _phase = 0;
      return;
    }
    else {
      _t_origin = 0;
    }
    if (!strcmp(args[0], "init"))
    {
      _init_x = 0;
      _init_y = 1.6;
      if (arg_cnt == 3) {
        _init_x = String(args[1]).toFloat();
        _init_y = String(args[2]).toFloat();
      }
      init();
      return;
    }
    if (!strcmp(args[0], "rot"))
    {
      leg1.axisMiddle(0, 15000);
      delay(5);
      leg2.axisMiddle(0, 15000);
      delay(5);
      leg3.axisMiddle(0, -15000);
      delay(5);
      leg4.axisMiddle(0, -15000);
      delay(5);
      leg1.axisMiddle(1, -15000);
      delay(5);
      leg2.axisMiddle(1, -15000);
      delay(5);
      leg3.axisMiddle(1, 15000);
      delay(5);
      leg4.axisMiddle(1, 15000);
      delay(1500);
      while(1) {
        leg1.axisMiddle(1, -1500);
        delay(1);
        leg2.axisMiddle(1, -1500);
        delay(1);
        leg3.axisMiddle(1, -1500);
        delay(1);
        leg4.axisMiddle(1, -1500);
        delay(7);
      }
      // leg1.axisVel(1, -15000);
      // delay(5);
      // leg2.axisVel(1, -15000);
      // delay(5);
      // leg3.axisVel(1, -15000);
      // delay(5);
      // leg4.axisVel(1, -15000);
      // delay(5);
      return;
    }
  }
    void cliHelp()
    {
      DEBUG_SERIAL.println("-----------------------dog commands-----------------------");
      DEBUG_SERIAL.println("连接：begin");
      DEBUG_SERIAL.println("初始化：init");
      DEBUG_SERIAL.println("-----------------------");
    }
    char *getName()
    {
      char tmp[20];
      strcpy(tmp, _name);
      if (_connected)
        return strcat(tmp, " on");
      return strcat(tmp, " off");
    }

  public:
   void begin() {
     if (_connected) return;
     leg1.begin();
     leg2.begin();
     leg3.begin();
     leg4.begin();
     _connected = 1;
     delay(100);
   }
   void init()
   {
     leg1.pos(_init_x, _init_y);
     delay(500);
     leg2.pos(_init_x, _init_y);
     delay(500);
     leg3.pos(_init_x, _init_y);
     delay(500);
     leg4.pos(_init_x, _init_y);
   }

 private:
  char _name[20];
  float _init_x;
  float _init_y;
  bool _connected;
  bool _calibrated;
  float _phase;
  long _t_prev;
  long _t_origin;
  gaitParam _gait[4];
};

Dog dog;

#endif