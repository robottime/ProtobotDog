#ifndef _SWITCH_H_
#define _SWITCH_H_

#define SWITCH_1_PIN 8
#define SWITCH_2_PIN 12
#define SWITCH_3_PIN 3

#include <Arduino.h>
#include "dog.h"
#include "machine.h"

class Switch
{
 private:
  int _pos;
  void switchCallback1();
  void switchCallback2();
  void switchCallback3();
public:
  Switch();
  void setup();
  void read();
};

Switch::Switch()
{
  _pos = 0;
}

void Switch::setup()
{
  pinMode(SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(SWITCH_2_PIN, INPUT_PULLUP);
  pinMode(SWITCH_3_PIN, INPUT_PULLUP);
}
 
void Switch::read() {
  if(!digitalRead(SWITCH_1_PIN)) {
    if(_pos != 1) {
      _pos = 1;
      switchCallback1();
    }
  }
  else if (!digitalRead(SWITCH_2_PIN))
  {
    if (_pos != 2) {
      _pos = 2;
      switchCallback2();
    }
  }
  else if (!digitalRead(SWITCH_3_PIN))
  {
    if (_pos != 3) {
      _pos = 3;
      switchCallback3();
    }
  }
  else {
    if (digitalRead(SWITCH_1_PIN) && digitalRead(SWITCH_2_PIN) && digitalRead(SWITCH_3_PIN))
    {
      _pos = 0;
    }
  }
}

void Switch::switchCallback1() {
  char *begin_cmd[1] = {"begin"};
  char *cal_cmd[1] = {"cal"};
  dog.onCmd(1, begin_cmd);
  delay(200);
  dog.onCmd(1, cal_cmd);
}

void Switch::switchCallback2()
{
  char *init_cmd[1] = {"init"};
  char *step_cmd[1] = {"step"};
  // char *rot_cmd[1] = {"rot"};
  DEBUG_SERIAL.println("准备！");
  dog.onCmd(1, init_cmd);
  delay(5000);
  DEBUG_SERIAL.println("开始！");
  dog.onCmd(1, step_cmd);
  // dog.onCmd(1, rot_cmd);
}

void Switch::switchCallback3()
{
  char *init_cmd[3] = {"init", "-0.2", "1.2"};
  char *walk_cmd[1] = {"walk"};
  dog.onCmd(3, init_cmd);
  delay(5000);
  dog.onCmd(1, walk_cmd);
}

Switch dog_switch;

#endif
