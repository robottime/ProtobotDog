#ifndef _DEVICE_H_
#define _DEVICE_H_

class Device {
 public:
  virtual void setup() = 0;
  virtual void loop() = 0;
  virtual void setupCli(void (*func)(int argc, char **argv)) {}
  virtual void onCmd(int arg_cnt, char **args) = 0;
  virtual void cliHelp() = 0;
  virtual char *getName() = 0;
};

#endif