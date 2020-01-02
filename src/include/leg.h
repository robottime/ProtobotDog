#ifndef _LEG_H_
#define _LEG_H_

#include <Arduino.h>
#include "device.h"
#include <math.h>
#include <ODriveArduino.h>
#include "machine.h"
#include "cli.h"
#include "gait.h"

struct legParam
{
  float vel_limit;
  float vel_limit_tolerance;
  float pos_gain;
  float vel_gain;
  float vel_integrator_gain;
  int control_mode;
  int pole_pairs;
  float current_lim;
  float current_lim_tolerance;
  float calibration_current;
  float inverter_temp_limit_upper;
  int cpr;
  float requested_current_range;
};

legParam kLarmCfg = {
    .vel_limit = 50000,
    .vel_limit_tolerance = 2,
    .pos_gain = 18,                // 18
    .vel_gain = 0.0002,            // 0.0002
    .vel_integrator_gain = 0.0004, // 0.001
    .control_mode = 3,
    .pole_pairs = 6,
    .current_lim = 13,
    .current_lim_tolerance = 2,
    .calibration_current = 10,
    .inverter_temp_limit_upper = 10000,
    .cpr = 4000,
    .requested_current_range = 90,
};

legParam kUarmCfg = {
    .vel_limit = 50000,
    .vel_limit_tolerance = 2,
    .pos_gain = 18,
    .vel_gain = 0.0002,
    .vel_integrator_gain = 0.0004,
    .control_mode = 3,
    .pole_pairs = 6,
    .current_lim = 13,
    .current_lim_tolerance = 2,
    .calibration_current = 10,
    .inverter_temp_limit_upper = 10000,
    .cpr = 4000,
    .requested_current_range = 90
};

class Leg : public Device {
 public:
  Leg(HardwareSerial *serial, char *name="default leg", bool dir = 0) {
    _serial = serial;
    _odrive = new ODriveArduino(*_serial);
    strcat(_name, name);
    _dir = dir;
    _middle[0] = 0;
    _middle[1] = 0;
    _connected = 0;
  }
  void setup() {
    _t_origin = 0;
    _enable_motor[0] = 0;
    _enable_motor[1] = 0;
  }
  void loop() {
    if (_t_origin && (millis() - _t_prev) > 30)
    {
      float x, y;
      long t = millis() - _t_origin;
      gait(t, 0, x, y, _gait);
      pos(x, y);
      _t_prev = millis();
    }
  }
  char *getName() {
    char tmp[20];
    strcpy(tmp, _name);
    if (_connected) return strcat(tmp, " on");
    return strcat(tmp, " off");
  }
  void setupCli(void (*func)(int argc, char **argv)) {
    cli.newCmd("cal", func);
    cli.newCmd("mid", func);
    cli.newCmd("mv", func);
    cli.newCmd("begin", func);
    cli.newCmd("dir", func);
    cli.newCmd("pos", func);
    cli.newCmd("area", func);
    cli.newCmd("step", func);
  }
  void cliHelp()
  {
    DEBUG_SERIAL.println("-----------------------leg commands-----------------------");
    DEBUG_SERIAL.println("连接：begin");
    DEBUG_SERIAL.println("显示运动范围：area");
    DEBUG_SERIAL.println("校准：cal [axis]");
    DEBUG_SERIAL.println("复位：mid [axis] [angle]");
    DEBUG_SERIAL.println("运动测试：mv (times)");
    DEBUG_SERIAL.println("设置方向：dir [direction]");
    DEBUG_SERIAL.println("运动到：pos [x] [y]");
    DEBUG_SERIAL.println("-----------------------");
  }
  void onCmd(int arg_cnt, char **args) {
    if (!strcmp(args[0], "begin"))
    {
      begin();
      return;
    }
    if (!strcmp(args[0], "area"))
    {
      plotArea();
      return;
    }
    if(!_connected) {
      DEBUG_SERIAL.println("在操作前需要先输入\"begin\"启动串口。");
      return;
    }
    if (!strcmp(args[0], "cal")) {
      if (arg_cnt < 1) return;
      axisCalibrate(String(args[1]).toInt());
      return;
    }
    if (!strcmp(args[0], "mid")) {
      if (arg_cnt < 2) return;
      axisMiddle(String(args[1]).toInt(), String(args[2]).toInt());
    }
    if (!strcmp(args[0], "mv")) {
      int n = 5;
      if (arg_cnt >= 1) n = String(args[1]).toInt();
      for (int i = 0; i < n; i++)
      {
          write(30000, -15000);
          delay(2000);
          write(0, 0);
          DEBUG_SERIAL << "第" << i + 1 << "次完成" << '\n';
          delay(2000);
      }
    }
    if (!strcmp(args[0], "dir"))
    {
      if (arg_cnt < 1) return;
      setDir(String(args[1]).toInt());
    }
    if (!strcmp(args[0], "pos"))
    {
      if (arg_cnt < 2)
        return;
      pos(String(args[1]).toFloat(), String(args[2]).toFloat());
    }
    if (!strcmp(args[0], "step"))
    {
      if (arg_cnt > 1) {
        int m = String(args[1]).toInt();
        DEBUG_SERIAL.println(m);
        if (m == 0) {
          _enable_motor[0] = 1;
          _enable_motor[1] = 0;
        }
        else if (m == 1) {
          _enable_motor[0] = 0;
          _enable_motor[1] = 1;
        }
      }
      _t_origin = millis();
      _t_prev = _t_origin;
      _gait = stepGait;
      return;
    }
    else
    {
      _t_origin = 0;
      _enable_motor[0] = 1;
      _enable_motor[1] = 1;
    }
  }
  void begin() {
    if (_connected)
      return;
    _serial->begin(115200);
    while (!_serial)
    {
    }
    delay(10);
    *_serial << "w config.dc_bus_undervoltage_trip_level " << 0 << '\n';
    delay(10);
    axisConfig(0, kUarmCfg);
    axisConfig(1, kLarmCfg);
    _connected = 1;
    return;
  }
  void write(float c_alpha, float c_beta) {
    if (_enable_motor[0]) axisWrite(0, _middle[0] + c_alpha);
    if (_enable_motor[1]) axisWrite(1, _middle[1] + c_beta);
  }
  void pos(float x, float y) {
    float alpha, beta;
    x = _dir ? x : -x;
    if(!cartesianToAlphaBeta(x, y, alpha, beta, _dir)) {
      write(radToCount(alpha), radToCount(beta));
      // DEBUG_SERIAL << _name << ": " <<  radToCount(alpha) << ", " << radToCount(beta) << '\n';
    }
  }

  void zenWrite(int axis,float ang)
  {
    if (!validAxis(axis)) return;
    if( (axis==0)&&((ang>M_PI/2)||(ang<-M_PI/2)) ) return;
    axisWrite(axis, _middle[axis] + radToCount(ang));
  }

  void setDir(bool dir) {
    _dir = dir;
  }
  void axisCalibrate(int axis) {
    if (!validAxis(axis)) return;
    DEBUG_SERIAL.println("校准中...");
    _odrive->run_state(axis, ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE, false);
    delay(15000);
    _odrive->run_state(axis, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false);
     _enable_motor[axis] = 1;
    DEBUG_SERIAL.println("校准完成");
  }
  void axisWrite(int axis, float pos) {
    if (!validAxis(axis)) return;
    _odrive->SetPosition(axis, pos);
    delay(1);
  }
  void axisMiddle(int axis, float offset) {
    if (!validAxis(axis)) return;
    _odrive->SetPosition(axis, _middle[axis] += offset);
    DEBUG_SERIAL<<"axis "<< axis << " middle: " << _middle[axis] << '\n';
  }
  void axisVel(int axis, float vel) {
    if (!validAxis(axis)) return;
    _odrive->SetVelocity(axis, vel);
  }
  float getAxisMiddle(int axis) {
    if (!validAxis(axis)) return 0;
    return _middle[axis];
  }
  void plotArea() {
    float alpha, beta;
    for (float y = 0; y <= 2; y += 0.1) {
      for (float x = -2; x <= 2; x  += 0.1) {
        DEBUG_SERIAL << cartesianToAlphaBeta(x, y, alpha, beta, _dir) << ' ';
      }
      DEBUG_SERIAL.println();
    }
  }
 private:
  HardwareSerial *_serial;
  ODriveArduino *_odrive;
  char _name[20];
  int _connected;
  float _middle[2];
  bool _dir;
  long _t_origin;
  long _t_prev;
  gaitParam _gait ;
  bool _enable_motor[2];
  void axisConfig(int axis, legParam config = kUarmCfg)
  {
    if (!validAxis(axis)) return;
    *_serial << "w axis" << axis << ".controller.config.vel_limit " << config.vel_limit << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".controller.config.vel_limit_tolerance " << config.vel_limit_tolerance << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".controller.config.pos_gain " << config.pos_gain << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".controller.config.vel_gain " << config.vel_gain << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".controller.config.vel_integrator_gain " << config.vel_integrator_gain << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".controller.config.control_mode " << config.control_mode << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".motor.config.pole_pairs " << config.pole_pairs << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".motor.config.current_lim " << config.current_lim << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".motor.config.current_lim_tolerance " << config.current_lim_tolerance << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".motor.config.inverter_temp_limit_upper " << config.inverter_temp_limit_upper << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".motor.config.calibration_current " << config.calibration_current << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".motor.config.requested_current_range " << config.requested_current_range << '\n';
    delay(10);
    *_serial << "w axis" << axis << ".encoder.config.cpr " << config.cpr << '\n';
    delay(10);
  }
  inline bool validAxis(int axis) {
    return axis == 0 || axis == 1;
  }
  int intersection(float x, float y, float &ix1, float &iy1, float &ix2, float &iy2)
  {
    float d = sqrt(x * x + y * y);
    if (d == 0)
    {
      return 1;
    }
    if (d > 2)
    {
      return 2;
    }
    float a = d / 2.0;
    float h = sqrt(1 - a * a);
    float tx = a * x / d;
    float ty = a * y / d;
    ix1 = tx + h * y / d;
    iy1 = ty - h * x / d;
    ix2 = tx - h * y / d;
    iy2 = ty + h * x / d;
    return 0;
  }

  int cartesianToAlphaBeta(float x, float y, float &alpha, float &beta, bool dir = 0)
  {
    float ix1, iy1, ix2, iy2;
    if (!intersection(x, y, ix1, iy1, ix2, iy2))
    {
      float ix = ix1, iy = iy1;
      if (dir)
      {
        ix = ix2;
        iy = iy2;
      }
      alpha = atan2(iy, ix);
      if (alpha < 0) return 2;
      beta = atan2(y - iy, x - ix) - alpha;
      if (beta < -M_PI) beta += 2*M_PI;
      alpha -= M_PI/2;
      return 0;
    }
    if (x == 0 && y == 0)
    {
      alpha = 0;
      beta = M_PI;
      return 0;
    }
    return 1;
  }

  inline int radToCount(float rad) {
    static float M_RATIO = 3.5;
    static int CPR = 4000;
    static float K = M_RATIO*M_RATIO*M_RATIO*CPR / 2.0 / M_PI;
    return K * rad;
  }
};

Leg leg1(&serial_leg1, "leg1",1);
Leg leg2(&serial_leg2, "leg2",0);
Leg leg3(&serial_leg3, "leg3",0);
Leg leg4(&serial_leg4, "leg4",1);

#endif
