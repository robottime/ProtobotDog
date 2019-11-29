#ifndef _MACHINE_H_
#define _MACHINE_H_

#include <Arduino.h>
#include <soft_uart.h>

using namespace arduino_due;

// Printing with stream operator
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
    obj.print(arg);
    return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
    obj.print(arg, 4);
    return obj;
}

#define RX_BUF_LENGTH 256 // software serial port's reception buffer length
#define TX_BUF_LENGTH 2048 // software serial port's transmision buffer length

#define DEBUG_SERIAL Serial
#define GYRO_SERIAL Serial2

serial_tc7_declaration(RX_BUF_LENGTH, TX_BUF_LENGTH, 30, A0);
serial_tc8_declaration(RX_BUF_LENGTH, TX_BUF_LENGTH, 31, A1);
serial_tc1_declaration(RX_BUF_LENGTH, TX_BUF_LENGTH, 32, A4);
serial_tc6_declaration(RX_BUF_LENGTH, TX_BUF_LENGTH, 33, A5);

auto &serial_leg1 = serial_tc7; // serial_tc2_t& serial_leg2=serial_tc2;
auto &serial_leg2 = serial_tc8;
auto &serial_leg3 = serial_tc1;
auto &serial_leg4 = serial_tc6;

void setup_machines() {
    DEBUG_SERIAL.begin(9600);
    GYRO_SERIAL.begin(115200);

    while (!DEBUG_SERIAL) {}
    delay(200);
}

#endif