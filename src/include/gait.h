#ifndef _GAIT_H_
#define _GAIT_H_

#include <Arduino.h>
#include <math.h>

struct gaitParam {
    float x_origin;
    float y_origin;
    float amp;
    float half_cycle;
    float period;
    float flight_percent;
};

// gaitParam forwardGait = {
//     .x_origin = -0.3,
//     .y_origin = 1.2,
//     .amp = -0.4,
//     .half_cycle = 0.3,
//     .period = 800,
//     .flight_percent = 0.5
// };

// gaitParam forwardGait2 = {
//     .x_origin = -0.55,
//     .y_origin = 1.2,
//     .amp = -0.4,
//     .half_cycle = 0.3,
//     .period = 800,
//     .flight_percent = 0.5};

gaitParam forwardGait = {
    .x_origin = -0.4,
    .y_origin = 1.5,
    .amp = -0.4,
    .half_cycle = 0.5,
    .period = 600, // 1000
    .flight_percent = 0.5};

gaitParam forwardGait2 = {
    .x_origin = -0.9,
    .y_origin = 1.5,
    .amp = -0.4,
    .half_cycle = 0.5,
    .period = 600,
    .flight_percent = 0.5};

gaitParam backwardGait = {
    .x_origin = 0.5,
    .y_origin = 1.4,
    .amp = -0.2,
    .half_cycle = -1,
    .period = 700,
    .flight_percent = 0.6
};

gaitParam backwardGait2 = {
    .x_origin = 0.2,
    .y_origin = 0.4,
    .amp = -0.2,
    .half_cycle = -0.4,
    .period = 700,
    .flight_percent = 0.6};

gaitParam stepGait = {
    .x_origin = -0.2,
    .y_origin = 1.6, 
    .amp = -0.5,       // -0.5 -1
    .half_cycle = 0.01,
    .period = 1600, // 1600 2600
    .flight_percent = 0.6};

gaitParam stepGait2 = {
    .x_origin = 0.2, // 0
    .y_origin = 1.6,
    .amp = -0.3, // -0.3
    .half_cycle = 0.01,
    .period = 500, // 600
    .flight_percent = 0.45
};

void gait(float millis, float offset, float &x, float &y, gaitParam param = backwardGait)
{
    offset = fmod(offset, 1.0);
    float phase = fmod(millis/param.period + offset, 1.0);
    if (phase < param.flight_percent) {
        float dx = param.half_cycle * phase / param.flight_percent;
        float dy = param.amp * sin(M_PI / param.half_cycle * dx);
        x = param.x_origin + dx;
        y = param.y_origin + dy;
    } else {
        x = param.x_origin + param.half_cycle * (1 - phase) / (1 - param.flight_percent);
        y = param.y_origin;
    }
}

#endif
