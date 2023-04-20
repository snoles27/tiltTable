#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "servos.h"
#include <string>

//setting numbers that are useful for talking about anlges in radians
#define PI 3.1415926
#define PIHALF 1.570796

//pwm clock settings for both servos
const uint16_t wrap = 18750;
const uint16_t initLevel = 6250;
const uint8_t clkdiv_int = 20;
const uint8_t clkdiv_frac = 0;

//servo specific parameters, tested with power supply variety of voltages 4/19/23 Samuel Noles
//servo 0 --> xaxis (might say 2 on it)
//servo 1 --> yaxis
const int servoPin[2] = {14, 15};
const double servo_angleMin[2] = {-PIHALF, -PIHALF};
const int servo_levelMin[2] = {13572, 5600};
const double servo_angleMax[2] = {PIHALF, PIHALF};
const int servo_levelMax[2] = {5576,13895};

void setPosition(int servoNum, double angle, bool manual){

    // make sure the set angle is within the allowed range. If not set to max or min. 
    if(manual){
        if(angle > servo_angleMax[servoNum]){
            angle = servo_angleMax[servoNum];
        } else if (angle < servo_angleMin[servoNum]) {
            angle = servo_angleMin[servoNum];
        } else {}
    }
    //get slice and channel for servoPin
    unsigned int slice=pwm_gpio_to_slice_num(servoPin[servoNum]);
    unsigned int channel=pwm_gpio_to_channel(servoPin[servoNum]);

    //get level2angle conversion
    double level2angle;
    level2angle = (double)(servo_levelMax[servoNum] - servo_levelMin[servoNum])/(servo_angleMax[servoNum]-servo_angleMin[servoNum]);
    int level;
    level = (int)(level2angle * (angle - servo_angleMin[servoNum])) + servo_levelMin[servoNum];

    // // printing level for debuging
    if(manual){
        std::string levelPrint = std::to_string(level);
        printf(levelPrint.c_str());
        printf("\n");
    }

    pwm_set_chan_level(slice, channel, level);

}

void initServo(int servoNum) {

    gpio_set_function(servoPin[servoNum], GPIO_FUNC_PWM);
    unsigned int slice=pwm_gpio_to_slice_num(servoPin[servoNum]);
    unsigned int channel=pwm_gpio_to_channel(servoPin[servoNum]);

    pwm_set_clkdiv_int_frac(slice, clkdiv_int, clkdiv_frac);
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, channel, initLevel);
    pwm_set_phase_correct(slice, 0);

    pwm_set_enabled(slice, 1); //start servo;
    setPosition(servoNum, 0.0, false); //set angle to 0

}

void sweepServos(float initServo1, float initServo2, float finalServo1, float finalServo2, double seconds){

    int numSteps = 100; //number of steps to break the sweep into
    int timeStep;
    timeStep = (seconds/numSteps) * 1000000.0; //time step in us
    float servo1Step;
    servo1Step = (finalServo1 - initServo1)/numSteps;
    float servo2Step;
    servo2Step = (finalServo2 - initServo2)/numSteps;

    //move servos to inital position
    setPosition(0, initServo1, false);
    setPosition(1, initServo2, false);

    for(int i = 1; i <= numSteps; i++){
        setPosition(0, initServo1 + i * servo1Step, false);
        setPosition(1, initServo2 + i * servo2Step, false);
        sleep_us(timeStep);
    }

}


