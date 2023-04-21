#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "servos.h"
#include <string>

//defining useful numbers
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
    //moves servo to set angle using parameters hardcoded at top of file

    //inputs//
    //servoNum: 0 or 1 sepecifying which servo to move
    //angle: angle in radians to move the servo
    //maunal: boolean variable which has function do more checks before it moves it

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

    //convert the angle to pwm level
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

    //set the channel level
    pwm_set_chan_level(slice, channel, level);

}

void initServo(int servoNum) {
    // initializes pwm for servo motion using hardcoded parameters at top of page

    //inputs//
    //servoNum: servo index to set up

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

void sweepServos(float initServo0, float initServo1, float finalServo0, float finalServo1, double seconds){
    //sweeps servos simultaneously from inital angles to final angle in set time

    //inputs//
    //initServo0: inital angle of servo 0 (radians)
    //initServo1: intial angle of servo 1 (radians)
    //finalServo0: final angle of servo 0 (radians)
    //finalServo1: final angle of servo 1 (radians)
    //seconds: time to perform move (seconds)

    int numSteps = 100; //number of steps to break the sweep into

    int timeStep; //time step in us
    timeStep = (seconds/numSteps) * 1000000.0; 
    float servo0Step; //amount to change servo angle 0 angle per time step 
    servo0Step = (finalServo0 - initServo0)/numSteps; 
    float servo1Step; //amount to change servo angle 1 angle per time step 
    servo1Step = (finalServo1 - initServo1)/numSteps;

    //move servos to inital position (often will already be there)
    setPosition(0, initServo0, false);
    setPosition(1, initServo1, false);

    for(int i = 1; i <= numSteps; i++){
        setPosition(0, initServo0 + i * servo0Step, false);
        setPosition(1, initServo1 + i * servo1Step, false);
        sleep_us(timeStep);
    }

}


