#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
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
const int servoPin[2] = {15, 14};
const double servo_angleMin[2] = {-PIHALF, -PIHALF};
const int servo_levelMin[2] = {13895, 13572};
const double servo_angleMax[2] = {PIHALF, PIHALF};
const int servo_levelMax[2] = {5600, 5576};

void initServo(int servoNum) {

    gpio_set_function(servoPin[servoNum], GPIO_FUNC_PWM);
    unsigned int slice=pwm_gpio_to_slice_num(servoPin[servoNum]);
    unsigned int channel=pwm_gpio_to_channel(servoPin[servoNum]);

    pwm_set_clkdiv_int_frac(slice, clkdiv_int, clkdiv_frac);
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, channel, initLevel);
    pwm_set_phase_correct(slice, 0);

    pwm_set_enabled(slice, 1);

}

void setPosition(int servoNum, double angle){

    // make sure the set angle is within the allowed range. If not set to max or min. 
    // if(angle > servo_angleMax[servoNum]){
    //     angle = servo_angleMax[servoNum];
    // } else if (angle < servo_angleMin[servoNum]) {
    //     angle = servo_angleMin[servoNum];
    // } else {}

    //get slice and channel for servoPin
    unsigned int slice=pwm_gpio_to_slice_num(servoPin[servoNum]);
    unsigned int channel=pwm_gpio_to_channel(servoPin[servoNum]);

    //get level2angle conversion
    double level2angle;
    level2angle = (double)(servo_levelMax[servoNum] - servo_levelMin[servoNum])/(servo_angleMax[servoNum]-servo_angleMin[servoNum]);
    int level;
    level = (int)(level2angle * (angle - servo_angleMin[servoNum])) + servo_levelMin[servoNum];

    // // printing level for debuging
    std::string levelPrint = std::to_string(level);
    printf(levelPrint.c_str());
    printf("\n");

    pwm_set_chan_level(slice, channel, level);

}

int main() {

    
    initServo(0);
    initServo(1);

    stdio_init_all();

    float angleRead = 0.0;
    int servoNumber = 0;

    while (1) {

        // printf("Enter a servo number: \n");
        // scanf("%d", &servoNumber);
        printf("Enter an number angle: \n");
        scanf("%f", &angleRead);

        setPosition(0, angleRead);
        setPosition(1, angleRead);
        sleep_ms(500);

     }

}