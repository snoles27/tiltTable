#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

//pwm clock settings for both servos
const uint16_t wrap = 18750;
const uint16_t initLevel = 6250;
const uint8_t clkdiv_int = 20;
const uint8_t clkdiv_frac = 0;

//servo specific parameters
const int servoPin[2] = {15, 14};
const int servo_angleMin[2] = {0, 0};
const int servo_levelMin[2] = {3125, 3125};
const int servo_angleMax[2] = {270, 270};
const int servo_levelMax[2] = {15625, 15625};

void initServo(int servoNum) {

    gpio_set_function(servoPin[servoNum], GPIO_FUNC_PWM);
    uint slice=pwm_gpio_to_slice_num(servoPin[servoNum]);
    uint channel=pwm_gpio_to_channel(servoPin[servoNum]);

    pwm_set_clkdiv_int_frac(slice, clkdiv_int, clkdiv_frac);
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, channel, initLevel);
    pwm_set_phase_correct(slice, 0);

    pwm_set_enabled(slice, 1);

}

void setPosition(int servoNum, double angle){

    //get slice and channel for servoPin
    uint slice=pwm_gpio_to_slice_num(servoPin[servoNum]);
    uint channel=pwm_gpio_to_channel(servoPin[servoNum]);

    //get level2angle conversion
    double level2angle;
    level2angle = (double)(servo_levelMax[servoNum] - servo_levelMin[servoNum])/(servo_angleMax[servoNum]-servo_angleMin[servoNum]);
    int level;
    level = (int)(level2angle * (angle - servo_angleMin[servoNum])) + servo_levelMin[servoNum];

    pwm_set_chan_level(slice, channel, level);

}

int main() {

    initServo(0);

    stdio_init_all();

    double angleRead = 0.0;

    while (1) {

        printf("Enter an number angle: \n");
        scanf("%f", &angleRead);

        setPosition(0, angleRead);
        sleep_ms(500);

     }

}