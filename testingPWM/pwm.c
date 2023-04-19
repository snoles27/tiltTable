#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"




const uint prim_pin = 15;
const uint16_t wrap = 18750;
const uint16_t initLevel = 6250;
const uint8_t clkdiv_int = 20;
const uint8_t clkdiv_frac = 0;

const int angleMin = 0;
const int levelMin = 3125;
const int angleMax = 270;
const int levelMax = 15625;



/*
   DS SERVO 20KG stats
   1500us neutral
   333Hz
   500us-2500us 

   1.25e8 Hz base
   20 clkdivide
   Ts = 1.6e-7
   levelMin = 3125
   levelMax = 15625
   levelMix = 9375
*/


int main() {

    gpio_set_function(prim_pin, GPIO_FUNC_PWM);
    uint slice=pwm_gpio_to_slice_num(prim_pin);
    uint channel=pwm_gpio_to_channel(prim_pin);

    //pwm_set_enabled(slice, 1);

    pwm_set_clkdiv_int_frac(slice, clkdiv_int, clkdiv_frac);
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, channel, initLevel);
    pwm_set_phase_correct(slice, 0);

    pwm_set_enabled(slice, 1);

    stdio_init_all();

    int level = initLevel;
    int angleRead = 0;

    double level2angle;
    level2angle = (double)(levelMax - levelMin)/(angleMax-angleMin);


     while (1) {

        printf("Enter an integer angle between 0 and 90\n");
        scanf("%d", &angleRead);

        level = (int)(level2angle * (angleRead - angleMin)) + levelMin;
        pwm_set_chan_level(slice, channel, level);
        sleep_ms(500);

     }
    

}