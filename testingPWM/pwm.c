#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"


const uint prim_pin = 15;
const uint16_t wrap = 12500;
const uint16_t level = 6250;
const uint8_t clkdiv_int = 40;
const uint8_t clkdiv_frac = 0;


int main() {

    gpio_set_function(prim_pin, GPIO_FUNC_PWM);
    uint slice=pwm_gpio_to_slice_num(prim_pin);
    uint channel=pwm_gpio_to_channel(prim_pin);

    //pwm_set_enabled(slice, 1);

    pwm_set_clkdiv_int_frac(slice, clkdiv_int, clkdiv_frac);
    pwm_set_wrap(slice, wrap);
    pwm_set_chan_level(slice, channel, level);
    pwm_set_phase_correct(slice, 0);

    pwm_set_enabled(slice, 1);


     while (1)
        tight_loop_contents();
    

}