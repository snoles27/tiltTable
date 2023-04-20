#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "servos.h"
#include <string>


int main() {
 
    initServo(0);
    initServo(1);

    stdio_init_all();

    float angleRead = 0.0;
    int servoNumber = 0;


    float pos1 = 0.0;
    float pos2 = 0.0;
    float initPos1 = 0.0;
    float initPos2 = 0.0;
    float time = 0.0;

    while (1) {

        scanf("%f", &pos1);
        scanf("%f", &pos2);
        scanf("%f", &time);

        sweepServos(initPos1, initPos2, pos1, pos2, time);
        initPos1 = pos1;
        initPos2 = pos2;
        printf("Complete! \n");

        // printf("Enter a servo number: \n");
        // scanf("%d", &servoNumber);
        // printf("Enter an number angle: \n");
        // scanf("%f", &angleRead);

        // setPosition(servoNumber, angleRead, true);
        // sleep_ms(500);

     }

}