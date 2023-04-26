#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "servos.h"
#include <string>
#include <cmath>

#define PI 3.1415926

int main() {
    
    // testing solvers



    // testing with stand

    // initServo(0);
    // initServo(1);

    // stdio_init_all();

    // float angleRead = 0.0;
    // int servoNumber = 0;

    // float amplitude = 0.8;
    // float freq = 0.5;
    // float timeStep = 0.02; //servo motor timestep (s)
    // int numRotations = 5;
    // int numStep = (int)((1/timeStep)/freq * numRotations);

    // float pos1 = 0.0;
    // float pos2 = 0.0;
    // float initPos1 = 0.0;
    // float initPos2 = 0.0;
    // float time = 0.0;

    // while (1) {

    //     // //moving in circle
    //     setPosition(0, amplitude, false);
    //     setPosition(1, 0., false);
    //     sleep_ms(6000);
    //     std::string dis = "hold";
    //     for(int i = 0; i < numStep; i++){
    //         pos1 = amplitude * std::cos(2. * PI * i * timeStep * freq);
    //         pos2 = amplitude * std::sin(2. * PI * i * timeStep * freq);
    //         setPosition(0, pos1, false);
    //         setPosition(1, pos2, false);
    //         // dis = std::to_string(pos1);
    //         // printf(dis.c_str());
    //         // printf("\n");
    //         sleep_ms((int)(timeStep * 1000));
    //     }

        // scanf("%f", &pos1);
        // scanf("%f", &pos2);
        // scanf("%f", &time);

        // sweepServos(initPos1, initPos2, pos1, pos2, time);
        // initPos1 = pos1;
        // initPos2 = pos2;
        // printf("Complete! \n");

        // printf("Enter a servo number: \n");
        // scanf("%d", &servoNumber);
        // printf("Enter an number angle: \n");
        // scanf("%f", &angleRead);

        // setPosition(servoNumber, angleRead, true);
        // sleep_ms(500);

    // }

}