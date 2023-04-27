#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <string>
#include <cmath>

#include "servos.h"
#include "circle.hpp"

#define PI 3.1415926

//methods for testing displaying things
void display(double num){
    std::string dis;
    dis = std::to_string(num);
    printf(dis.c_str());
    printf("\n");  
}

void display(int num){
    std::string dis;
    dis = std::to_string(num);
    printf(dis.c_str());
    printf("\n");  
}

void display(std::string dis){
    printf(dis.c_str());
    printf("\n");  
}

void display(double num[], int n){
    for(int i = 0; i < n; i++){
        display(num[i]);
    }
}

int main() {
    
    stdio_init_all();
    sleep_ms(3000);
    printf("begin\n");

    
    //testing solve
    // double A[4] {1., 2., 0.,1.};
    // double b[2] {3., 1.};
    // double* x = solve(A, b);
    // display(x[0]);
    // display(x[1]);
    
    //test nrstep 
    double center1[2] {0.0, 0.0};
    double r1 = 1.0;
    double center2[2] {2.0,2.0};
    double r2 = 2.0;
    double point[2] {2.0, 0.0};

    double* intersect = circleIntersect(center1, center2, r1, r2, point, .00000001);
    display(intersect, 2);

    // double* checkans = nrstep(point, center1, center2, r1, r2);
    // display(checkans, 2);


    //test circle center with IO
    // while(1){

    //     double center1[2] {0.0, 0.0};
    //     double r1 = 1.0;
    //     double center2[2] {2.0,2.0};
    //     double r2 = 1.0;
    //     double point[2] {1.0, 0.0};

    //     printf("Type now\n");
    //     scanf("%lf", &center1[0]);
    //     scanf("%lf", &center1[1]);
    //     scanf("%lf", &center2[0]);
    //     scanf("%lf", &center2[1]);
    //     scanf("%lf", &r1);
    //     scanf("%lf", &r2);

    //     double* circleDot1 = circleDot(point, center1);
    //     display(circleDot1[0]);
    //     display(circleDot1[1]);
    //     double* circleDot2 = circleDot(point, center2);
    //     display(circleDot2[0]);
    //     display(circleDot2[1]);
    //     printf("\n");
    //     double* twoCircleDotPtr = twoCircleDot(point, center1, center2);
    //     display(twoCircleDotPtr[0]);
    //     display(twoCircleDotPtr[1]);
    //     display(twoCircleDotPtr[2]);
    //     display(twoCircleDotPtr[3]);



        // double* costPntr; 
        // costPntr = twoCircle(point, center1, center2, r1, r2);
        // display(costPntr[0]);
        // display(costPntr[1]);

    // }
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