#include <stdio.h>
#include "hardware/pwm.h"
#include <cmath>
#include <chrono>

#include "servos.h"
#include "circle.hpp"
#include "helper.hpp"
#include "tiltCalcs.hpp"

void demoMove(){

    float amplitude = 0.8;
    float freq = 0.5;
    float timeStep = 0.02; //servo motor timestep (s)
    int numRotations = 5;
    int numStep = (int)((1/timeStep)/freq * numRotations);

    float pos1 = 0.0;
    float pos2 = 0.0;
    float initPos1 = 0.0;
    float initPos2 = 0.0;
    float time = 0.0;

    while (1) {

        // //moving in circle
        setPosition(0, amplitude, false);
        setPosition(1, 0., false);
        sleep_ms(6000);
        for(int i = 0; i < numStep; i++){
            pos1 = amplitude * std::cos(2. * M_PI * i * timeStep * freq);
            pos2 = amplitude * std::sin(2. * M_PI * i * timeStep * freq);
            setPosition(0, pos1, false);
            setPosition(1, pos2, false);
            sleep_ms((int)(timeStep * 1000));
        }
    }

}
void move(double* ElAz){
    //ElAz: elevation azimuth to move table normal

    double angleThresh = 1.3; //threshold for too much combied servo rotation for the ball joints. not a great metric

    double* servoAngles; 
    printf("Solving...");
    servoAngles = getServoAngles(ElAz); //solve for servo angles;
    printf("complete.\n");

    //check if angles are too large
    if(norm(servoAngles,2) > angleThresh){
        printf("Too Large!");
        display(norm(servoAngles,2));
        servoAngles[0] = 0.0;
        servoAngles[1] = 0.0;
    }

    display(norm(servoAngles));

    //set positions
    setPosition(0, servoAngles[0], false);
    setPosition(1, servoAngles[1], false);
}

int main() {

    //initialize servo/pwm
    initServo(0);
    initServo(1);
    
    stdio_init_all(); //start serial coms
    sleep_ms(3000); //pause before starting serial stuff
    printf("begin\n");

    
    //MAIN READ AND MOVE CODE
    while(1){
        double ElAzWant[2];
        double* servoAngles;
        printf("Enter Elevation (radians)\n");
        scanf("%lf", &ElAzWant[0]);
        printf("Enter Azimuth (radians)...\n");
        scanf("%lf", &ElAzWant[1]);
        move(ElAzWant);
        sleep_ms(100);
    }
    


    //dancing code 



    // //testing thetas2ElAz and timing it
    // double angles[2] {0.5, -0.5};
    // auto start = std::chrono::high_resolution_clock::now();
    // double* result;
    // for(int i = 0; i < 100; i++)
    // {
    //     result = thetas2ElAz(angles);
    // }
    // display(result,2);
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    // display((double)duration.count());


    // double* ptr;
    // double zeros[2] {0.0, 0.0};
    // double init[2] {RODLOC, 0.0};
    // double rodEndTest[2] {1.33, -1.88};

    // // ptr = nrstep(init, rodEndTest, zeros, 2.0, 1.0);


    // ptr = circleIntersect(rodEndTest, zeros, LROD, RODLOC, init);
    // display(ptr, 2);


    //testing solve
    // double A[4] {1., 2., 0.,1.};
    // double b[2] {3., 1.};
    // double* x = solve(A, b);
    // display(x[0]);
    // display(x[1]);
    
    //test nrstep 
    // double center1[2] {0.0, 0.0};
    // double r1 = 1.0;
    // double center2[2] {2.0,2.0};
    // double r2 = 2.0;
    // double point[2] {2.0, 0.0};

    // double* intersect;
    // auto start = std::chrono::high_resolution_clock::now();
    // for(int i = 0; i < 100; i++){
    //     intersect = circleIntersect(center1, center2, r1, r2, point, .00000001); //cicleIntersect() 400us compute time seems to be the testnig value
    // }
    // auto stop = std::chrono::high_resolution_clock::now();
    // display(intersect, 2);
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    // display((double)duration.count());

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
    //         pos1 = amplitude * std::cos(2. * M_PI * i * timeStep * freq);
    //         pos2 = amplitude * std::sin(2. * M_PI * i * timeStep * freq);
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