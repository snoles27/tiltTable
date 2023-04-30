#include "circle.hpp"
#include "helper.hpp"
#include "tiltCalcs.hpp"

#define SERVOMAX M_PI_2
#define BOXZERO 0.0

double* rodEndLoc(double theta){
    //theta:: angle over servo arm. 0 corresponds to horozontal
    //returns:: array pointer, (x (or y), z) position of servo end of rod

    static double loc[2];
    loc[0] = XYSERV + RSERV * cos(theta);
    loc[1] = ZSERV + RSERV * sin(theta);
    return loc;

}

double* findPlanePoint(double theta){
    //theta: servo angles
    //returns: point of rod end in plane of servo arm rotation (xz or yz plane for servo1 and servo2 respectivly)

    static double planePoint[2];
    double zeros[2] {0.0, 0.0};
    double init[2] {RODLOC, 0.0};

    double* rodEnd;
    rodEnd = rodEndLoc(theta);

    double* hold;
    hold = circleIntersect(rodEnd, zeros, LROD, RODLOC,  init); 

    std::copy(hold, hold + 2, planePoint);

    return planePoint;
}

double* getTableNormal(double theta0, double theta1){
    //theta0: servo angle 0
    //theta1: servo angle 1
    //returns: table normal correesponding to given servo anlges array length 3

    static double nhat[3];
    double* tempPtr;
    double xz[2];
    double yz[2];
    double n[3];
    double nnorm;

    tempPtr = findPlanePoint(theta0);
    std::copy(tempPtr, tempPtr + 2, xz);
    tempPtr = findPlanePoint(theta1);
    std::copy(tempPtr, tempPtr + 2, yz);

    n[0] = -1 * xz[1] * yz[0];
    n[1] = -1 * xz[0] * yz[1];
    n[2] = xz[0] * yz[0];

    nnorm = norm(n, 3);
    for(int i = 0; i < 3; i++){
        nhat[i] = n[i]/nnorm;
    }
    return nhat;

}

double* getElAz(double* normal){
    //normal: normal vector 
    //returns: El and Az associateed with the normal vector

    static double ElAz[2];
    ElAz[0] = asin(normal[2]);

    if(normal[1] == 0.0 && normal[0] == 0.0){
        ElAz[1] = 0.0;
    }else{
        ElAz[1] = atan2(normal[1], normal[0]);
    }
    return ElAz;
}

double* thetas2ElAz(double* angles){
    //angles: [s1, s2] where s1 and s2 are the servo angles
    //returns: El and Az of table corresponding to servo angles

    static double ElAz[2];
    double* nhat = getTableNormal(angles[0], angles[1]);
    // printf("nhat:\n");
    // display(nhat,3);
    double* hold;
    hold = getElAz(nhat);
    // printf("elAz:\n");
    // display(hold,2);
    std::copy(hold, hold + 2, ElAz);
    return ElAz; 

}

double rem2pi(double angle){
    return angle - 2.0 * M_PI * round(angle/(2.0 * M_PI));
}

double* thetas2DelElAz(double* angles, double* ElAz0){
    //angles: [s0, s1]
    //ElAz0: desired Elevation and Azimuth [El0, Az0]
    //returns: [delEl, delAz] where delAz is RoundNearest

    static double del[2];
    double* ElAz;
    ElAz = thetas2ElAz(angles);
    // printf("ElAz: ");
    // display(ElAz,2);
    del[0] = ElAz[0] - ElAz0[0];
    del[1] = rem2pi(ElAz[1] - ElAz0[1]);
    return del;
}

double windingSegment_s0(double x0, double xf, double y, double* ElAz0){
    //x0: initial servo 0 angle
    //xf: final servo 0 anlge
    //y: constant servo 1 angle
    //returns winding number along segment where only s0 changes

    //parameters
    int numStep = 25;

    double* outhold;
    double angles[2];
    double an;
    double an1;
    double step;
    step = (xf - x0)/numStep;

    angles[0] = x0;
    angles[1] = y;
    outhold = thetas2DelElAz(angles, ElAz0); //get ElAz (output space) of inital position 
    an = atan2(outhold[1], outhold[0]);
    // printf("a0 ");
    // display(an);

    double winding = 0;
    
    for(int i = 1; i <= 25; i++){
        angles[0] = x0 + step * i; //update angle
        // printf("angle: ");
        // display(angles[0]);
        outhold = thetas2DelElAz(angles, ElAz0); //update output point
        an1 = atan2(outhold[1], outhold[0]);
        // printf("an1: ");
        // display(an1);
        winding += rem2pi(an1 - an);
        // printf("winding: ");
        // display(winding);
        an = an1;
    }

    return winding;
}

double windingSegment_s1(double y0, double yf, double x, double* ElAz0){
    //x0: initial servo 0 angle
    //xf: final servo 0 anlge
    //y: constant servo 1 angle
    //returns winding number along segment where only s0 changes

    //parameters
    int numStep = 25;

    double* outhold;
    double angles[2];
    double an;
    double an1;
    double step;
    step = (yf - y0)/numStep;

    angles[0] = x;
    angles[1] = y0;
    outhold = thetas2DelElAz(angles, ElAz0); //get ElAz (output space) of inital position 
    an = atan2(outhold[1], outhold[0]);
    // printf("a0 ");
    // display(an);

    double winding = 0;
    
    for(int i = 1; i <= 25; i++){
        angles[1] = y0 + step * i; //update angle
        // printf("angle: ");
        // display(angles[0]);
        outhold = thetas2DelElAz(angles, ElAz0); //update output point
        an1 = atan2(outhold[1], outhold[0]);
        // printf("an1: ");
        // display(an1);
        winding += rem2pi(an1 - an);
        // printf("winding: ");
        // display(winding);
        an = an1;
    }

    return winding;

 }

double windingBox(double* box, double* ElAz0){
    //box: (x0, x1, y0, y1) array storing location of box sides

    double winding = 0;
    winding += windingSegment_s0(box[0], box[1], box[2], ElAz0);
    winding += windingSegment_s1(box[2], box[3], box[1], ElAz0);
    winding += windingSegment_s0(box[1], box[0], box[3], ElAz0);
    winding += windingSegment_s1(box[3], box[2], box[0], ElAz0);

    return winding;

}

double* twoDBisectionStep(double* box, double* ElAz0){
    //box: 4 element array representing box
    //returns box of half the area with the zero still in it

    static double returnbox[4];
    double thresh = 0.000001;
    double* twoBox;
    twoBox = splitLong(box);

    double winding1;
    double winding2;
    winding1 = windingBox(twoBox, ElAz0);
    winding2 = windingBox(&twoBox[4], ElAz0);

    if(rem2pi(winding1)<thresh && abs(winding1) > thresh){
        std::copy(twoBox, twoBox + 4, returnbox);
    }else if (rem2pi(winding2)<thresh && abs(winding2) > thresh){
        std::copy(&twoBox[4], &twoBox[4] + 4, returnbox);
    }else{
        printf("Fail at twoDBisectionstep\n");
    }
    return returnbox;
}

double* twoDBisection(double* initBox, double* ElAz0){
    //initBox: box array to start the iteration --> solution must be in this box
    //ElAz0: desired Elevation and azimuth

    double convergedArea = 0.0001; //area to compare box area too. once less it will return center of box

    static double ans[2]; //ans to return
    double box[4]; //holds box
    double* hold; //pointer holder to copy to box

    std::copy(initBox, initBox + 4, box); //copy initial box to the box
    // printf("Init Box: \n");
    // display(box,4);
    while(boxArea(box) > convergedArea){
        hold = twoDBisectionStep(box, ElAz0);
        std::copy(hold, hold + 4, box); //copy result over to box
        // printf("Box: \n");
        // display(box,4);
    }

    //get center of box
    ans[0] = (box[0] + box[1])/2;
    ans[1] = (box[2] + box[3])/2;

    return ans;

}

double boxArea(double* box){
    //box: box like all the rest
    //returns: area of box. relies on size returning positive values
    double* size;
    size = boxSize(box);
    double area;
    area =  size[0] * size[1];
    printf("Area: ");
    display(area);
    return area;
}

double* boxSize(double* box){
    //box: array of length 4 encoding box
    //returns: array containing side length of box
    //assumes box[1] > box[0] and box[3] > box[2]

    static double size[2];
    size[0] = box[1] - box[0];
    size[1] = box[3] - box[2];
    return size;
}

double* splitLong(double* box){
    //box: array of length 4 encoding box
    //returns newBoxes: array of length 8 containing 2 boxes back to back 
    static double newBoxes[8];
    double* size = boxSize(box);
    
    if(size[0] > size[1]){ //split horozontally. //left box is first
        double newX = (box[0] + box[1])/2; //middle of two xpositions
        newBoxes[0] = box[0];
        newBoxes[1] = newX;
        newBoxes[2] = box[2];
        newBoxes[3] = box[3];
        newBoxes[4] = newX;
        newBoxes[5] = box[1];
        newBoxes[6] = box[2];
        newBoxes[7] = box[3];
    }
    else{
        double newY = (box[2] + box[3])/2;
        newBoxes[0] = box[0];
        newBoxes[1] = box[1];
        newBoxes[2] = box[2];
        newBoxes[3] = newY;
        newBoxes[4] = box[0];
        newBoxes[5] = box[1];
        newBoxes[6] = newY;
        newBoxes[7] = box[3];
    }

    return newBoxes;
}

double* getInitBox(double Az){
    //returns inital box based on the desired azimuth
    static double box[4];
    if (Az < M_PI_4 && Az >= -1 * M_PI_4){
        box[0] = -1 * SERVOMAX;
        box[1] = BOXZERO;
        box[2] = -1 * SERVOMAX;
        box[3] = SERVOMAX;
    }else if(Az < 3 * M_PI_4 && Az >= M_PI_4){
        box[0] = -1 * SERVOMAX;
        box[1] = SERVOMAX;
        box[2] = -1 * SERVOMAX;
        box[3] = BOXZERO;
    }else if(Az < -1 * M_PI_4 && Az >= -3 * M_PI_4){
        box[0] = -1 * SERVOMAX;
        box[1] = SERVOMAX;
        box[2] = BOXZERO;
        box[3] = SERVOMAX;
    }else{
        box[0] = BOXZERO;
        box[1] = SERVOMAX;
        box[2] = -1 * SERVOMAX;
        box[3] = SERVOMAX;
    }

    return box;

}

double* getServoAngles(double* ElAz){

    //ElAz: array of two elements that has desired elevation and Azimuth
    //returns: servo angles to s0, s1 to give desired elevation/azimuth

    static double result[2];
    double* initBox;
    initBox = getInitBox(ElAz[1]);

    double* servoAngles;
    servoAngles = twoDBisection(initBox, ElAz);
    std::copy(servoAngles, servoAngles + 2, result);

    return result;
}

