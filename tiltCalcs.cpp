#include "circle.hpp"
#include "helper.hpp"
#include "tiltCalcs.hpp"



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

