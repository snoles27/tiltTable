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
    ElAz[1] = atan2(normal[1], normal[0]);

    return ElAz;
}

double* thetas2ElAz(double* angles){
    //angles: [s1, s2] where s1 and s2 are the servo angles
    //returns: El and Az of table corresponding to servo angles

    static double ElAz[2];
    double* nhat = getTableNormal(angles[0], angles[1]);
    double* hold;
    hold = getElAz(nhat);
    std::copy(hold, hold + 2, ElAz);
    return ElAz; 

}