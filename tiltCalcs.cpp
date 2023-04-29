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


