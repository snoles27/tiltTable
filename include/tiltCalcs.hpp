#ifndef tiltCalcs_H_
#define tiltCalcs_H_

//CONSTANT DEFINITIONS
#define RSERV .97  //radius of servo arm (inches)
#define RODLOC 1.3 //location of rod attachment to plate from center (inches)
#define LROD  2.75 //length of actuation rod. Servo pivot --> ball center (inches)
#define XYSERV .815 //derived rodLoc - rServ/2
#define ZSERV  -2.7068939764977866 //derived -sqrt(lRod^2 - (rServ/2)^2)

double* rodEndLoc(double theta);
double* findPlanePoint(double theta);
double* getTableNormal(double theta0, double theta1);
double* getElAz(double* normal);
double* thetas2ElAz(double* angles);
double* thetas2DelElAz(double* angles, double* ElAz0);
double windingSegment_s0(double x0, double xf, double y, double* ElAz0);
double windingSegment_s1(double y0, double yf, double x, double* ElAz0);
double windingBox(double* box, double* ElAz0);
double rem2pi(double angle);
double* splitLong(double* box);
double* boxSize(double* box);
double boxArea(double* box);
double* twoDBisectionStep(double* box, double* ElAz0);
double* twoDBisection(double* initBox, double* ElAz0);
double* getServoAngles(double* ElAz);
double* getInitBox(double Az);

#endif