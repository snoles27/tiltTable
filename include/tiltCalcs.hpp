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

#endif