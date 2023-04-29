#ifndef circle_H_   /* Include guard */
#define circle_H_

#include <cmath>


double* circleIntersect(double *center1, double *center2, double r1, double r2, double *x0, double tol);
double* circleIntersect(double *center1, double *center2, double r1, double r2, double *x0);
double norm(double v[]);
double normdiff(double v1[], double v2[]);
double circle(double point[], double center[], double r);
double* circleDot(double point[], double center[]);
double* twoCircle(double point[], double center1[], double center2[], double r1, double r2);
double* twoCircleDot(double point[], double center1[], double center2[]);
double* nrstep(double xn[], double center1[], double center2[], double r1, double r2);
double* solve(double A[], double b[]);

#endif