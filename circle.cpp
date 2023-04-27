#include <cmath>
#include <stdio.h>
#include "circle.hpp"
#include <algorithm>

//matrix: we have max 2x2 matrixes for this 2D problem stored as double[4] --> {a,b,c,d} --> [a b; c d]


double* circleIntersect(double *center1, double *center2, double r1, double r2, double *x0, double tol){

    int imax = 30;

    static double xn1[2]; //array that will be returned, also one that will be continually updated;
    double xn[2]; 
    double resid = 100.0;
    int i = 0;

    std::copy(x0, x0 + 2, xn);

    while(resid > tol){

        if(i > imax){
            printf("REACH MAX COUNT\n");
            return xn1;
        }

        double* tempPtr = nrstep(xn, center1, center2, r1, r2); //get updated value
        std::copy(tempPtr, tempPtr + 2, xn1);

        resid = normdiff(xn1, xn); 

        std::copy(xn1, xn1 + 2, xn);
        i++;
    }

    return xn1;

}

double* nrstep(double xn[], double center1[], double center2[], double r1, double r2){

    static double xn1[2]; //next value in the iteration
    double* fxn;
    double* dfxn;
    double* dx;
    fxn = twoCircle(xn, center1, center2, r1, r2);
    dfxn = twoCircleDot(xn, center1, center2);
    dx = solve(dfxn, fxn);
    xn1[0] = xn[0] - dx[0];
    xn1[1] = xn[1] - dx[1];

    return xn1;
}

double* solve(double A[], double b[]){
    //2D linear solve Ax = b where A is 2x2 and b is 2x1

    static double x[2];
    x[1] = b[1]/(A[3] - A[1] * A[2]/A[0]); 
    x[0] = (b[0] - A[1]*x[1])/A[0];

    return x;
}

double circle(double point[], double center[], double r){
    //returns 0 if point is on the circle described by center and r
    //point: test point 
    //cetner: center of circle
    //r: radius of circle

    return pow(point[0] - center[0],2) + pow(point[1] - center[1], 2) - pow(r, 2);
}

double* circleDot(double point[], double center[]){
    //derivative of circle function --> [df/dx df/dy]
    static double row[2];
    row[0] = 2 * (point[0] - center[0]);
    row[1] = 2 * (point[1] - center[1]);

    return row;
}

double* twoCircleDot(double point[], double center1[], double center2[]){
    //derivative of twoCircle function [df1/dx df2/dy df2/dx df2/dy]
    //point: test point
    //center1: center of first circle
    //center2: center of second circle

    static double matrix[4];
    double* tmpPtr;
    tmpPtr = circleDot(point, center1);
    matrix[0] = tmpPtr[0];
    matrix[1] = tmpPtr[1];
    tmpPtr = circleDot(point, center2);
    matrix[2] = tmpPtr[0];
    matrix[3] = tmpPtr[1];

    return matrix;
}

double* twoCircle(double point[], double center1[], double center2[], double r1, double r2){
    
    static double cost[2];

    cost[0] = circle(point, center1, r1);
    cost[1] = circle(point, center2, r2);

    return cost;
}

double norm(double v[]){
    return sqrt(pow(v[0],2) + pow(v[1],2));
}

double normdiff(double v1[], double v2[]){
    double diff[2];
    diff[0] = v1[0] - v2[0];
    diff[1] = v1[1] - v2[1];
    return norm(diff);
}