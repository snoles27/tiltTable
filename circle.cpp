#include <cmath>
#include <stdio.h>
#include "circle.hpp"
#include <algorithm>
#include "helper.hpp"
#include <Eigen/Dense>

//matrix: we have max 2x2 matrixes for this 2D problem stored as double[4] --> {a,b,c,d} --> [a b; c d]


double* circleIntersect(double *center1, double *center2, double r1, double r2, double *x0,  double tol){
    //finds intersection of two circles 

    int imax = 30;

    static double xn1[2]; //array that will be returned, also one that will be continually updated;
    double xn[2]; 
    double resid = 100.0;
    int i = 0;

    std::copy(x0, x0 + 2, xn); //copy x0 into xn 

    while(resid > tol){

        //check if we have iterated too many times. if so return what is there and print error. 
        if(i > imax){ 
            printf("REACH MAX COUNT\n");
            return xn1;
        }

        // printf("xn\n");
        // display(xn, 2);

        double* tempPtr = nrstep(xn, center1, center2, r1, r2); //get updated value
        std::copy(tempPtr, tempPtr + 2, xn1); //copy updated value to xn1

        resid = normdiff(xn1, xn);  //calculate change in value

        std::copy(xn1, xn1 + 2, xn); //copy xn1 to xn
        i++; 
    }

    return xn1;

}

double* circleIntersect(double *center1, double *center2, double r1, double r2, double *x0){
    double tol = 0.00001;
    return circleIntersect(center1, center2, r1, r2, x0, tol);
}

double* nrstep(double xn[], double center1[], double center2[], double r1, double r2){
    //netwon raphson step of two circle intersection problem. 

    static double xn1[2]; //next value in the iteration
    double* fxn;
    double* dfxn;
    double* dx;
    fxn = twoCircle(xn, center1, center2, r1, r2);
    // printf("twoCircle:\n");
    // display(fxn, 2);
    dfxn = twoCircleDot(xn, center1, center2);
    // printf("twoCircleDot:\n");
    // display(dfxn,4);
    dx = solve(dfxn, fxn);
    xn1[0] = xn[0] - dx[0];
    xn1[1] = xn[1] - dx[1];

    return xn1;
}

double* solve(double A[], double b[]){
    //2D linear solve Ax = b where A is 2x2 and b is 2x1

    static double x[2];

    // printf("A: \n");
    // display(A, 4);
    // printf("b: \n");
    // display(b, 2);

    // if(A[0] != 0.0){
    //     printf("main A\n");
    //     x[1] = b[1]/(A[3] - A[1] * A[2]/A[0]); 
    //     x[0] = (b[0] - A[1]*x[1])/A[0];
    // } else if (A[1] != 0.0){
    //     printf("Main B\n");
    //     x[0] = b[1]/(A[2] - A[3] * A[0]/A[1]);
    //     x[1] = (b[0] - x[0] * A[0])/A[1];
    // }
    // else{
    //     printf("You gotta add more idiot\n");
    //     x[0] = 272727.0;
    //     x[1] = 272727.0;
    // }

    //using eigen to do solve cause I am dummy. Doubles run time compared to above code that only works sometimes
    Eigen::Matrix2d Amat;
    Amat(0,0) = A[0];
    Amat(0,1) = A[1];
    Amat(1,0) = A[2];
    Amat(1,1) = A[3];
    Eigen::Vector2d bvec;
    bvec(0) = b[0];
    bvec(1) = b[1];

    Eigen::Vector2d xvec = Amat.colPivHouseholderQr().solve(bvec);
    x[0] = xvec(0);
    x[1] = xvec(1);
 
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

double norm(double v[], int n){
    double sum = 0;
    for(int i = 0; i < n; i++){
        sum += pow(v[i],2);
    }
    return sqrt(sum);
}

double normdiff(double v1[], double v2[]){
    //finds distance between two points in R2 defined by v1 and v2
    double diff[2];
    diff[0] = v1[0] - v2[0];
    diff[1] = v1[1] - v2[1];
    return norm(diff);
}