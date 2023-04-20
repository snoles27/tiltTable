#ifndef servos_H_   /* Include guard */
#define serovs_H_

void setPosition(int servoNum, double angle, bool manual);
void initServo(int servoNum);
void sweepServos(float initServo1, float initServo2, float finalServo1, float finalServo2, double seconds);

#endif