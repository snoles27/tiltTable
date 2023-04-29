#ifndef helper_H_
#define helper_H_

#include <string>
#include "pico/stdlib.h"

//methods for testing displaying things
void display(double num);
void display(int num);
void display(std::string dis);
void display(double num[], int n);

#endif