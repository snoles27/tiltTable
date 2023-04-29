#include "pico/stdlib.h"
#include <string>


//methods for testing displaying things
void display(double num){
    std::string dis;
    dis = std::to_string(num);
    printf(dis.c_str());
    printf("\n");  
}

void display(int num){
    std::string dis;
    dis = std::to_string(num);
    printf(dis.c_str());
    printf("\n");  
}

void display(std::string dis){
    printf(dis.c_str());
    printf("\n");  
}

void display(double num[], int n){
    for(int i = 0; i < n; i++){
        display(num[i]);
    }
}