# tiltTable
Code for my PHYS 4410 skill/project section of the course. 

Project is making a platform that can orient itself at any angle. Use cases include solar panels, directional antenaas etc. 

Julia code is for testing algorithm to go from disired table orinetation given in elevation and azimuth to the two servo angles. Implimented 2D bisection algorthim using winding numbers to solve. 

Final implimentation is in C++ for RP2040. 

    circle.cpp - Newton-Raphson iteration for finding insersection of two circles
    
    helper.cpp - print functions to help with dev and debugging
    
    servos.cpp - servo control functions and setting up PWM
    
    tiltCalcs.cpp - functions for implimneting 2D bisection
    
    main.cpp - main function with some helpful functions for bringing it all together
    
