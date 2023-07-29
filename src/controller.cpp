#include "controller.h"
#include <iostream>
#include <chrono>



void controller::do_step(float desired_value,float present_value,float dt, float& Plog,float& Ilog,float& Dlog,float& dtLog,float& PIDOutput){
    //auto begin =std::chrono::steady_clock::now();

    float output=0.0;
    float D=0.0;
    float I=0.0;
    _dt=dt;
    float error=desired_value-present_value;
    float derivative=0.0;
    
    
    //****PROPORTIONAL-ANTEIL********
    float P=_KP*error;
    Plog=P;
    //std::cout<<"P-Anteil: "<<P<<std::endl;

    //*****INTEGRAL-ANTEIL*******
    _integral+=(error*_dt);
    I=_integral*_KI;
    Ilog=I;
    //std::cout<<"I-Anteil: "<<I<<std::endl;
    

    //****GRADIENTEN-ANTEIL
    derivative = (error - _pre_error) / _dt;
    D = _KD * derivative;
    Dlog=D;
    //std::cout<<"D-Anteil: "<<D<<std::endl;

    output=P+I+D;
    //***Stellgrößenbeschränkung***
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    _pre_error = error;
    PIDOutput=output;
}


