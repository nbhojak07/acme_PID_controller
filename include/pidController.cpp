#include "pidController.h"
#include <iostream>

void pidController::SetKp(double new_kp){
    std::cout << "Updated kp" << std::endl;
}

void pidController::SetKi(double new_ki){
    std::cout << "Updated ki" << std::endl;
}

void pidController::SetKd(double new_kd){
    std::cout << "Updated kd" << std::endl;
}

double pidController::GetKp(){
    std::cout << "Kp: " << kp << std::endl;
    return kp;
}

double pidController::GetKi(){
    std::cout << "Ki: " << ki << std::endl;
    return ki;
}

double pidController::GetKd(){
    std::cout << "Kd: " << kd << std::endl;
    return kd;
}

double Compute(double p_err,double d_err,double i_err){
    double gain = 0; 
    return gain;
}

