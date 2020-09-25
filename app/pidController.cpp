#include "pidController.h"
#include <iostream>

pidController::pidController(double KP,double KI,double KD, double target){
    kp = KP;
    ki = KI;
    kd = KD;
    target_velocity = target;
}

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

double pidController::Compute(double velocty){
    double gain = 0; 
    // std::cout << "Running Compute()" << std::endl;
    return gain;
}

double pidController::CalculatePError(){
    double p_error = 0;
    return p_error;
}

double pidController::CalculateIError(){
    double i_error = 0;
    return i_error;
}

double pidController::CalculateDError(){
    double d_error = 0;
    return d_error;
}

