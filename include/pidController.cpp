#include "pidController.h"
#include <iostream>

void pidController::set_kp(double new_kp){
    kp = new_kp;
    std::cout << "Updated kp" << std::endl;
}

double pidController::get_kp(){
    std::cout << "Kp: " << kp << std::endl;
    return kp;
}


