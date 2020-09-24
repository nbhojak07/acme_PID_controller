#include <iostream>
#include "pidController.h"

int main()
{
    pidController controller;
    controller.set_kp(5);
    controller.get_kp(); 
    return 0;
}
