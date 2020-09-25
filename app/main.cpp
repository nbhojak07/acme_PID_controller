#include <iostream>
#include "pidController.h"

int main()
{
    pidController controller(1,1,1,1);
    controller.Compute(0);

    return 0;
}
