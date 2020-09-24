#include <iostream>
#include "pidController.h"
#include "TestPID.h"

int main()
{
    TestPID tester;
    tester.TestCompute();
    // pidController controller;
    // controller.SetKp(5);
    // controller.GetKp(); 
    return 0;
}
