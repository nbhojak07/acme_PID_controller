#include <iostream>
#include <cassert>
#include "TestPID.h"

void TestPID::TestCompute(){
    std::cout << "Running unit test on Compute()" << std::endl;
    assert(controller.Compute(1) == 0);
}

// void TestPID::TestSetKp(){
//     // assert();
// }

// void TestPID::TestGetKp(){
//     // assert();
// }