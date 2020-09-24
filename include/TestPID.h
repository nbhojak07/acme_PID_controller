#pragma once
#include "pidController.h"

class TestPID{
public:
    pidController controller{1,1,1,1};
    void TestCompute();
    void TestSetKp();
    void TestGetKp();

};