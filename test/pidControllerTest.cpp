// #include <iostream>
#include <gtest/gtest.h>
#include "pidController.h"

TEST(computetest, zero_input) {
    pidController controller(1,1,1,1);
    double a = controller.Compute(0);
    double b = 0;
    EXPECT_EQ(b, a);
}

TEST(computetest, input_5) {
    pidController controller(1,1,1,1);
    double a = controller.Compute(5);
    double b = 0;
    EXPECT_EQ(b, a);
}