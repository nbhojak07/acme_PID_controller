/**
 * @copyright (c) 2020, Govind Ajith Kumar, Justin Albrecht
 *
 * @file pidControllerTest.cpp
 *
 * @authors
 * Govind Ajith Kumar (govindak-umd) \n
 * Justin Albrecht(jaybrecht) \n
 *
 * @version 1.0
 *
 * @section LICENSE
 *
 * BSD 3-Clause License
 *
 *
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This is a test function to test the methods of the PID controller implementation.
 */

#include <iostream>
#include <gtest/gtest.h>
#include "pidController.h"

/**
 * @brief This Test function aims to test the Compute method of the
 *        pidController class.
 * @param Computetest : - Test Name
 * @param errorCalculationTest : - Test to verify the return for zero input
 * @return none
 */

TEST(Computetest, zero_input) {
  double kp = 1;
  double ki = 1;
  double kd = 1;
  double target = 1;
  pidController controller(kp, ki, kd, target);
  double a = controller.Compute(0, {1, 2, 3, 4});
  double b = 9;
  EXPECT_EQ(b, a);
}

/**
 * @brief This Test function aims to test the kp getter of the
 *        pidController class.
 * @param GetterTest : - Test Name
 * @param Computetest : - Test to verify working of getter and setter
 * @return none
 */

TEST(GetterTest, TestForCorrectGet) {
  double kp = 1;
  double ki = 1;
  double kd = 1;
  double target = 1;
  pidController controller(kp, ki, kd, target);

  controller.SetKp(2.5);
  double a = controller.GetKp();
  double b = 2.5;
  EXPECT_EQ(b, a);
}
