/**
 * @copyright (c) 2020, Govind Ajith Kumar, Justin Albrecht
 *
 * @file pidController.cpp
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
#include "pidController.h"
#include <iostream>
/**
 * \@fn  pidController(double, double, double, double)
 * @brief User defined constructor with 4 double arguments
 *
 * @param KP the proportional constant
 * @param KI the integral constant
 * @param KD the derivative constant
 * @param target the target velocity
 */
pidController::pidController(double KP, double KI, double KD, double target) {
  kp = KP;
  ki = KI;
  kd = KD;
  target_velocity = target;
}

/**
 * @fn void SetKp(double)
 * @brief Function to set Kp to a desired value
 *
 * @param new_kp The updated proportional Constant
 */
void pidController::SetKp(double new_kp) {
}

/**
 * @fn void SetKi(double)
 * @brief Function to set Ki to a desired value
 *
 * @param new_ki The updated integral constant
 */

void pidController::SetKi(double new_ki) {
}

/**
 * @fn void SetKd(double)
 * @brief Function to set Kd to a desired value
 *
 * @param new_kd The updated derivative constant
 */
void pidController::SetKd(double new_kd) {
}

/**
 * @fn double GetKp()
 * @brief The function to get the Kp value
 *
 * @return Returns the Kp value, stored as a private variable
 */

double pidController::GetKp() {
  return 0;
}

/**
 * @fn double GetKi()
 * @brief The function to get the Ki value
 *
 * @return Returns the Ki value, stored as a private variable
 */
double pidController::GetKi() {
  return 0;
}

/**
 * @fn double GetKd()
 * @brief The function to get the Kd value
 *
 * @return Returns the Kd value, stored as a private variable
 */
double pidController::GetKd() {
  return 0;
}

/**
 * @fn double Compute(double)
 * @brief Computes the gain by the PID Controller
 *
 * @param velocity
 * @return The gain value
 */
double pidController::Compute(double velocity, std::vector<double> err_vec) {
  double gain = 0;
  // std::cout << "Running Compute()" << std::endl;
  return gain;
}

/**
 * @fn double CalculatePError()
 * @brief Calculates the Proportional Error
 *
 * @return The proportional error of the system
 */
double pidController::CalculatePError(double curr_err) {
  double p_error = 0;
  return p_error;
}

/**
 * @fn double CalculateIError()
 * @brief Calculates the Integral Error
 *
 * @return The integral error of the system
 */
double pidController::CalculateIError(std::vector<double> err_vec, double curr_err) {
  double i_error = 0;
  return i_error;
}

/**
 * @fn double CalculateDError()
 * @brief Calculates the Derivative Error
 *
 * @return The derivative error of the system
 */

double pidController::CalculateDError(double curr_err, std::vector<double> err_vec) {
  double d_error = 0;
  return d_error;
}

