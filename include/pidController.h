#pragma once
#include <vector>

class pidController{
public:
    pidController(double,double,double,double);
    void SetKp(double);
    void SetKi(double);
    void SetKd(double);
    double GetKp();
    double GetKi();
    double GetKd();
    double Compute(double);
    double CalculatePError();
    double CalculateIError();
    double CalculateDError();

private:
    double kp;
    double ki;
    double kd;
    std::vector<double> err_vec;
    double target_velocity;
    double dt = 1;

};