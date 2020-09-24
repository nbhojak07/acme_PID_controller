#pragma once

class pidController{
public:
    void SetKp(double);
    void SetKi(double);
    void SetKd(double);
    double GetKp();
    double GetKi();
    double GetKd();
    double Compute(double,double,double);


private:
    double kp;
    double ki;
    double kd;

};