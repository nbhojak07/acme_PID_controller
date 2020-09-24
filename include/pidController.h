#pragma once

class pidController{
public:
    void set_kp(double);
    void set_ki(double);
    void set_kd(double);
    double get_kp();
    double get_ki();
    double get_kd();
    double compute(double);


private:
    double kp;
    double ki;
    double kd;

};