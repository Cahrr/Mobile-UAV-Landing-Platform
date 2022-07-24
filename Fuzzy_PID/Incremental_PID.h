#include <iostream>

class Incremental_PID
{
private:
    double Kp;
    double Ki;
    double Kd;
    double set_point;
    double last_error = 0;
    double prev_error = 0;

public:
    void init(double sp, double kp, double ki, double kd);
    double Incremental_PID_output(double current_value);
};
