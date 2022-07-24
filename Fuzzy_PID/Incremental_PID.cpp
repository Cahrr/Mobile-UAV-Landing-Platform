#include "Incremental_PID.h"

void Incremental_PID::init(double sp, double kp, double ki, double kd)
{
    Incremental_PID::set_point = sp;
    Incremental_PID::Kp = kp;
    Incremental_PID::Ki = ki;
    Incremental_PID::Kd = kd;
}

double Incremental_PID::Incremental_PID_output(double current_value)
{
    double error = Incremental_PID::set_point - current_value;
    double K_error = error - Incremental_PID::last_error;
    double I_error = error;
    double D_error = error - 2 * Incremental_PID::last_error + Incremental_PID::prev_error;
    Incremental_PID::prev_error = Incremental_PID::last_error;
    Incremental_PID::last_error = error;

    return Incremental_PID::Kp * K_error + Incremental_PID::Ki * I_error + Incremental_PID::Kd * D_error;
}
