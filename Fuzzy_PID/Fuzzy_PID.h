#include <iostream>
#include <string>
#include <math.h>
#include <algorithm>
using std::string;
using namespace std;

#define NB 0
#define NM 1
#define NS 2
#define ZE 3
#define PS 4
#define PM 5
#define PB 6

class Fuzzy_PID
{
private:
    double Kp;
    double Ki;
    double Kd;
    double set_point;
    double scale_P;
    double scale_I;
    double scale_D;
    double scale_err;
    double scale_d_err;

public:
    int rules_P[7][7] = {{PB, PB, PM, PM, PS, ZE, ZE},
                         {PB, PB, PM, PS, PS, ZE, NS},
                         {PM, PM, PM, PS, ZE, NS, NS},
                         {PM, PM, PS, ZE, NS, NM, NM},
                         {PS, PS, ZE, NS, NS, NM, NM},
                         {PS, ZE, NS, NM, NM, NM, NB},
                         {ZE, ZE, NM, NM, NM, NB, NB}};
    
    int rules_I[7][7] = {{NB, NB, NM, NM, NS, ZE, ZE},
                         {NB, NB, NM, NS, NS, ZE, ZE},
                         {NB, NM, NS, NS, ZE, PS, PS},
                         {NM, NM, NS, ZE, PS, PM, PM},
                         {NM, NS, ZE, PS, PS, PM, PB},
                         {ZE, ZE, PS, PS, PM, PB, PB},
                         {ZE, ZE, PS, PM, PM, PB, PB}};

    int rules_D[7][7] = {{PS, NS, NB, NB, NB, NM, PS},
                         {PS, NS, NB, NM, NM, NS, ZE},
                         {ZE, NS, NM, NM, NS, NS, ZE},
                         {ZE, NS, NS, NS, NS, NS, ZE},
                         {ZE, ZE, ZE, ZE, ZE, ZE, ZE},
                         {PB, NS, PS, PS, PS, PS, PB},
                         {PB, PM, PM, PM, PS, PS, PB}};

    void init(double sp, double kp, double ki, double kd);
    double *Fuzzy_input_e(double x);
    double *Fuzzy_input_ec(double x);
    double defuzzification(const string &type, double *L);
    double Fuzzy_PID_output(double current_value, double e, double ec, double scale_P, double scale_I, double scale_D, double scale_err, double scale_d_err);
};
