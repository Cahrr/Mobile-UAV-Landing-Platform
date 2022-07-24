#include "Fuzzy_PID.h"
#include "Incremental_PID.h"

void Fuzzy_PID::init(double sp, double kp, double ki, double kd)
{
    Fuzzy_PID::set_point = sp;
    Fuzzy_PID::Kp = kp;
    Fuzzy_PID::Ki = ki;
    Fuzzy_PID::Kd = kd;
}

//三角形隶属度函数
double *Fuzzy_PID::Fuzzy_input_e(double x)
{
    double mf[7] = {0, 0, 0, 0, 0, 0, 0};
    // NB
    mf[0] = max(-2.0 - x, 0.0);
    // NM
    mf[1] = max(min(x + 3.0, -1.0 - x), 0.0);
    // NS
    mf[2] = max(min(x + 2.0, -x), 0.0);
    // ZE
    mf[3] = max(min(x + 1.0, 1.0 - x), 0.0);
    // PS
    mf[4] = max(min(x, 2.0 - x), 0.0);
    // PM
    mf[5] = max(min(x - 1.0, 3.0 - x), 0.0);
    // PB
    mf[6] = max(x - 2.0, 0.0);

    return mf;
}

//三角形隶属度函数
double *Fuzzy_PID::Fuzzy_input_ec(double x)
{
    double mf[7] = {0, 0, 0, 0, 0, 0, 0};
    // NB
    mf[0] = max(-2.0 - x, 0.0);
    // NM
    mf[1] = max(min(x + 3.0, -1.0 - x), 0.0);
    // NS
    mf[2] = max(min(x + 2.0, -x), 0.0);
    // ZE
    mf[3] = max(min(x + 1.0, 1.0 - x), 0.0);
    // PS
    mf[4] = max(min(x, 2.0 - x), 0.0);
    // PM
    mf[5] = max(min(x - 1.0, 3.0 - x), 0.0);
    // PB
    mf[6] = max(x - 2.0, 0.0);

    return mf;
}

double Fuzzy_PID::defuzzification(const string &type, double *L)
{
    double u = 0;
    double num = 0;
    double den = 0;
    if (type == "centroid")
    {
        for (int i = 0; i < (sizeof(L) / sizeof(L[0])); i++)
        {
            num += (i - 3) * L[i];
            den += L[i];
        }
        u = num / den;
    }
    return u;
}

double Fuzzy_PID::Fuzzy_PID_output(double current_value, double e, double ec, double scale_P, double scale_I, double scale_D, double scale_err, double scale_d_err)
{
    double input1[7];
    double input2[7];

    e = e * scale_err;
    ec = ec * scale_d_err;

    *input1 = *Fuzzy_PID::Fuzzy_input_e(e);
    *input2 = *Fuzzy_PID::Fuzzy_input_ec(ec);

    double fuzzy_kp[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double fuzzy_ki[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double fuzzy_kd[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            fuzzy_kp[Fuzzy_PID::rules_P[i][j]] = max(min(input1[i], input2[j]), fuzzy_kp[Fuzzy_PID::rules_P[i][j]]);
            fuzzy_ki[Fuzzy_PID::rules_I[i][j]] = max(min(input1[i], input2[j]), fuzzy_ki[Fuzzy_PID::rules_I[i][j]]);
            fuzzy_kd[Fuzzy_PID::rules_D[i][j]] = max(min(input1[i], input2[j]), fuzzy_kd[Fuzzy_PID::rules_D[i][j]]);
        }
    }

    double delta_P = 0.0;
    double delta_I = 0.0;
    double delta_D = 0.0;

    delta_P = Fuzzy_PID::defuzzification("centroid", fuzzy_kp);
    delta_I = Fuzzy_PID::defuzzification("centroid", fuzzy_ki);
    delta_D = Fuzzy_PID::defuzzification("centroid", fuzzy_kd);

    delta_P = delta_P * scale_P;
    delta_I = delta_I * scale_I;
    delta_D = delta_D * scale_D;

    Fuzzy_PID::Kp += delta_P;
    Fuzzy_PID::Ki += delta_I;
    Fuzzy_PID::Kd += delta_D;

    //设置Kp、Ki、Kd的取值范围
    if (Fuzzy_PID::Kp > 1)
    {
        Fuzzy_PID::Kp = 1;
    }

    if (Fuzzy_PID::Kp < 0)
    {
        Fuzzy_PID::Kp = 0;
    }

    if (Fuzzy_PID::Ki > 0.5)
    {
        Fuzzy_PID::Ki = 0.5;
    }

    if (Fuzzy_PID::Ki < 0)
    {
        Fuzzy_PID::Ki = 0;
    }

    if (Fuzzy_PID::Kd > 0.5)
    {
        Fuzzy_PID::Kd = 0.5;
    }

    if (Fuzzy_PID::Kd < 0)
    {
        Fuzzy_PID::Kd = 0;
    }
    Incremental_PID pid;
    pid.init(Fuzzy_PID::set_point, Fuzzy_PID::Kp, Fuzzy_PID::Ki, Fuzzy_PID::Kd);

    double output;
    output = pid.Incremental_PID_output(current_value);

    return output;
}