#include <iostream>

using namespace std;

class Range
{
public:
	double x;
	double y;
	double z;
	double Limit(double d, double r, double l);
};

double Range::Limit(double d, double r, double l)
{
	if (d > r)
	{
		d = l;
	}
	if (d < (-r))
	{
		d = (-l);
	}
	return d;
}

class Body
{
public:
	double x;
	double y;
	double z;
};

class Err
{
public:
	double x;
	double y;
	double z;
};

class Err_sum
{
public:
	double x;
	double y;
	double z;
};

class dErr
{
public:
	double x;
	double y;
	double z;
};

class PI
{
public:
	Range range;
	Body body;
	Err err;
	Err_sum err_sum;
	dErr derr;
};

class PID
{
public:
	Range range;
	Body body;
	Err err;
	Err_sum err_sum;
	dErr derr;
};

class Icm_PID
{
public:
	Body body;
	Err err;
	Err_sum err_sum;
	dErr derr;
};
