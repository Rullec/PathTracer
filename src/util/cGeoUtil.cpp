#include <util/cGeoUtil.hpp>
#include <iostream>

tVector cGeoUtil::CalcPlaneEquation(const tVector & p1, const tVector & p2, const tVector & p3)
{
	assert(std::abs(p1[3] - 1) < 1e-10);
	assert(std::abs(p2[3] - 1) < 1e-10);
	assert(std::abs(p3[3] - 1) < 1e-10);

	tVector normal = (p2 - p1).cross3(p3 - p2);
	normal[3] = 0;
	if (normal.norm() < 1e-10)
	{
		std::cout << "[error] cGeoUtil::CalculatePlaneEquation can not construct a plane" << std::endl;
	}

	normal[3] = -(normal.dot(p1));
	// normal = [a, b, c, d] -> ax + by + cz + d = 0
	return normal;
}

tVector cGeoUtil::SamplePointFromPlane(const tVector &equation)
{
	assert(equation.segment(0, 3).norm() > 1e-10);
	double x = std::rand(), y = std::rand(), z = (equation[3] + equation[0] * x + equation[1] * y) / (-equation[2]);
	return tVector(x, y, z, 1);
}

inline double Det(double a, double b, double c, double d)
{
	return a * d - b * c;
}

//2D Line-line intersection using determinants
//by Tim Sheerman-Chase, 2016
//Released under CC0
bool cGeoUtil::CalcLineLineIntersection(double x1, double y1, //Line 1 start
	double x2, double y2, //Line 1 end
	double x3, double y3, //Line 2 start
	double x4, double y4, //Line 2 end
	double &ixOut, double &iyOut) //Output 
{
	//http://mathworld.wolfram.com/Line-LineIntersection.html
	const int scale = 10000;
	x1 *= scale, y1 *= scale;
	x2 *= scale, y2 *= scale;
	x3 *= scale, y3 *= scale;
	x4 *= scale, y4 *= scale;

	double detL1 = Det(x1, y1, x2, y2);
	double detL2 = Det(x3, y3, x4, y4);
	double x1mx2 = x1 - x2;
	double x3mx4 = x3 - x4;
	double y1my2 = y1 - y2;
	double y3my4 = y3 - y4;

	double xnom = Det(detL1, x1mx2, detL2, x3mx4);
	double ynom = Det(detL1, y1my2, detL2, y3my4);
	double denom = Det(x1mx2, y1my2, x3mx4, y3my4);
	if (denom == 0.0)//Lines don't seem to cross
	{
		ixOut = NAN;
		iyOut = NAN;
		return false;
	}

	ixOut = xnom / denom;
	iyOut = ynom / denom;
	if (!isfinite(ixOut) || !isfinite(iyOut)) //Probably a numerical issue
		return false;

	ixOut /= scale, iyOut /= scale;
	return true; //All OK
}

bool cGeoUtil::CalcLineLineIntersection(double x1, double y1, double x2, double y2, double y0, double & ixOut)
{
	if ((y0 < y1 && y0 > y2)
		||
		(y0 < y2 && y0 > y1))
	{
		ixOut = (x2 + x1 * (y2 - y0) / (y0 - y1)) / (1 + (y2 - y1) / (y0 - y1));
	}
	else
	{
		return false;
	}
}

double cGeoUtil::CalcSlope2D(double x1, double y1, double x2, double y2)
{
	// calculate the slope between 2 2d pts
	if (std::abs(x2 - x1) < 1e-7)
	{
		return cMathUtil::sgn((y2 - y1) / (x2 - x1)) * 1e7;
	}
	return (y2 - y1) / (x2 - x1);
};