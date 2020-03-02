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
		return true;
	}
	return false;
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

tVector cGeoUtil::CalcNormalFrom3Pts(const tVector &p1, const tVector &p2, const tVector &p3)
{
	tVector normal = (p2 - p1).cross3(p3 - p2).normalized();
	assert(std::abs(normal.norm() - 1) < 1e-5);

	return normal;
}

/*
	normal: the normal vector of a surface
	incoming: the incident light, from an remote pt to the point lying on the surface
	return: outgoing light from ref_pt to remote pt
*/
tVector cGeoUtil::Reflect(const tVector & normal, const tVector & incoming)
{
	assert(cMathUtil::IsNormalized(normal));
	assert(cMathUtil::IsNormalized(incoming));

	tVector correct_normal = normal;
	if(normal.dot(incoming) > 0) correct_normal = -normal;

	return 2 * (normal.dot(-incoming)) * normal + incoming;
}

// ni = n1 / n2, n1是normal所指向材料的折射率，n2是-normal所指向材料的折射率
// 真空折射率为1, 玻璃折射率为1.5，所以法线朝向空气，入射光从空气来，ni=1/1.5
tVector cGeoUtil::Refract(const tVector & normal_, const tVector & incoming, double ni)
{
	assert(cMathUtil::IsVector(normal_));
	assert(cMathUtil::IsNormalized(normal_));
	assert(cMathUtil::IsVector(incoming));
	assert(cMathUtil::IsNormalized(incoming));

	tVector outgoing = tVector::Zero();
	// std::cout <<"normal = " << normal << std::endl;
	tVector normal = normal_;
	if(normal.dot(incoming) > 0)
	{
		normal = -normal;
		ni = 1.0 / ni;
	}
	// assert(ni > 1.0);
	{
		// 从外向内
		/*  incoming /|\ normal
			   \      |
			    _\|   |				n1
			--------------------
						\			n2
						 \
						 _\| outgoing
		*/
		// ni = n1 / n2;
		double ratio = ni;
		tVector nxi = normal.cross3(incoming);
		outgoing = ratio * (normal.cross3(-nxi)) - normal * std::sqrt(1 - ratio * ratio * nxi.dot(nxi));
	}
	// if(outgoing.hasNaN() == true)
	// {
	// 	// 光密到光疏 发生反射
	// 	assert(ni > 1.0);
	// 	assert(normal.dot(incoming) < 0);
	// 	outgoing = cGeoUtil::Reflect(normal, incoming);
	// }
	return outgoing;
}

tVector cGeoUtil::CalcBarycentricCoordinate(const tVector & p_, const tVector & a, const tVector & b, const tVector & c)
{
	assert(cMathUtil::IsPoint(p_));
	assert(cMathUtil::IsPoint(a));
	assert(cMathUtil::IsPoint(b));
	assert(cMathUtil::IsPoint(c));
	// std::cout <<"debgu begin bary centric coords assert\n";
	tVector normal1 = cGeoUtil::CalcNormalFrom3Pts(p_, a, b), normal2 = cGeoUtil::CalcNormalFrom3Pts(p_, b, c);
	if(false == cMathUtil::IsSame(normal1, normal2))
	{
		std::cout <<"v1 = " << a.transpose() << std::endl;
		std::cout <<"v2 = " << b.transpose() << std::endl;
		std::cout <<"v3 = " << c.transpose() << std::endl;
		std::cout <<"normal1 = " << normal1.transpose() <<", normal2 = " << normal2.transpose() << std::endl;
		exit(1);
	}

	// // https://www.gamedev.net/forums/topic/621445-barycentric-coordinates-c-code-check/
	// tVector res = tVector::Zero();
	// double den = 1 / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));

	// res.x = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) * den;
	// res.y = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) * den;
	// res.z = 1 - p.x - p.y;

	// std::cout <<"debgu begin bary centric coords inv\n";
	tVector p = p_;
	Eigen::Matrix3d A;
	A.col(0) = a.segment(0, 3);
	A.col(1) = b.segment(0, 3);
	A.col(2) = c.segment(0, 3);
	const double eps = 1e-6;
	for(int i=0; i<3; i++) 
	{
		if(A.row(i).norm() < 1e-10)
		{
			A.row(i) += Eigen::Vector3d::Ones() * eps;
			p[i] += eps;
		}
	}
	// std::cout <<"debgu begin bary centric coords calc1\n";
	Eigen::Vector3d res = A.inverse() * p.segment(0, 3);
	// std::cout <<"debgu begin bary centric coords calc2\n";
	if(res.hasNaN())
	{
		std::cout <<"res = " << res.transpose() << std::endl;
		std::cout <<"a = " << a.transpose() << std::endl;
		std::cout <<"b = " << b.transpose() << std::endl;
		std::cout <<"c = " << c.transpose() << std::endl;
		std::cout <<"p = " << p.transpose() << std::endl;
	}
	return tVector(res[0], res[1], res[2], 0).normalized();
}