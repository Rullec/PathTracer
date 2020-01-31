// geometry tools
#include <util/cMathUtil.hpp>
#include <cmath>

class cGeoUtil {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	static double CalcDistPointToPlane(const tVector & target_point,const tVector & plane_point, const tVector & norm)
	{
		tVector dist = target_point - plane_point;
		assert(std::abs(norm[3]) < 1e-10);
		return std::abs(dist.dot(norm));
	}
	static tVector CalcPlaneEquation(const tVector & p1, const tVector & p2, const tVector & p3);
	static tVector SamplePointFromPlane(const tVector &plane);
	static bool CalcLineLineIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4, double &ixOut, double &iyOut);
	static bool CalcLineLineIntersection(double x1, double y1, double x2, double y2, double y0, double &ixOut);
	static double CalcSlope2D(double x1, double y1, double x2, double y2);
};