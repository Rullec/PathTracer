#pragma once
#include <Eigen/Dense>
#include <vector>
#define THROW_IF(val) if (val) throw "[error] in " __FUNCTION__;
#define THROW_IF_LOG(val, log) if(val) throw "[error] in " __FUNCTION__ log
// #define M_PI  3.14159265358979323

typedef Eigen::Vector4d tVector;
typedef Eigen::Matrix4d tMatrix;
typedef Eigen::MatrixXd tMatrixXd;
typedef Eigen::Quaterniond tQuaternion;
template <typename T>
using tEigenArr = std::vector<T, Eigen::aligned_allocator<T>>;
typedef tEigenArr<tVector> tVectorArr;

enum eRotationOrder {
	XYZ = 0,
	ZYX,
};

extern enum eRotationOrder gRotationOrder;

class cMathUtil {
public:
	template <typename T> static T clamp(T val, T max, T down)
	{
		if (down > max)
		{
			T tmp = max;
			max = down;
			down = tmp;
		}
		return std::max(std::min(val, max), down);
	}
	template <typename T> static int sgn(T val)
	{
		return (T(0) < val) - (val < T(0));
	}

	static tMatrix Translate(const tVector &);
	static tMatrix RotMat(const tQuaternion & quater);
	static tVector QuaternionToCoef(const tQuaternion & quater);
	static tQuaternion CoefToQuaternion(const tVector & );
	static tQuaternion AxisAngleToQuaternion(const tVector & angvel);
	static tVector QuaternionToAxisAngle(const tQuaternion & );
	static tVector CalcAngularVelocity(const tQuaternion & old_rot, const tQuaternion & new_rot, double timestep);
	static bool JudgeInRange(double val, double thre0, double thre1, double eps = 1e-5);
};