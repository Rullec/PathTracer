#pragma once
#include <Eigen/Dense>
#include <vector>
#define THROW_IF(val)                                                          \
    if (val)                                                                   \
        throw "[error] in " __FUNCTION__;
#define THROW_IF_LOG(val, log)                                                 \
    if (val)                                                                   \
    throw "[error] in " __FUNCTION__ log
// #define M_PI  3.14159265358979323

typedef Eigen::Vector4d tVector;
typedef Eigen::Matrix4d tMatrix;
typedef Eigen::MatrixXd tMatrixXd;
typedef Eigen::Quaterniond tQuaternion;
template <typename T>
using tEigenArr = std::vector<T, Eigen::aligned_allocator<T>>;
typedef tEigenArr<tVector> tVectorArr;

enum eRotationOrder
{
    XYZ = 0,
    ZYX,
};

extern enum eRotationOrder gRotationOrder;

class cMathUtil
{
public:
    // general
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
    // template <typename T> static bool IsSame(const T & v1, const T & v2, double eps = 1e-5)
    // {
    // 	return (v1 - v2).norm() < eps;
    // }
    static bool IsSame(const tVector &v1, const tVector &v2,
                       const double eps = 1e-5);
    static bool IsSame(const double &v1, const double &v2,
                       const double eps = 1e-5);
    static bool IsNormalized(const tVector &v);
    static bool JudgeInRange(double val, double thre0, double thre1,
                             double eps = 1e-5);
    static bool IsPoint(const tVector &v);
    static bool IsVector(const tVector &v);
    static tVector WiseProduct(const tVector &v1, const tVector &v2);
    // rotation
    static tMatrix Translate(const tVector &);
    static tMatrix RotMat(const tQuaternion &quater);
    static tVector QuaternionToCoef(const tQuaternion &quater);
    static tQuaternion CoefToQuaternion(const tVector &);
    static tQuaternion AxisAngleToQuaternion(const tVector &angvel);
    static tQuaternion RotFrom2Vec(const tVector &origin,
                                   const tVector &target);
    static tVector QuaternionToAxisAngle(const tQuaternion &);
    static tVector CalcAngularVelocity(const tQuaternion &old_rot,
                                       const tQuaternion &new_rot,
                                       double timestep);
    static tVector QuatRotVec(const tQuaternion &quater, const tVector &vec);
    static double Radians(double degree);
    static tMatrix InvMat(const tMatrix &mat);

    // distribution
    static tVector SampleHemiSphereUniform(const tVector &, double &pdf);
    static tVector SampleSphereUniform(double &pdf);
    static tVector SampleHemiSphereCosine(const tVector &, double &pdf);
    static tVector SampleCone(const tVector &axis, const double max_theta,
                              double &pdf);

    // geometry
    static tVector RayCast(const tVector &ori, const tVector &dir,
                           const tVector &p1, const tVector &p2,
                           const tVector &p3, double eps = 1e-10);
    static double RayCastT(const tVector &ori, const tVector &dir,
                           const tVector &p1, const tVector &p2,
                           const tVector &p3, double eps = 1e-10);
    static tMatrix SkewMat(const tVector &vec);
};