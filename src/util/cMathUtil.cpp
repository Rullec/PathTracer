#include "cMathUtil.hpp"
#include <iostream>
enum eRotationOrder gRotationOrder = eRotationOrder::XYZ;

tMatrix cMathUtil::Translate(const tVector & pos)
{
	tMatrix res = tMatrix::Identity();
	res.block(0, 3, 3, 1) = pos.block(0, 0, 3, 1);
	return res;
}

tMatrix cMathUtil::RotMat(const tQuaternion & quater_)
{
	// https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix

	tMatrix res = tMatrix::Zero();
	double w = quater_.w(), x = quater_.x(), y = quater_.y(), z = quater_.z();
	res << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0,
		2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0,
		2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0,
		0, 0, 0, 1;
	return res;
}

tVector cMathUtil::QuaternionToCoef(const tQuaternion & quater)
{
	// quaternion -> vec = [x, y, z, w]
	return tVector(quater.x(), quater.y(), quater.z(), quater.w());
}

tQuaternion cMathUtil::CoefToQuaternion(const tVector & vec)
{
	// vec = [x, y, z, w] -> quaternion
	if(vec[3] > 0) return tQuaternion(vec[3], vec[0], vec[1], vec[2]);
	else return tQuaternion(-vec[3], -vec[0], -vec[1], -vec[2]);
}

tQuaternion cMathUtil::AxisAngleToQuaternion(const tVector & angvel)
{
	if (angvel.norm() < 1e-7)
	{
		// angvel = [0, 0, 0, 0], no rotation
		return tQuaternion(1, 0, 0, 0);
	}
	double theta = angvel.norm();
	double theta_2 = theta / 2;
	double cos_theta_2 = std::cos(theta_2),
		sin_theta_2 = std::sin(theta_2);
	
	tVector norm_angvel = angvel.normalized();
	return tQuaternion(cos_theta_2, norm_angvel[0] * sin_theta_2,\
		norm_angvel[1] * sin_theta_2,\
		norm_angvel[2] * sin_theta_2);
}

tVector cMathUtil::QuaternionToAxisAngle(const tQuaternion & quater)
{
	/* 	quater = [w, x, y, z]
			w = cos(theta / 2)
			x = ax * sin(theta/2)
			y = ay * sin(theta/2)
			z = az * sin(theta/2)
		axis angle = theta * [ax, ay, az, 0]
	*/
	tVector axis_angle = tVector::Zero();

	double theta = 2 * std::acos(quater.w());

	if (theta < 1e-4) return tVector::Zero();

	//std::cout << theta << " " << std::sin(theta / 2) << std::endl;
	double ax = quater.x() / std::sin(theta / 2),
		ay = quater.y() / std::sin(theta / 2),
		az = quater.z() / std::sin(theta / 2);
	return theta * tVector(ax, ay, az, 0);
}

tVector cMathUtil::CalcAngularVelocity(const tQuaternion & old_rot,\
	const tQuaternion & new_rot, double timestep)
{
	tQuaternion trans = new_rot * old_rot.conjugate();
	double theta = std::acos(trans.w()) * 2;	// std::acos() output range [0, pi]
	//if (theta > 2 * M_PI - theta)
	//{
	//	// theta = theta - 2*pi
	//	theta = theta - 2 * M_PI;	// -pi - pi
	//	trans.coeffs().segment(0, 3) *= -1;
	//}

	tVector vel = tVector::Zero();
	double coef = theta / (sin(theta / 2) * timestep);
	vel.segment(0, 3) = trans.coeffs().segment(0, 3) * coef;
	return vel;
}

bool cMathUtil::IsSame(const tVector & v1, const tVector & v2, const double eps)
{
	for(int i=0; i<v1.size(); i++) if(std::fabs(v1[i] - v2[i])>eps) return false;
	return true;
}

bool cMathUtil::IsNormalized(const tVector & v)
{
	return std::abs(v.norm() - 1) < 1e-10;
}

bool cMathUtil::IsPoint(const tVector & v)
{
	return std::abs(v[3] - 1) < 1e-10;
}

bool cMathUtil::IsVector(const tVector & v)
{
	return std::abs(v[3]) < 1e-10;
}

bool cMathUtil::JudgeInRange(double val, double thre0, double thre1, double eps)
{
	bool res1 = (val > thre0 - eps) && (val < thre1 + eps);
	bool res2 = (val > thre1 - eps) && (val < thre0 + eps);
	bool final_res =  (res1 || res2);
	//if (final_res == false)
	//{
	//	std::cout << "res1 = " << res1 << " res2 = " << res2 << std::endl;
	//	std::cout << val << " " << thre0 << " " << thre1 << std::endl;
	//}
	return final_res;
}

tVector cMathUtil::WiseProduct(const tVector & v1, const tVector &v2)
{
	return tVector(v1.array() * v2.array());
}

double cMathUtil::Radians(double degree)
{
	return degree / 180.0 * M_PI;
}


tVector cMathUtil::RayCast(const tVector & ori, const tVector & dir,\
	const tVector & p1, const tVector & p2, const tVector & p3, double eps/* = 1e-5*/)
{
	Eigen::Matrix3d mat;
	mat.col(0) = (p1 - p2).segment(0, 3);
	mat.col(1) = (p1 - p3).segment(0, 3);
	mat.col(2) = dir.segment(0, 3);
	Eigen::Vector3d vec;
	vec = (p1 - ori).segment(0, 3);
	Eigen::Vector3d res = mat.inverse() * vec;
	// std::cout << "res = " << res.transpose() << std::endl;
	double beta = res[0], gamma = res[1], t = res[2], alpha = 1 - beta - gamma;
	// std::cout <<"ray cast = " << res.transpose() << std::endl;
	tVector inter = tVector(std::nan(""), std::nan(""), std::nan(""), std::nan(""));
	if(0-eps < alpha && alpha < 1 + eps && 0-eps < beta && beta < 1+eps && 0-eps < gamma && gamma <1+eps && t >0-eps)
	{
		inter = ori + t * dir;
	}
	return inter;
}


double cMathUtil::RayCastT(const tVector & ori, const tVector & dir,\
	const tVector & p1, const tVector & p2, const tVector & p3, double eps/* = 1e-5*/)
{
	Eigen::Matrix3d mat;
	mat.col(0) = (p1 - p2).segment(0, 3);
	mat.col(1) = (p1 - p3).segment(0, 3);
	mat.col(2) = dir.segment(0, 3);
	Eigen::Vector3d vec;
	vec = (p1 - ori).segment(0, 3);
	Eigen::Vector3d res = mat.inverse() * vec;
	// std::cout << "res = " << res.transpose() << std::endl;
	double beta = res[0], gamma = res[1], t = res[2], alpha = 1 - beta - gamma;
	// std::cout <<"ray cast = " << res.transpose() << std::endl;
	if(false == (0-eps < alpha && alpha < 1 + eps && 0-eps < beta && beta < 1+eps && 0-eps < gamma && gamma <1+eps && t >0-eps))
	{
		t = std::nan("");
	}
	
	return t;
}

// sample from a hemisphere, implied by the normal vector
tVector cMathUtil::SampleHemiSphereUniform(const tVector & normal, double & pdf)
{
	assert(std::fabs(normal[3]) < 1e-10);
	assert(std::fabs(normal.norm() - 1) < 1e-10);

	double xi1 = drand48(), xi2 = drand48();
	double theta = std::acos(1 - xi1), phi = 2 * M_PI * xi2; 
	tVector res = tVector(std::cos(theta) * std::cos(phi), std::sin(theta), -std::cos(theta) * std::sin(phi), 0);
	pdf = 1.0 / (2 * M_PI);

	tVector y_axis = tVector(0, 1, 0, 0);
	tVector axis = y_axis.cross3(normal - y_axis).normalized();
	double angle = std::acos(y_axis.dot(normal));
	tQuaternion rot = cMathUtil::AxisAngleToQuaternion(axis * angle);
	res = cMathUtil::QuatRotVec(rot, res);
	return res;
}

tVector cMathUtil::SampleHemiSphereCosine(const tVector & normal, double & pdf)
{
	assert(std::fabs(normal[3]) < 1e-10);
	assert(std::fabs(normal.norm() - 1) < 1e-10);

	double xi1 = drand48(), xi2 = drand48();
	double theta = 0.5 * std::acos(1 - 2 * xi1), phi = 2 * M_PI * xi2; 
	tVector res = tVector(std::cos(theta) * std::cos(phi), std::sin(theta), -std::cos(theta) * std::sin(phi), 0);

	// rotate 
	tVector y_axis = tVector(0, 1, 0, 0);
	tVector axis = y_axis.cross3(normal - y_axis).normalized();
	double angle = std::acos(y_axis.dot(normal));
	tQuaternion rot = cMathUtil::AxisAngleToQuaternion(axis * angle);
	res = cMathUtil::QuatRotVec(rot, res);

	return res;
}

tVector cMathUtil::QuatRotVec(const tQuaternion & quater, const tVector & vec)
{
	tVector res = tVector::Zero();
	res.segment(0, 3) = quater * vec.segment(0, 3);
	return res;
}