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

bool cMathUtil::IsSame(const double & v1, const double & v2, const double eps)
{
	return std::fabs(v1 - v2) < eps;
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

tQuaternion cMathUtil::RotFrom2Vec(const tVector & origin, const tVector & target)
{
	assert(cMathUtil::IsNormalized(origin) && cMathUtil::IsVector(origin));
	assert(cMathUtil::IsNormalized(target) && cMathUtil::IsVector(target));

	tVector axis;
	double angle = 0;
	if((origin+target).norm() < 1e-10)
	{
		int i = -1;
		for(i=0; i<3; i++ ) if(std::fabs(origin[i])> 1e-6) break;
		axis = tVector::Random();
		axis[3] = 0;
		axis[i] = -(axis[(i + 1)%3] * origin[(i + 1)%3] + axis[(i + 2)%3] * origin[(i + 2)%3]) / origin[i];
		axis.normalize();
		angle = M_PI;
		// std::cout <<"[if] axis = " << axis.transpose() <<" angle = " << angle << std::endl;
	}
	else
	{
			/*
	tVector y_axis = tVector(0, 1, 0, 0);
	tVector axis = y_axis.cross3(normal - y_axis).normalized();
	
	double angle = std::acos(y_axis.dot(normal));
	*/
		// origin = tVector(0, 1, 0, 0);
		axis = origin.cross3(target - origin).normalized();
		assert(cMathUtil::IsNormalized(axis));
		angle = std::acos(origin.dot(target));
		// std::cout <<"origin = " << origin.transpose() <<" target = " << target.transpose() << std::endl; 
		// std::cout <<"axis = " << axis.transpose() <<" angle = " << angle << std::endl;
	}
	return cMathUtil::AxisAngleToQuaternion(axis * angle);
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
	// assert(std::fabs(normal[3]) < 1e-10);
	assert(cMathUtil::IsNormalized(normal));
	assert(cMathUtil::IsVector(normal));

	double xi1 = drand48(), xi2 = drand48();
	double theta = std::acos(1 - xi1), phi = 2 * M_PI * xi2; 
	tVector res = tVector(std::cos(theta) * std::cos(phi), std::sin(theta), -std::cos(theta) * std::sin(phi), 0);
	pdf = 1.0 / (2 * M_PI);

	tVector y_axis = tVector(0, 1, 0, 0);
	tVector axis = y_axis.cross3(normal - y_axis).normalized();
	
	double angle = std::acos(y_axis.dot(normal));
	if(cMathUtil::IsNormalized(axis) == false)
	{
		// which means normal = (0, -1, 0) || (0, 1, 0);
		axis = tVector(1, 0, 0, 0);
		if(std::fabs(normal[1] - 1) < 1e-10)
		{
			angle = 0;
		}
		else if(std::fabs(normal[1] + 1) < 1e-10)
		{
			angle = M_PI;
		}
		else
		{
			std::cout <<"[error] cMathUtil::SampleHemiSphereUniform normal = " << normal.transpose() << std::endl;
		}
	}
	tQuaternion rot = cMathUtil::AxisAngleToQuaternion(axis * angle);
	res = cMathUtil::QuatRotVec(rot, res);
	return res;
}

tVector cMathUtil::SampleSphereUniform(double &pdf)
{
	// assert(std::fabs(normal[3]) < 1e-10);
	// assert(cMathUtil::IsNormalized(normal));
	// assert(cMathUtil::IsVector(normal));

	double xi1 = drand48() -0.5, xi2 = drand48() -0.5, xi3 = drand48() -0.5;
	tVector res = tVector(xi1, xi2, xi3, 0).normalized();
	pdf = 1.0 / (4 * M_PI);
	return res;
}

tVector cMathUtil::SampleHemiSphereCosine(const tVector & normal, double & pdf)
{
	assert(std::fabs(normal[3]) < 1e-10);
	assert(std::fabs(normal.norm() - 1) < 1e-10);

	double xi1 = drand48(), xi2 = drand48();
	double theta = 0.5 * std::acos(1 - 2 * xi1), phi = 2 * M_PI * xi2; 
	tVector res = tVector(std::cos(theta) * std::cos(phi), std::sin(theta), -std::cos(theta) * std::sin(phi), 0);
	pdf = std::cos(theta) / M_PI;
	
	// rotate 
	tVector y_axis = tVector(0, 1, 0, 0);
	tVector axis = y_axis.cross3(normal - y_axis).normalized();
	double angle = std::acos(y_axis.dot(normal));

	tQuaternion rot = cMathUtil::AxisAngleToQuaternion(axis * angle);
	res = cMathUtil::QuatRotVec(rot, res);

	assert(cMathUtil::IsNormalized(res));
	assert(cMathUtil::IsVector(res));
	return res;
}

/*
	SampleCone: given an axis, and max_theta, 
	pdf = 1 / (2 * PI * (1 - cos(max_theta))) per solid angle 

*/
tVector cMathUtil::SampleCone(const tVector & medial_axis, const double max_theta, double & pdf)
{
	assert(cMathUtil::IsVector(medial_axis));
	assert(cMathUtil::IsNormalized(medial_axis));
	assert(cMathUtil::JudgeInRange(max_theta, 0, M_PI_2));

	// 1. sample a direction according to (0, 1, 0)
	double xi1 = drand48(), xi2 = drand48();
	double phi = 2 * M_PI * drand48(), theta = std::acos(1 - xi2 +xi2 * std::cos(max_theta));
	double y = std::cos(theta);
	double radius = std::sqrt(1 - pow(std::cos(theta), 2));
	double x = radius * std::cos(phi), z = radius * std::sin(phi);
	tVector raw_dir = tVector(x, y, z, 0).normalized();
	pdf = 1.0 / (2 * M_PI * (1 - std::cos(max_theta)));
	
	// 2. rotate this vec to axis
	tQuaternion rot = cMathUtil::RotFrom2Vec(tVector(0, 1, 0, 0), medial_axis);
	tVector res = cMathUtil::QuatRotVec(rot, raw_dir).normalized();

	assert(cMathUtil::IsVector(res));
	if(false == cMathUtil::IsNormalized(res))
	{
		std::cout <<"res = " << res.transpose() << std::endl;
		exit(1);
	}
	return res;
}

tVector cMathUtil::QuatRotVec(const tQuaternion & quater, const tVector & vec)
{
	tVector res = tVector::Zero();
	res.segment(0, 3) = quater * vec.segment(0, 3);
	return res;
}

tMatrix cMathUtil::SkewMat(const tVector & w)
{
	tMatrix result = tMatrix::Zero();
	result(0, 1) = -w[2];
	result(0, 2) = w[1];
	result(1, 0) = w[2];
	result(1, 2) = -w[0];
	result(2, 0) = -w[1];
	result(2, 1) = w[0];
	result(3, 3) = 1;
	
	return result;
}

tMatrix cMathUtil::InvMat(const tMatrix & mat)
{
	tMatrix res = tMatrix::Identity();
	res.block(0,0,3,3) = mat.block(0, 0, 3, 3).inverse();
	return res;
}