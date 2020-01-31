#include "cGlmUtil.hpp"

glm::vec3 cGlmUtil::tVectorToGlmVector3(const tVector & t)
{
	return glm::vec3(t[0], t[1], t[2]);
}

tMatrix cGlmUtil::GlmMatixTotMatrix(const glm::mat4 & mat)
{
	tMatrix res_mat = tMatrix::Identity();
	for(int i=0; i<3; i++)
		for (int j = 0; j < 3; j++)
			res_mat(i, j) = mat[i][j];
	return res_mat;
}