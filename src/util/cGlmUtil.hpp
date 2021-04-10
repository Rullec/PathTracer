#include <glm/glm.hpp>
#include <util/cMathUtil.hpp>

class cGlmUtil
{
public:
    static glm::vec3 tVectorToGlmVector3(const tVector &t);
    static tMatrix GlmMatixTotMatrix(const glm::mat4 &mat);
};
