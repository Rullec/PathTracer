#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor;
layout (location = 2) in vec2 aTexCoord;
out vec4 VertColor;
out vec2 TexCoord;
uniform mat4 ModelMat, ViewMat, ProjectMat;

void main()
{
    mat4 MVP = ProjectMat * ViewMat * ModelMat;
    gl_Position =  MVP * vec4(aPos.x, aPos.y, aPos.z, 1.0);
    VertColor = aColor;
    TexCoord = aTexCoord;
    gl_PointSize = 30.0f;
}