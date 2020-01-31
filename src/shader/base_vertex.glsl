
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec4 aColor;
out vec4 vertexColor;
uniform mat4 MVP;

void main()
{
    gl_Position =  MVP * vec4(aPos.x, aPos.y, aPos.z, 1.0);
    vertexColor = aColor;
}