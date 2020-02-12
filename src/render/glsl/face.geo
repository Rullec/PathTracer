#version 330 core
layout(triangles) in;
layout (triangle_strip, max_vertices = 3) out;
out vec3 normal1;
in vec4 VertColor[3];
in vec2 TexCoord[3];
in vec3 VertPos[3];
out vec4 gVertColor;
out vec2 gTexCoord;
out vec3 gVertPos;

void main()
{
    vec3 n = normalize(cross(VertPos[1] - VertPos[0], VertPos[2] - VertPos[1]));
    for(int i = 0; i < gl_in.length(); i++)
    {
        gl_Position = gl_in[i].gl_Position;
        normal1 = n;
        gVertColor = VertColor[i];
        gTexCoord = TexCoord[i];
        gVertPos = VertPos[i];
        EmitVertex();
    }
    EndPrimitive();
}