#version 330 core
in vec4 VertColor;
in vec2 TexCoord;
out vec4 FragColor;

// texture sampler
uniform sampler2D texture1;
uniform bool texture_mode;
uniform bool point_mode;

void main()
{
    if(true == texture_mode)
    {
        vec4 TexColor = texture(texture1, TexCoord);
        // FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);
        FragColor = TexColor;
        // FragColor = vec4(0.8f, 0.9f, 1.0f, 1.0f);
    }
    else
    {
        FragColor = VertColor;
    }

    // for point mode    
    if(true == point_mode)
    {
        vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
        if (dot(circCoord, circCoord) > 1.0) 
        {
            discard;
        }
    }

} 