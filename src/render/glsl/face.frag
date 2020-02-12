#version 330 core
in vec3 gVertPos;
in vec4 gVertColor;
in vec2 gTexCoord;
in vec3 normal1;
out vec4 FragColor;

// texture sampler
uniform sampler2D texture1;
uniform bool texture_mode_face;
// uniform bool point_mode;
uniform vec3 light_pos;
uniform vec3 camera_pos;

void main()
{
    if(true == texture_mode_face)
    {
        vec4 TexColor = texture(texture1, gTexCoord);
        // FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);
        FragColor = TexColor;
        // FragColor = vec4(0.8f, 0.9f, 1.0f, 1.0f);
    }
    else
    {
        // no texture, phong model:
        // diffuse()*myself_color + specular() * light
        vec3 light_color = vec3(1, 1, 1),
            myself_color = gVertColor.xyz;
        // calculate ambient 
        vec3 part0 = 0.3 * myself_color;
        // calculate diffuse 
        vec3 part1 = vec3(0, 0, 0);
        {
            float diffuse_coef = dot(normalize(normal1), normalize(light_pos - gVertPos));
            diffuse_coef = clamp(diffuse_coef, 0, 1);
            part1 = diffuse_coef * myself_color;
        }
        // calculate specular
        vec3 part2 = vec3(0, 0, 0);
        // 计算视线和出射光线
        {
            vec3 view_light = normalize(camera_pos - gVertPos),
                in_light = normalize(light_pos - gVertPos),
                out_light = normalize(reflect(in_light, normal1));
            float specular_coeff = dot(normal1, in_light) > 0 ? 1 : 0;   // enable or disable
            specular_coeff *= pow((dot(view_light, out_light)), 10);
            part2 = specular_coeff * myself_color;
        }
        FragColor = vec4(part0 + part1 + part2, 1);
        // FragColor = vec4( part2 , 1);
    }

    // color = ambient_color * cos() + (1 - cos)^n * specular_color
    // for point mode    
    // if(true == point_mode)
    // {
    //     vec2 circCoord = 2.0 * gl_PointCoord - 1.0;
    //     if (dot(circCoord, circCoord) > 1.0) 
    //     {
    //         discard;
    //     }
    // }

    // 
} 