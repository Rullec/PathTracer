set(camera_src
    ./camera/cArcballCamera.cpp
    ./camera/cBaseCamera.cpp
    ./camera/cFPSCamera.cpp
)
set(shader_src
    ./shader/cBaseShader.cpp
)
set(render_src
    cBaseRender.cpp
    cPolyRender.cpp
)

add_library(render_lib ${render_src} ${camera_src} ${shader_src})