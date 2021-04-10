set(
    scene_src
    cDrawScene.cpp
    cRTScene.cpp
    cScene.cpp
    BuildScene.hpp
)

add_library(scene_lib ${scene_src})
target_link_libraries(scene_lib render_lib)