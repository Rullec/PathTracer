add_subdirectory(./util)
add_subdirectory(./render)
add_subdirectory(./scene)
add_subdirectory(./geometry)
add_subdirectory(./tracing)

# add_library(geo_lib ${geometry_src})
# add_library(tracing_lib ${tracing_src})
add_library(main ./main.cpp)

target_link_libraries(main render_lib scene_lib tracing_lib geo_lib util_lib )