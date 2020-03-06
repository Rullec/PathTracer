aux_source_directory(./scene scene_src)
aux_source_directory(./geometry geometry_src)
aux_source_directory(./tracing tracing_src)
aux_source_directory(./util util_src)

add_subdirectory(render)
add_library(scene_lib ${scene_src})
add_library(geo_lib ${geometry_src})
add_library(tracing_lib ${tracing_src})
add_library(util_lib ${util_src})
add_library(main ./main.cpp)

target_link_libraries(main render scene_lib tracing_lib geo_lib util_lib )