aux_source_directory(./scene scene_src)
aux_source_directory(./geometry geometry_src)
aux_source_directory(./model model_src)

add_subdirectory(render)
aux_source_directory(./util util_src)

add_library(src ${scene_src} ${geometry_src} ${model_src} ${util_src})
target_link_libraries(src render)