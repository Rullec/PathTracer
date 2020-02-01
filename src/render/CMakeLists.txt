aux_source_directory(./camera camera_src)
aux_source_directory(./shader shader_src)
aux_source_directory(./ render_src)

add_library(render ${render_src} ${camera_src} ${shader_src})