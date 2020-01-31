include_directories(./include)

aux_source_directory(./scene Scene_src)
aux_source_directory(./geometry Geometry_src)
aux_source_directory(./model Model_src)
aux_source_directory(./render Render_src)
aux_source_directory(./util Util_src)

add_library(src ${Scene_src}  ${Geometry_src} ${Model_src} ${Render_src} ${Util_src} )
