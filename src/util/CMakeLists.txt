set(
    util_src
    cFileUtil.cpp
cGeoUtil.cpp
cGlmUtil.cpp
cJsonUtil.cpp
cMathUtil.cpp
cTimeUtil.cpp
./json/json_reader.cpp
./json/json_value.cpp
./json/json_writer.cpp
)

add_library(util_lib ${util_src})