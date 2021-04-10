set(
    tracing_src
    cBxDF.cpp
    cLight.cpp
    cPathTracer.cpp
)

add_library(tracing_lib ${tracing_src})