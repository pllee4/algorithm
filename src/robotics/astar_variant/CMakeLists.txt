add_library(astar_variant STATIC src/astar_variant_base.cpp)
target_include_directories(astar_variant PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)