add_library(shared_type STATIC
src/constraint.cpp
src/constraint_table.cpp)
target_include_directories(shared_type PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
