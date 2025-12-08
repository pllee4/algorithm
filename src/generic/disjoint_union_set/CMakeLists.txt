add_library(disjoint_union_set ${CMAKE_CURRENT_SOURCE_DIR}/disjoint_union_set.cpp)
target_include_directories(disjoint_union_set PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)