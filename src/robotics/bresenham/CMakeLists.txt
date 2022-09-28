# Bresenham as library target
add_library(bresenham STATIC src/bresenham.cpp)
target_link_libraries(bresenham INTERFACE shared_type)
target_include_directories(bresenham PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)