find_package(Eigen3 REQUIRED)
# Polynomial as library target
add_library(common INTERFACE)
target_link_libraries(common INTERFACE Eigen3::Eigen)
target_include_directories(common INTERFACE
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)