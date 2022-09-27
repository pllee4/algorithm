add_library(shared_type INTERFACE)
target_include_directories(shared_type INTERFACE
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)