# Bresenham as library target
add_library(bresenham STATIC src/bresenham.cpp)
target_link_libraries(bresenham INTERFACE shared_type)
target_include_directories(bresenham PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
# test for bresenham
mark_as_advanced(
    BUILD_GTEST BUILD_SHARED_LIBS
    gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
add_executable(bresenham_test test/bresenham_test.cpp)
target_link_libraries(bresenham_test PRIVATE gtest gtest_main bresenham)
gtest_discover_tests(bresenham_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_test(NAME gtest_all COMMAND bresenham_test)
endif()

# main
add_executable(bresenham_main main.cpp)
target_link_libraries(bresenham_main PRIVATE bresenham)