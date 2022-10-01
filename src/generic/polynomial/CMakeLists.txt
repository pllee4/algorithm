# Polynomial as library target
add_library(polynomial INTERFACE)
target_link_libraries(polynomial INTERFACE common)
target_include_directories(polynomial INTERFACE
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
# test for Polynomial
mark_as_advanced(
    BUILD_GTEST BUILD_SHARED_LIBS
    gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
add_executable(polynomial_test test/polynomial_test.cpp)
target_link_libraries(polynomial_test PRIVATE gtest gtest_main polynomial)
gtest_discover_tests(polynomial_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_test(NAME gtest_all COMMAND polynomial_test)
endif()

# main
add_executable(polynomial_main main.cpp)
target_link_libraries(polynomial_main PRIVATE polynomial)