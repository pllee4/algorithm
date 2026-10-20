add_library(shared_type STATIC
src/constraint_table.cpp)
target_include_directories(shared_type PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
mark_as_advanced(
    BUILD_GTEST BUILD_SHARED_LIBS
    gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
add_executable(constraint_table_test test/constraint_table_test.cpp)
target_link_libraries(constraint_table_test PRIVATE gtest gtest_main shared_type)
gtest_discover_tests(constraint_table_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)
endif()
