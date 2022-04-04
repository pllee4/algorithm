# dijikstra as library target
add_library(dijikstra STATIC src/dijikstra.cpp)
target_include_directories(dijikstra PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
# test for dijikstra
mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
add_executable(dijikstra_test test/dijikstra_test.cpp)
target_link_libraries(dijikstra_test PRIVATE gtest gmock gtest_main dijikstra)
gtest_discover_tests(dijikstra_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_test(NAME gtest_all COMMAND dijikstra_test)
endif()

# main
add_executable(dijikstra_main main.cpp)
target_link_libraries(dijikstra_main PRIVATE dijikstra)