# dijikstra as library target
add_library(dijikstra INTERFACE)
target_link_libraries(dijikstra INTERFACE astar_variant)
target_include_directories(dijikstra INTERFACE
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
# test for dijikstra
mark_as_advanced(
    BUILD_GTEST BUILD_SHARED_LIBS
    gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
add_executable(dijikstra_test test/dijikstra_test.cpp)
target_link_libraries(dijikstra_test PRIVATE gtest gtest_main dijikstra)
gtest_discover_tests(dijikstra_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_test(NAME gtest_all COMMAND dijikstra_test)
endif()

# main
add_executable(dijikstra_main main.cpp)
target_link_libraries(dijikstra_main PRIVATE dijikstra)