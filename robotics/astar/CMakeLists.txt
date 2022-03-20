# Astar as library target
add_library(astar STATIC src/astar.cpp)
target_include_directories(astar PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# test for astar
mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
# add_executable(astar_test test/astar_test.cpp)
# target_link_libraries(astar_test PRIVATE gtest gmock gtest_main astar)
# gtest_discover_tests(astar_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

# add_test(NAME gtest_all COMMAND astar_test)

# main
add_executable(astar_main main.cpp)
target_link_libraries(astar_main PRIVATE astar)