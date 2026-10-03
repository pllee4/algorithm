# Astar as library target
add_library(astar STATIC space_time_astar.cpp)
target_link_libraries(astar PUBLIC astar_variant shared_type)
target_include_directories(astar PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
# test for astar
mark_as_advanced(
    BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
add_executable(astar_test test/astar_test.cpp)
target_link_libraries(astar_test PRIVATE gtest gtest_main astar)
gtest_discover_tests(astar_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_executable(weighted_astar_test test/weighted_astar_test.cpp)
target_link_libraries(weighted_astar_test PRIVATE gtest gtest_main astar)
gtest_discover_tests(weighted_astar_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_executable(space_time_astar_test test/space_time_astar_test.cpp)
target_link_libraries(space_time_astar_test PRIVATE gtest gtest_main astar)
gtest_discover_tests(space_time_astar_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_test(NAME gtest_all COMMAND astar_test weighted_astar_test)
endif()

# main
add_executable(astar_main main.cpp)
target_link_libraries(astar_main PRIVATE astar)

add_executable(space_time_astar_main space_time_astar_main.cpp)
target_link_libraries(space_time_astar_main PRIVATE astar)