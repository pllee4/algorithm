# cbs as library target
add_library(cbs STATIC 
    src/conflict.cpp
    src/cbs.cpp
    src/mdd.cpp
    src/instance.cpp)
target_link_libraries(cbs PUBLIC shared_type astar)
target_include_directories(cbs PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
# test for cbs
mark_as_advanced(
    BUILD_GTEST BUILD_SHARED_LIBS
    gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
add_executable(conflict_test test/conflict_test.cpp)
target_link_libraries(conflict_test PRIVATE gtest gtest_main cbs)
gtest_discover_tests(conflict_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_executable(mdd_test test/mdd_test.cpp)
target_link_libraries(mdd_test PRIVATE gtest gtest_main cbs)
gtest_discover_tests(mdd_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_executable(cbs_test test/cbs_test.cpp)
target_link_libraries(cbs_test PRIVATE gtest gtest_main cbs)
gtest_discover_tests(cbs_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

endif()

# main
add_executable(cbs_main cbs_main.cpp)
target_link_libraries(cbs_main PRIVATE cbs)
