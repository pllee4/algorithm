add_library(disjoint_union_set ${CMAKE_CURRENT_SOURCE_DIR}/disjoint_union_set.cpp)
target_include_directories(disjoint_union_set PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
# test for Polynomial
mark_as_advanced(
    BUILD_GTEST BUILD_SHARED_LIBS
    gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

add_executable(disjoint_union_set_test
  ${CMAKE_CURRENT_SOURCE_DIR}/test/disjoint_union_set_test.cpp
)

target_link_libraries(disjoint_union_set_test PRIVATE gtest_main disjoint_union_set)
gtest_discover_tests(disjoint_union_set_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)
endif()