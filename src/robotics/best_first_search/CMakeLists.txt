# Astar as library target
add_library(best_first_search INTERFACE)
target_link_libraries(best_first_search INTERFACE astar_variant)
target_include_directories(best_first_search INTERFACE
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
# test for best_first_search
mark_as_advanced(
    BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)
add_executable(best_first_search_test test/best_first_search_test.cpp)
target_link_libraries(best_first_search_test PRIVATE gtest gtest_main best_first_search)
gtest_discover_tests(best_first_search_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_test(NAME gtest_all COMMAND best_first_search_test)
endif()