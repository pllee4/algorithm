# Digital filter as library target
add_library(digital_filter INTERFACE)
target_link_libraries(digital_filter 
    INTERFACE common polynomial)
target_include_directories(digital_filter INTERFACE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

if(ALGO_BUILD_TEST)
# test for Butterworth
mark_as_advanced(
    BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
    gmock_build_tests gtest_build_samples gtest_build_tests
    gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
)

add_executable(bilinear_transform_test test/bilinear_transform_test.cpp)
target_link_libraries(bilinear_transform_test PRIVATE gtest gmock gtest_main digital_filter)
gtest_discover_tests(bilinear_transform_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_executable(butterworth_test test/butterworth_test.cpp)
target_link_libraries(butterworth_test PRIVATE gtest gmock gtest_main digital_filter)
gtest_discover_tests(butterworth_test XML_OUTPUT_DIR ${PROJECT_BINARY_DIR}/test-reports)

add_test(NAME gtest_all COMMAND bilinear_transform_test butterworth_test)
endif()

# main
add_executable(digital_filter_main main.cpp)
target_link_libraries(digital_filter_main PRIVATE digital_filter)
target_include_directories(digital_filter_main PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)