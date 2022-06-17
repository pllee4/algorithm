cmake_minimum_required(VERSION 3.10.2)

project(algorithm VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

## Submodules
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/third_party/googletest/CMakeLists.txt")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

include(GNUInstallDirs)

## Put all binary files into /bin and libraries into /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

## Project options
option(ALGO_BUILD_TEST "Build the tests" ON)
option(ALGO_COVERAGE_CHECK "Coverage check" ON)

if(${ALGO_PACK})
  set(ALGO_BUILD_TEST OFF)
  set(ALGO_COVERAGE_CHECK OFF)
endif()

if(ALGO_COVERAGE_CHECK)
    set(ALGO_BUILD_TEST ON)
    find_program(GCOV gcov)
    if(GCOV)
        message(STATUS "Found gcov")
        set(CMAKE_BUILD_TYPE Debug)
        set(CMAKE_CXX_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
    endif()
endif()

if(ALGO_BUILD_TEST)
    enable_testing()
    include(GoogleTest)
    add_subdirectory(third_party)
endif()

add_subdirectory(generic)
add_subdirectory(robotics)

set(GENERIC_MODULES common digital_filter polynomial)
set(ROBOTICS_MODULES astar dijikstra)

add_library(algorithm INTERFACE)
target_link_libraries(algorithm INTERFACE ${GENERIC_MODULES} ${ROBOTICS_MODULES})

# Generally should be done only in the top level CMakeLists.txt
if(NOT WIN32 AND CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CPACK_PACKAGING_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
endif()

message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX} with 'make install'")

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR} CACHE PATH "Installation directory for libraries")
set(INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR} CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
if (WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKEDIR CMake)
else ()
    set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
endif ()
set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")

# Report to user
foreach (p LIB BIN INCLUDE CMAKE)
    file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}DIR} _path)
    message(STATUS " - To install ${p} components to ${_path}")
    unset(_path)
endforeach ()

# Install targets
install(TARGETS ${PROJECT_NAME} ${GENERIC_MODULES} ${ROBOTICS_MODULES}
        EXPORT algorithmTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(EXPORT algorithmTargets
        FILE algorithmTargets.cmake
        NAMESPACE pllee4::
        DESTINATION lib/cmake/algorithm)

install(DIRECTORY generic robotics
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/algorithm)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/algorithmConfig.cmake.in algorithmConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/algorithm
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
        algorithmConfigVersion.cmake
        VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}"
        COMPATIBILITY SameMajorVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/algorithmConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/algorithmConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/algorithm)
