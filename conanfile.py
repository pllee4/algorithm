from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools.files import collect_libs


class AlgorithmConan(ConanFile):
    name = "algorithm"
    version = "0.3.2"
    license = "proprietary@pinloon"
    author = "Pin Loon Lee"
    url = "https://github.com/pllee4/algorithm"
    description = "Algorithm"
    # CMakeLists.txt hardcodes STATIC, so there is no shared variant to offer
    package_type = "static-library"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"fPIC": [True, False]}
    default_options = {"fPIC": True}

    # Sources are located in the same place as this recipe, copy them to the recipe
    # cmake/ holds algorithmConfig.cmake.in, third_party/ only satisfies the googletest check
    exports_sources = "CMakeLists.txt", "cmake/*", "src/*", "include/*", "third_party/*"

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def requirements(self):
        # Public headers include <Eigen/Core>, and algorithmConfig.cmake calls find_package(Eigen3)
        self.requires("eigen/3.4.0", transitive_headers=True)

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        # Skip tests and coverage flags when packaging
        tc.cache_variables["ALGO_PACK"] = True
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        # CMakeDeps writes algorithm-config.cmake into the consumer's install folder,
        # so a consumer only needs that folder on CMAKE_PREFIX_PATH
        self.cpp_info.set_property("cmake_file_name", "algorithm")
        self.cpp_info.set_property("cmake_target_name", "pllee4::algorithm")
        # Picks up whatever archives CMake installed, instead of hardcoding them
        self.cpp_info.libs = collect_libs(self)