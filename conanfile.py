from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout

class AlgorithmConan(ConanFile):
    name = "algorithm"
    version = "0.3.2"
    license = "proprietary@pinloon"
    author = "Pin Loon Lee"
    url = "https://github.com/pllee4/algorithm"
    description = "Algorithm"
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False]}
    default_options = {"shared": False}
    generators = "CMakeToolchain", "CMakeDeps"
    exports_sources = "*"

    def layout(self):
        self.cpp.package.includedirs = ["include"]
        self.cpp.package.libs = ["algorithm"]

    def build(self, variables=["-DALGO_PACK=true"]):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        
    def package(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.install()