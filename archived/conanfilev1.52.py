from conans import ConanFile, CMake, tools

class AlgorithmConan(ConanFile):
    name = "algorithm"
    version = "0.3.1"
    license = "proprietary@pinloon"
    author = "Pin Loon Lee"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "Algorithm"
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False]}
    default_options = {"shared": False}
    generators = "cmake_paths"
    exports_sources = "*"

    def build(self):
        cmake = CMake(self)
        cmake.definitions["ALGO_PACK"] = True
        cmake.configure()
        cmake.build()
        
    def package(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.install()

    def package_info(self):
        self.cpp_info.includedirs = ["include"]
        self.cpp_info.libs = ["algorithm"]
