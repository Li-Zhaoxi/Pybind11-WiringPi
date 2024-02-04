import os
import subprocess
import sys
from pathlib import Path
import importlib

import shutil

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "", import_all: bool = False, parallel = 1) -> None:
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(Path(sourcedir).resolve())
        self.import_all = import_all
        self.parallel = parallel


class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:
        # Must be in this form due to bug in .resolve() only fixed in Python 3.10+
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        extdir = ext_fullpath.parent.resolve()

        # Using this requires trailing slash for auto-detection & inclusion of
        # auxiliary "native" libs

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"

        # CMake lets you override the generator - we need to check this.
        # Can be set with Conda-Build, for example.
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
        # EXAMPLE_VERSION_INFO shows you how to pass a value into the C++ code
        # from Python.
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",  # not used on MSVC, but no harm
        ]
        build_args = []
        # Adding CMake arguments set as environment variable
        # (needed e.g. to build for ARM OSx on conda-forge)
        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        # In this example, we pass in the version to C++. You might not need to.
        cmake_args += [f"-DEXAMPLE_VERSION_INFO={self.distribution.get_version()}"]

        # Using Ninja-build since it a) is available as a wheel and b)
        # multithreads automatically. MSVC would require all variables be
        # exported for Ninja to pick it up, which is a little tricky to do.
        # Users can override the generator with CMAKE_GENERATOR in CMake
        # 3.15+.
        if not cmake_generator or cmake_generator == "Ninja":
            import ninja

            ninja_executable_path = Path(ninja.BIN_DIR) / "ninja"
            cmake_args += [
                "-GNinja",
                f"-DCMAKE_MAKE_PROGRAM:FILEPATH={ninja_executable_path}",
            ]

        # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
        # across all generators.
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            # self.parallel is a Python 3 only way to set parallel jobs by hand
            # using -j in the build_ext call, not supported by pip or PyPA-build.
            parallel_num = ext.parallel
            if parallel_num <= 0:
                build_args += ["-j"]
            else:
                build_args += [f"-j{parallel_num}"]

        build_temp = Path(self.build_temp) / ext.name
        if not build_temp.exists():
            build_temp.mkdir(parents=True)
            
        print(ext.sourcedir, cmake_args, build_temp, build_args)

        subprocess.run(
            ["cmake", ext.sourcedir, *cmake_args], cwd=build_temp, check=True
        )
        subprocess.run(
            ["cmake", "--build", ".", *build_args], cwd=build_temp, check=True
        )
        
        # self.build_lib: build/lib.linux-aarch64-3.8
        subprocess.run(
            ["stubgen", "-p", ext.name,  
             "--include-docstrings", "--inspect-mode", 
             "--include-private",
             "-o", "./"], 
            cwd = self.build_lib, check=True)
    
    def make_init_py(self):
        init_codes = []
        package_folder = ""
        for ext in self.extensions:
            if isinstance(ext, CMakeExtension):
                ext_levels = ext.name.split(".")
                # 暂不支持多层so的cmake问题, 待有明确需求时会适配
                assert len(ext_levels) == 2 
                package_folder = ext_levels[0]
                if ext.import_all:                    
                    init_codes.append(f"from .{ext_levels[-1]} import *\n")
        
        pypath = os.path.join(self.build_lib, package_folder, "__init__.py")
        with open(pypath, "w") as f:
            f.writelines(init_codes)
        
        
        # 构建对应的pyi文件，这种pyi其实就是py的一个拷贝，
        shutil.copyfile(pypath, pypath + "i")
    
    def supplement_module_docstrings(self, ext_name, fileroot):
        split_names = ext_name.split(".")

        module_root = os.path.join(fileroot, *split_names)
        
        for filename in os.listdir(module_root):
            filepath = os.path.join(module_root, filename)
            print(f"Add docstrings to {filepath}")
            with open(filepath, "r") as f:
                file_context = f.read()
            if filename == "__init__.pyi":
                module = importlib.import_module(".".join(split_names))
            elif not os.path.isdir(filepath) and filepath.endswith(".pyi"):
                module_name = filename.split(".")[0]
                module = importlib.import_module(".".join(split_names + [module_name]))
            else:
                raise Exception(f"Not support to format {filepath}")

            file_context = f"\"\"\" {module.__name__} \n{module.__doc__}\n\"\"\"\n" + file_context
            with open(filepath, "w") as f:
                f.write(file_context)
        
    
    def build_extensions(self):
        super().build_extensions()
        # 后处理
        self.make_init_py() # 在包的根目录构造__init__.py
        
        # 给每个模块增加docstrings，stubgen没有增加模块的docstirngs
        libroot = os.path.join(os.getcwd(), self.build_lib)
        sys.path.append(libroot)
        for ext in self.extensions:
            self.supplement_module_docstrings(ext.name, libroot)
        sys.path.pop()
        
        
        
        


# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
    name="WiringPi",
    version="0.1.0",
    author="Zhaoxi.li",
    description="The python version of WiringPi, that is packaged by Pybind11",
    long_description="",
    ext_modules=[
        CMakeExtension("WiringPi.WiringPi", import_all=True, parallel=3)],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    python_requires=">=3.7",
    include_package_data=True
)
