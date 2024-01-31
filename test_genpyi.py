import os, sys
import cv2


module_name = "WiringPi"

from pybind11_stubgen import ModuleStubsGenerator

module = ModuleStubsGenerator(module_name)
module.parse()
module.write_setup_py = False

with open("%s.pyi" % module_name, "w") as fp:
    fp.write("#\n# Automatically generated file, do not edit!\n#\n\n")
    fp.write("\n".join(module.to_lines()))
