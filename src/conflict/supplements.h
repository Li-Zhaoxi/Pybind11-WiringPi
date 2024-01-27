#ifndef SUPPLEMENTS_H_
#define SUPPLEMENTS_H_

#include <pybind11/pybind11.h>
namespace py = pybind11;

#define ADD_RENAME_MODULE_ATTR(module, prefix, name) \
  module.attr(prefix"_"#name) = name

void supplement_mcp23x0817(py::module_ &subm);
void supplement_mcp23016(py::module_ &subm);
void supplement_mcp23x17reg(py::module_ &subm);


#endif