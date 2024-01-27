#ifndef SUB_MODULES_H_
#define SUB_MODULES_H_
#include <fstream>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
namespace py = pybind11;

#define ADD_MODULE_ATTR(module, name) \
  module.attr(#name) = name

// 子模块的合理性就是注释，可以对每个模块有个描述

// 数组转矩阵的封装参考：https://blog.csdn.net/tcy23456/article/details/119998548
template<typename Type>
py::array_t<Type> ptr_to_array1d(const Type* data, py::ssize_t col) 
{
  py::array_t<Type> out = py::array_t<Type>(col);
  auto r3 = out.template mutable_unchecked<1>();

  for (int i = 0; i < col; i++)
    r3(i) = data[i];

  return out;
}

template<typename Type>
void array1d_to_ptr(const py::array_t<Type> &pydata, Type* data, int length) 
{
  const auto r = pydata.template unchecked<1>();
  if (r.ndim() != 1 || r.shape(0) != length)
  {
    std::stringstream ss;
    ss << "The input data must be 1d array with << ";
    ss << length << " elements, but ";
    ss << "dim: " << r.ndim() << ", ";
    ss << "shape: " << r.shape(0);
    py::set_error(PyExc_ValueError, ss.str().c_str());
  }
  for(int i = 0; i < length; i++)
    data[i] = r(i);
}

// ADC:
//  ads1115.h
//  max5322.h
//  max31855.h
//  mcp3002.h
//  mcp3004.h
//  mcp3422.h
//  mcp4802.h
void add_submodule_adc(py::module_ &m);

// SENSOR:
//  bmp180.h
//  ds18b20.h
//  htu21d.h
//  maxdetect.h
//  rht03.h
void add_submodule_sensors(py::module_ &m);

// DRC
//  drcNet.h
//  drcSerial.h
void add_submodule_drc(py::module_ &m);

// RTC:
//  ds1302.h
void add_submodule_rtc(py::module_ &m);

// expansion:
//  gertboard.h
//  scrollPhat.h
//  piNes, piFace, piGlow
void add_submodule_expansion(py::module_ &m);

// LCD:
//  lcd.h 
//  lcd128x64.h 
void add_submodule_lcd(py::module_ &m);

// GPIO
//  mcp23s08.h
//  mcp23x0817.h
//  pcf8574
//  pcf8591
void add_submodule_gpio(py::module_ &m);


// software driven
void add_submodule_softdriven(py::module_ &m);


// LED
void add_submodule_led(py::module_ &m);


void add_submodule_wiring(py::module_ &m);



#endif 
