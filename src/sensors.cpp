#include "submodules.h"
#include <optional>

#include <bmp180.h>
#include <ds18b20.h>
#include <htu21d.h>
#include <maxdetect.h>
#include <rht03.h>

void supplement_bmp180(py::module_ &subm);
void supplement_ds18b20(py::module_ &subm);
void supplement_htu21d(py::module_ &subm);
void supplement_maxdetect(py::module_ &subm);
void supplement_rht03(py::module_ &subm);

void add_submodule_sensors(py::module_ &m)
{
  auto subm = m.def_submodule(
    "sensors", 
    R"pbdoc(
        Extend wiringPi with the
          BMP180 I2C Pressure and Temperature sensor. 
          DS18B20 1-Wire temperature sensor. 
          HTU21D I2C humidity and Temperature sensor.
          [MaxDetect] Driver for the MaxDetect series sensors.
          rht03 Maxdetect 1-Wire sensor.
        Copyright (c) 2016 Gordon Henderson
    )pbdoc"
  );

  supplement_bmp180(subm);
  supplement_ds18b20(subm);
  supplement_htu21d(subm);
  supplement_maxdetect(subm);
  supplement_rht03(subm);
}


void supplement_bmp180(py::module_ &subm)
{
  subm.def(
    "bmp180Setup", 
    [](int pinBase) -> bool
    {
      return bmp180Setup(pinBase) > 0;
    }, 
    py::arg("pinBase"),
    R"pbdoc(
        Create a new instance of a PCF8591 I2C GPIO interface. We know it
        has 4 pins, (4 analog inputs and 1 analog output which we'll shadow
        input 0) so all we need to know here is the I2C address and the
        user-defined pin base.
    )pbdoc"
  );
} 

void supplement_ds18b20(py::module_ &subm)
{
  subm.def(
    "ds18b20Setup", 
    [](int pinBase, const char *deviceId) -> bool
    {
      return ds18b20Setup(pinBase, deviceId) > 0;
    }, 
    py::arg("pinBase"), 
    py::arg("deviceId"), 
    R"pbdoc(
        Create a new instance of a DS18B20 temperature sensor.
    )pbdoc"
  );
}

void supplement_htu21d(py::module_ &subm)
{
  subm.def(
    "htu21dSetup", 
    [](int pinBase) -> bool
    {
      return htu21dSetup(pinBase) > 0;
    }, 
    py::arg("pinBase"),
    R"pbdoc(
        Create a new instance of a HTU21D I2C GPIO interface.
        This chip has a fixed I2C address, so we are not providing any allowance to change this.
    )pbdoc"
  );
}


void supplement_maxdetect(py::module_ &subm)
{
  subm.def(
    "maxDetectRead", 
    [](int pin) -> std::optional<py::array_t<unsigned char>>
    {
      unsigned char buffer[4];
      int state = maxDetectRead(pin, buffer);
      if(state)
        return ptr_to_array1d(buffer, 4);
      else
        return std::nullopt;

    }, 
    py::arg("pin"),
    R"pbdoc(
        Read in and return the 4 data bytes from the MaxDetect sensor. \n
        Return uchar[4] if passing the checksum validity
        else return None
    )pbdoc"
  );

  subm.def(
    "readRHT03", 
    [](int pin) -> std::optional<std::tuple<int, int>>
    {
      int temp, rh;
      int state = readRHT03(pin, &temp, &rh);
      if (state)
        return std::make_tuple(temp, rh);
      else
        return std::nullopt;
    }, 
    py::arg("pin"),
    R"pbdoc(
        Read the Temperature & Humidity from an RHT03 sensor
        Values returned are *10, so 123 is 12.3.
    )pbdoc"
  );
}
 
void supplement_rht03(py::module_ &subm)
{
  subm.def(
    "rht03Setup", 
    [](int pinBase, int piPin) -> bool
    {
      return rht03Setup(pinBase, piPin) > 0;
    }, 
    py::arg("pinBase"),
    py::arg("devicePin"),
    R"pbdoc(
        Create a new instance of an RHT03 temperature sensor.
    )pbdoc"
  );
}