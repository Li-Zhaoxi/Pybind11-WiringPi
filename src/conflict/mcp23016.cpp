
#include "supplements.h"

#include <mcp23016.h>
#include <mcp23016reg.h>

void supplement_mcp23016(py::module_ &subm)
{
  subm.def(
    "mcp23016Setup", 
    [](int pinBase, int i2cAddress) -> bool
    {
      return mcp23016Setup(pinBase, i2cAddress) > 0;
    }, 
    py::arg("pinBase"), py::arg("i2cAddress"), 
    R"pbdoc(
        Create a new instance of an MCP23016 I2C GPIO interface. We know it has 16 pins, so all we need to know here is the I2C address and the user-defined pin base.
    )pbdoc"
  );
  
  subm.attr("MCP23016_GP0") = MCP23016_GP0;
  subm.attr("MCP23016_GP1") = MCP23016_GP1;
  subm.attr("MCP23016_OLAT0") = MCP23016_OLAT0;
  subm.attr("MCP23016_OLAT1") = MCP23016_OLAT1;
  subm.attr("MCP23016_IPOL0") = MCP23016_IPOL0;
  subm.attr("MCP23016_IPOL1") = MCP23016_IPOL1;
  subm.attr("MCP23016_IODIR0") = MCP23016_IODIR0;
  subm.attr("MCP23016_IODIR1") = MCP23016_IODIR1;
  subm.attr("MCP23016_INTCAP0") = MCP23016_INTCAP0;
  subm.attr("MCP23016_INTCAP1") = MCP23016_INTCAP1;
  subm.attr("MCP23016_IOCON0") = MCP23016_IOCON0;
  subm.attr("MCP23016_IOCON1") = MCP23016_IOCON1;

  subm.attr("MCP23016_IOCON_IARES") = IOCON_IARES;
  subm.attr("MCP23016_IOCON_INIT") = IOCON_INIT;
}