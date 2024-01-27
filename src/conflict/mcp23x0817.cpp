#include "supplements.h"
#include <mcp23x0817.h>

void supplement_mcp23x0817(py::module_ &subm)
{
  // MCP23x08 Registers
  subm.attr("MCP23x08_IODIR") = MCP23x08_IODIR;
  subm.attr("MCP23x08_IPOL") = MCP23x08_IPOL;
  subm.attr("MCP23x08_GPINTEN") = MCP23x08_GPINTEN;
  subm.attr("MCP23x08_DEFVAL") = MCP23x08_DEFVAL;
  subm.attr("MCP23x08_INTCON") = MCP23x08_INTCON;
  subm.attr("MCP23x08_IOCON") = MCP23x08_IOCON;
  subm.attr("MCP23x08_GPPU") = MCP23x08_GPPU;
  subm.attr("MCP23x08_INTF") = MCP23x08_INTF;
  subm.attr("MCP23x08_INTCAP") = MCP23x08_INTCAP;
  subm.attr("MCP23x08_GPIO") = MCP23x08_GPIO;
  subm.attr("MCP23x08_OLAT") = MCP23x08_OLAT;

  // MCP23x17 Registers
  subm.attr("MCP23x17_IODIRA") = MCP23x17_IODIRA;
  subm.attr("MCP23x17_IPOLA") = MCP23x17_IPOLA;
  subm.attr("MCP23x17_GPINTENA") = MCP23x17_GPINTENA;
  subm.attr("MCP23x17_DEFVALA") = MCP23x17_DEFVALA;
  subm.attr("MCP23x17_INTCONA") = MCP23x17_INTCONA;
  subm.attr("MCP23x17_IOCON") = MCP23x17_IOCON;
  subm.attr("MCP23x17_GPPUA") = MCP23x17_GPPUA;
  subm.attr("MCP23x17_INTFA") = MCP23x17_INTFA;
  subm.attr("MCP23x17_INTCAPA") = MCP23x17_INTCAPA;
  subm.attr("MCP23x17_GPIOA") = MCP23x17_GPIOA;
  subm.attr("MCP23x17_OLATA") = MCP23x17_OLATA;

  subm.attr("MCP23x17_IODIRB") = MCP23x17_IODIRB;
  subm.attr("MCP23x17_IPOLB") = MCP23x17_IPOLB;
  subm.attr("MCP23x17_GPINTENB") = MCP23x17_GPINTENB;
  subm.attr("MCP23x17_DEFVALB") = MCP23x17_DEFVALB;
  subm.attr("MCP23x17_INTCONB") = MCP23x17_INTCONB;
  subm.attr("MCP23x17_IOCONB") = MCP23x17_IOCONB;
  subm.attr("MCP23x17_GPPUB") = MCP23x17_GPPUB;
  subm.attr("MCP23x17_INTFB") = MCP23x17_INTFB;
  subm.attr("MCP23x17_INTCAPB") = MCP23x17_INTCAPB;
  subm.attr("MCP23x17_GPIOB") = MCP23x17_GPIOB;
  subm.attr("MCP23x17_OLATB") = MCP23x17_OLATB;

  // Bits in the IOCON register
  subm.attr("MCP23x0817_IOCON_UNUSED") = IOCON_UNUSED;
  subm.attr("MCP23x0817_IOCON_INTPOL") = IOCON_INTPOL;
  subm.attr("MCP23x0817_IOCON_ODR") = IOCON_ODR;
  subm.attr("MCP23x0817_IOCON_HAEN") = IOCON_HAEN;
  subm.attr("MCP23x0817_IOCON_DISSLW") = IOCON_DISSLW;
  subm.attr("MCP23x0817_IOCON_SEQOP") = IOCON_SEQOP;
  subm.attr("MCP23x0817_IOCON_MIRROR") = IOCON_MIRROR;
  subm.attr("MCP23x0817_IOCON_BANK_MODE") = IOCON_BANK_MODE;

  // Default initialisation mode
  subm.attr("MCP23x0817_IOCON_INIT") = IOCON_INIT;

  // SPI Command codes
  subm.attr("MCP23x0817_CMD_WRITE") = CMD_WRITE;
  subm.attr("MCP23x0817_CMD_READ") = CMD_READ;
}