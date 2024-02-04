# Pybind11-WiringPi
The python version of WiringPi, that is packaged by Pybind11


## 1. Installation

由于我们的代码有一些第三方库的依赖，所以在使用前需要先将依赖的so文件进行编译

```
git clone --recursive https://github.com/Li-Zhaoxi/Pybind11-WiringPi
cd Pybind11-WiringPi
```

### 1.1 Prepare

- Build and install WiringPi-RDK: 
```
cd 3rdparty/WiringPi-RDK
./build
cd ..

```

### 1.2 Quick install  

这里补充whl的下载链接
sudo pip3 install package

### 1.3. Build wheel

- Install the dependent packages: `sudo pip3 install mypy ninja`
- Install Pybind11
```
cd 3rdparty/pybind11
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ..
sudo make install
```
- Build wheel. `python3 setup.py bdist_wheel`


A file named like "WiringPi.cpython-38-aarch64-linux-gnu.so" will be built in the folder `build`.

If you can `import WiringPi` correctly,  it means that you have successfully compiled this project.

## How to use

需要哪些函数请在后面的appendx中检索python用法。
目前已经支持注释自动弹出，

比如我们想使用pwm控制，在c++中是这样用的，在python中是这样用的。


## 2. Contribution

设计思想：C++各种驱动的py版本，因此不想构造太多文件层级。每个模块独立构造个so文件，该so文件会连接到C++版本，保证当同时加载时，能减少内存占用。

介绍如何贡献代码
- 如果想修改现有模块
- 如果想增加现有模块
- 修改之后最好做好文档检查，补充对应的pyi以及README

目前遗留的需要优化项：
1. 需要更多的测试
2. 需要规范文档
3. 怎么做单元测试
4. src结构，目前只有一个，




## Appendex

### A. Packaged APIs

- Module `WiringPi.adc` inclues:
  - `def ads1115Setup(pinBase: int, i2cAddress: int) -> bool` (Source: ads1115.h)
  - `def max31855Setup(pinBase: int, spiChannel: int) -> bool` (Source: max31855.h)
  - `def max5322Setup(pinBase: int, spiChannel: int) -> bool` (Source: max5322.h)
  - `def mcp3002Setup(pinBase: int, spiChannel: int) -> bool` (Source: mcp3002.h)
  - `def mcp3004Setup(pinBase: int, spiChannel: int) -> bool` (Source: mcp3004.h)
  - `def mcp3422Setup(pinBase: int, i2cAddress: int, sampleRate: int, gain: int) -> bool` (Source: mcp3422.h)
  - `def mcp4802Setup(pinBase: int, spiChannel: int) -> bool` (Source: mcp4802.h)
- Module `WiringPi.drc` includes:
  - `def drcSetupNet(pinBase: int, numPins: int, ipAddress: str, port: str, password: str) -> bool` (Source: drcNet.h)
  - `def drcSetupSerial(pinBase: int, numPins: int, device: str, baud: int) -> bool` (Source: drcSerial.h)
- Module `WiringPi.expansion` includes:
  - `def gertboardAnalogRead(chan: int) -> int` (Source: gertboard.h)
  - `def gertboardAnalogSetup(pinBase: int) -> int` (Source: gertboard.h)
  - `def gertboardAnalogWrite(chan: int, value: int) -> None` (Source: gertboard.h)
  - `def gertboardSPISetup() -> int` (Source: gertboard.h)
  - `def piFaceSetup(pinBase: int) -> int` (Source: piFace.h)
  - `def piGlow1(leg: int, ring: int, intensity: int) -> None` (Source: piGlow.h)
  - `def piGlowLeg(leg: int, intensity: int) -> None` (Source: piGlow.h)
  - `def piGlowRing(ring: int, intensity: int) -> None` (Source: piGlow.h)
  - `def piGlowSetup(clear: int) -> None` (Source: piGlow.h)
  - `def readNesJoystick(joystick: int) -> int` (Source: piNes.h)
  - `def scrollPhatClear() -> None` (Source: scrollPhat.h)
  - `def scrollPhatIntensity(percent: int) -> None` (Source: scrollPhat.h)
  - `def scrollPhatLine(x0: int, y0: int, x1: int, y1: int, colour: int) -> None` (Source: scrollPhat.h)
  - `def scrollPhatLineTo(x: int, y: int, colour: int) -> None` (Source: scrollPhat.h)
  - `def scrollPhatPoint(x: int, y: int, colour: int) -> None` (Source: scrollPhat.h)
  - `def scrollPhatPrintSpeed(cps10: int) -> None` (Source: scrollPhat.h)
  - `def scrollPhatPutchar(c: int) -> int` (Source: scrollPhat.h)
  - `def scrollPhatPuts(message: str) -> None` (Source: scrollPhat.h)
  - `def scrollPhatRectangle(x1: int, y1: int, x2: int, y2: int, colour: int, filled: int) -> None` (Source: scrollPhat.h)
  - `def scrollPhatSetup() -> int` (Source: scrollPhat.h)
  - `def scrollPhatUpdate() -> None` (Source: scrollPhat.h)
  - `def setupNesJoystick(dPin: int, cPin: int, lPin: int) -> int` (Source: piNes.h)
  - `MAX_NES_JOYSTICKS: int` (Source: piNes.h)
  - `NES_A: int` (Source: piNes.h)
  - `NES_B: int` (Source: piNes.h)
  - `NES_DOWN: int` (Source: piNes.h)
  - `NES_LEFT: int` (Source: piNes.h)
  - `NES_RIGHT: int` (Source: piNes.h)
  - `NES_SELECT: int` (Source: piNes.h)
  - `NES_START: int` (Source: piNes.h)
  - `NES_UP: int` (Source: piNes.h)
  - `PIGLOW_BLUE: int` (Source: piGlow.h)
  - `PIGLOW_GREEN: int` (Source: piGlow.h)
  - `PIGLOW_ORANGE: int` (Source: piGlow.h)
  - `PIGLOW_RED: int` (Source: piGlow.h)
  - `PIGLOW_WHITE: int` (Source: piGlow.h)
  - `PIGLOW_YELLOW: int` (Source: piGlow.h)
- Module `WiringPi.gpio` includes:
  - `def mcp23008Setup(pinBase: int, i2cAddress: int) -> bool` (Source: mcp23008.h)
  - `def mcp23016Setup(pinBase: int, i2cAddress: int) -> bool` (Source: mcp23016.h)
  - `def mcp23017Setup(pinBase: int, i2cAddress: int) -> bool` (Source: mcp23017.h)
  - `def mcp23s08Setup(pinBase: int, spiPort: int, devId: int) -> bool` (Source: mcp23s08.h)
  - `def mcp23s17Setup(pinBase: int, spiPort: int, devId: int) -> bool` (Source: mcp23s17.h)
  - `def pcf8574Setup(pinBase: int, i2cAddress: int) -> bool` (Source: pcf8574.h)
  - `def pcf8591Setup(pinBase: int, i2cAddress: int) -> bool` (Source: pcf8591.h)
  - `def sr595Setup(pinBase: int, numPins: int, dataPin: int, clockPin: int, latchPin: int) -> None` (Source: sr595.h)
  - `MCP23016_GP0: int` (Source: mcp23016reg.h)
  - `MCP23016_GP1: int` (Source: mcp23016reg.h)
  - `MCP23016_INTCAP0: int` (Source: mcp23016reg.h)
  - `MCP23016_INTCAP1: int` (Source: mcp23016reg.h)
  - `MCP23016_IOCON0: int` (Source: mcp23016reg.h)
  - `MCP23016_IOCON1: int` (Source: mcp23016reg.h)
  - `MCP23016_IOCON_IARES: int` (Source: mcp23016reg.h)
  - `MCP23016_IOCON_INIT: int` (Source: mcp23016reg.h)
  - `MCP23016_IODIR0: int` (Source: mcp23016reg.h)
  - `MCP23016_IODIR1: int` (Source: mcp23016reg.h)
  - `MCP23016_IPOL0: int` (Source: mcp23016reg.h)
  - `MCP23016_IPOL1: int` (Source: mcp23016reg.h)
  - `MCP23016_OLAT0: int` (Source: mcp23016reg.h)
  - `MCP23016_OLAT1: int` (Source: mcp23016reg.h)
  - `MCP23x0817_CMD_READ: int` (Source: mcp23x0817.h)
  - `MCP23x0817_CMD_WRITE: int` (Source: mcp23x0817.h)
  - `MCP23x0817_IOCON_BANK_MODE: int` (Source: mcp23x0817.h)
  - `MCP23x0817_IOCON_DISSLW: int` (Source: mcp23x0817.h)
  - `MCP23x0817_IOCON_HAEN: int` (Source: mcp23x0817.h)
  - `MCP23x0817_IOCON_INIT: int` (Source: mcp23x0817.h)
  - `MCP23x0817_IOCON_INTPOL: int` (Source: mcp23x0817.h)
  - `MCP23x0817_IOCON_MIRROR: int` (Source: mcp23x0817.h)
  - `MCP23x0817_IOCON_ODR: int` (Source: mcp23x0817.h)
  - `MCP23x0817_IOCON_SEQOP: int` (Source: mcp23x0817.h)
  - `MCP23x0817_IOCON_UNUSED: int` (Source: mcp23x0817.h)
  - `MCP23x08_DEFVAL: int`
  - `MCP23x08_GPINTEN: int`
  - `MCP23x08_GPIO: int`
  - `MCP23x08_GPPU: int`
  - `MCP23x08_INTCAP: int`
  - `MCP23x08_INTCON: int`
  - `MCP23x08_INTF: int`
  - `MCP23x08_IOCON: int`
  - `MCP23x08_IODIR: int`
  - `MCP23x08_IPOL: int`
  - `MCP23x08_OLAT: int`
  - `MCP23x17_CMD_READ: int`
  - `MCP23x17_CMD_WRITE: int`
  - `MCP23x17_DEFVALA: int`
  - `MCP23x17_DEFVALB: int`
  - `MCP23x17_GPINTENA: int`
  - `MCP23x17_GPINTENB: int`
  - `MCP23x17_GPIOA: int`
  - `MCP23x17_GPIOB: int`
  - `MCP23x17_GPPUA: int`
  - `MCP23x17_GPPUB: int`
  - `MCP23x17_INTCAPA: int`
  - `MCP23x17_INTCAPB: int`
  - `MCP23x17_INTCONA: int`
  - `MCP23x17_INTCONB: int`
  - `MCP23x17_INTFA: int`
  - `MCP23x17_INTFB: int`
  - `MCP23x17_IOCON: int`
  - `MCP23x17_IOCONB: int`
  - `MCP23x17_IOCON_BANK_MODE: int`
  - `MCP23x17_IOCON_DISSLW: int`
  - `MCP23x17_IOCON_HAEN: int`
  - `MCP23x17_IOCON_INIT: int`
  - `MCP23x17_IOCON_INTPOL: int`
  - `MCP23x17_IOCON_MIRROR: int`
  - `MCP23x17_IOCON_ODR: int`
  - `MCP23x17_IOCON_SEQOP: int`
  - `MCP23x17_IOCON_UNUSED: int`
  - `MCP23x17_IODIRA: int`
  - `MCP23x17_IODIRB: int`
  - `MCP23x17_IPOLA: int`
  - `MCP23x17_IPOLB: int`
  - `MCP23x17_OLATA: int`
  - `MCP23x17_OLATB: int`
- Module `WiringPi.lcd` includes:
  - `def lcd128x64circle(x: int, y: int, r: int, colour: int, filled: int) -> None`
  - `def lcd128x64ellipse(cx: int, cy: int, xRadius: int, yRadius: int, colour: int, filled: int) -> None`
  - `def lcd128x64getScreenSize() -> tuple[int, int]`
  - `def lcd128x64line(x0: int, y0: int, x1: int, y1: int, colour: int) -> None`
  - `def lcd128x64lineTo(x: int, y: int, colour: int) -> None`
  - `def lcd128x64orientCoordinates(x: int, y: int) -> tuple[int, int]`
  - `def lcd128x64point(x: int, y: int, colour: int) -> None`
  - `def lcd128x64putchar(x: int, y: int, c: int, bgCol: int, fgCol: int) -> None`
  - `def lcd128x64puts(x: int, y: int, text: str, bgCol: int, fgCol: int) -> None`
  - `def lcd128x64rectangle(x1: int, y1: int, x2: int, y2: int, colour: int, filled: int) -> None`
  - `def lcd128x64setOrientation(orientation: int) -> None`
  - `def lcd128x64setOrigin(x: int, y: int) -> None`
  - `def lcd128x64setup() -> int`
  - `def lcd128x64update() -> None`
  - `def lcdCharDef(fd: int, index: int, data: numpy.ndarray[numpy.uint8]) -> None`
  - `def lcdClear(fd: int) -> None`
  - `def lcdCursor(fd: int, state: int) -> None`
  - `def lcdCursorBlink(fd: int, state: int) -> None`
  - `def lcdDisplay(fd: int, state: int) -> None`
  - `def lcdHome(fd: int) -> None`
  - `def lcdInit(rows: int, cols: int, bits: int, rs: int, strb: int, d0: int, d1: int, d2: int, d3: int, d4: int, d5: int, d6: int, d7: int) -> int`
  - `def lcdPosition(fd: int, x: int, y: int) -> None`
  - `def lcdPutchar(fd: int, data: int) -> None`
  - `def lcdPuts(fd: int, string: str) -> None`
  - `def lcdSendCommand(fd: int, command: int) -> None`