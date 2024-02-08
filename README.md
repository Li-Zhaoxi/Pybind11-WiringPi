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
  - `MCP23x08_DEFVAL: int` (Source: mcp23x0817.h)
  - `MCP23x08_GPINTEN: int` (Source: mcp23x0817.h)
  - `MCP23x08_GPIO: int` (Source: mcp23x0817.h)
  - `MCP23x08_GPPU: int` (Source: mcp23x0817.h)
  - `MCP23x08_INTCAP: int` (Source: mcp23x0817.h)
  - `MCP23x08_INTCON: int` (Source: mcp23x0817.h)
  - `MCP23x08_INTF: int` (Source: mcp23x0817.h)
  - `MCP23x08_IOCON: int` (Source: mcp23x0817.h)
  - `MCP23x08_IODIR: int` (Source: mcp23x0817.h)
  - `MCP23x08_IPOL: int` (Source: mcp23x0817.h)
  - `MCP23x08_OLAT: int` (Source: mcp23x0817.h)
  - `MCP23x17_CMD_READ: int` (Source: mcp23x0817.h)
  - `MCP23x17_CMD_WRITE: int` (Source: mcp23x0817.h)
  - `MCP23x17_DEFVALA: int` (Source: mcp23x0817.h)
  - `MCP23x17_DEFVALB: int` (Source: mcp23x0817.h)
  - `MCP23x17_GPINTENA: int` (Source: mcp23x0817.h)
  - `MCP23x17_GPINTENB: int` (Source: mcp23x0817.h)
  - `MCP23x17_GPIOA: int` (Source: mcp23x0817.h)
  - `MCP23x17_GPIOB: int` (Source: mcp23x0817.h)
  - `MCP23x17_GPPUA: int` (Source: mcp23x0817.h)
  - `MCP23x17_GPPUB: int` (Source: mcp23x0817.h)
  - `MCP23x17_INTCAPA: int` (Source: mcp23x0817.h)
  - `MCP23x17_INTCAPB: int` (Source: mcp23x0817.h)
  - `MCP23x17_INTCONA: int` (Source: mcp23x0817.h)
  - `MCP23x17_INTCONB: int` (Source: mcp23x0817.h)
  - `MCP23x17_INTFA: int` (Source: mcp23x0817.h)
  - `MCP23x17_INTFB: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCONB: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON_BANK_MODE: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON_DISSLW: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON_HAEN: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON_INIT: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON_INTPOL: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON_MIRROR: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON_ODR: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON_SEQOP: int` (Source: mcp23x0817.h)
  - `MCP23x17_IOCON_UNUSED: int` (Source: mcp23x0817.h)
  - `MCP23x17_IODIRA: int` (Source: mcp23x0817.h)
  - `MCP23x17_IODIRB: int` (Source: mcp23x0817.h)
  - `MCP23x17_IPOLA: int` (Source: mcp23x0817.h)
  - `MCP23x17_IPOLB: int` (Source: mcp23x0817.h)
  - `MCP23x17_OLATA: int` (Source: mcp23x0817.h)
  - `MCP23x17_OLATB: int` (Source: mcp23x0817.h)
- Module `WiringPi.lcd` includes:
  - `def lcd128x64circle(x: int, y: int, r: int, colour: int, filled: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64ellipse(cx: int, cy: int, xRadius: int, yRadius: int, colour: int, filled: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64getScreenSize() -> tuple[int, int]` (Source: lcd128x64.h)
  - `def lcd128x64line(x0: int, y0: int, x1: int, y1: int, colour: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64lineTo(x: int, y: int, colour: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64orientCoordinates(x: int, y: int) -> tuple[int, int]` (Source: lcd128x64.h)
  - `def lcd128x64point(x: int, y: int, colour: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64putchar(x: int, y: int, c: int, bgCol: int, fgCol: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64puts(x: int, y: int, text: str, bgCol: int, fgCol: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64rectangle(x1: int, y1: int, x2: int, y2: int, colour: int, filled: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64setOrientation(orientation: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64setOrigin(x: int, y: int) -> None` (Source: lcd128x64.h)
  - `def lcd128x64setup() -> int` (Source: lcd128x64.h)
  - `def lcd128x64update() -> None` (Source: lcd128x64.h)
  - `def lcdCharDef(fd: int, index: int, data: numpy.ndarray[numpy.uint8]) -> None` (Source: lcd.h)
  - `def lcdClear(fd: int) -> None` (Source: lcd.h)
  - `def lcdCursor(fd: int, state: int) -> None` (Source: lcd.h)
  - `def lcdCursorBlink(fd: int, state: int) -> None` (Source: lcd.h)
  - `def lcdDisplay(fd: int, state: int) -> None` (Source: lcd.h)
  - `def lcdHome(fd: int) -> None` (Source: lcd.h)
  - `def lcdInit(rows: int, cols: int, bits: int, rs: int, strb: int, d0: int, d1: int, d2: int, d3: int, d4: int, d5: int, d6: int, d7: int) -> int` (Source: lcd.h)
  - `def lcdPosition(fd: int, x: int, y: int) -> None` (Source: lcd.h)
  - `def lcdPutchar(fd: int, data: int) -> None` (Source: lcd.h)
  - `def lcdPuts(fd: int, string: str) -> None` (Source: lcd.h)
  - `def lcdSendCommand(fd: int, command: int) -> None` (Source: lcd.h)
- Module `WiringPi.led` includes:
  - `def sn3218Setup(pinBase: int) -> bool` (Source: sn3218.h)
- Module `WiringPi.rtc` includes:
  - `def ds1302clockRead() -> numpy.ndarray[numpy.int32]`
  - `def ds1302clockWrite(clockData: numpy.ndarray[numpy.int32]) -> None`
  - `def ds1302ramRead(addr: int) -> int`
  - `def ds1302ramWrite(addr: int, data: int) -> None`
  - `def ds1302rtcRead(reg: int) -> int`
  - `def ds1302rtcWrite(reg: int, data: int) -> None`
  - `def ds1302setup(clockPin: int, dataPin: int, csPin: int) -> None`
  - `def ds1302trickleCharge(diodes: int, resistors: int) -> None`
- Module `WiringPi.sensors` includes:
  - `def bmp180Setup(pinBase: int) -> bool`
  - `def ds18b20Setup(pinBase: int, deviceId: str) -> bool`
  - `def htu21dSetup(pinBase: int) -> bool`
  - `def maxDetectRead(pin: int) -> std::optional<array_t<unsigned char, 16> >`
  - `def readRHT03(pin: int) -> std::variant<std::tuple<int, int>, none>`
  - `def rht03Setup(pinBase: int, devicePin: int) -> bool`
- Module `WiringPi.softdriven` includes:
  - `def pseudoPinsSetup(pinBase: int) -> bool`
  - `def softPwmCreate(pin: int, value: int, range: int) -> int`
  - `def softPwmStop(pin: int) -> None`
  - `def softPwmWrite(pin: int, value: int) -> None`
  - `def softServoSetup(p0: int, p1: int, p2: int, p3: int, p4: int, p5: int, p6: int, p7: int) -> int`
  - `def softServoWrite(pin: int, value: int) -> None`
  - `def softToneCreate(pin: int) -> int`
  - `def softToneStop(pin: int) -> None`
  - `def softToneWrite(pin: int, freq: int) -> None`
- Module `WiringPi.softdriven` includes:
  - `def analogRead(pin: int) -> int`
  - `def analogWrite(pin: int, value: int) -> None`
  - `def delay(howLong: int) -> None`
  - `def delayMicroseconds(howLong: int) -> None`
  - `def digitalRead(pin: int) -> int`
  - `def digitalReadByte() -> int`
  - `def digitalReadByte2() -> int`
  - `def digitalWrite(pin: int, value: int) -> None`
  - `def digitalWriteByte(value: int) -> None`
  - `def digitalWriteByte2(value: int) -> None`
  - `def getAlt(pin: int) -> int`
  - `def gpioClockSet(pin: int, freq: int) -> None`
  - `def loadWPiExtension(progName: str, extensionData: str, verbose: int) -> bool`
  - `def micros() -> int`
  - `def millis() -> int`
  - `def physPinToGpio(physPin: int) -> int`
  - `def piBoardId() -> tuple[int, int, int, int, int]`
  - `def piGpioLayout() -> int`
  - `def piHiPri(pri: int) -> int`
  - `def piLock(key: int) -> None`
  - `piThreadCreate(fn: std::function<void* (void*)>) -> int`
  - `def piUnlock(key: int) -> None`
  - `def pinMode(pin: int, mode: int) -> None`
  - `def pinModeAlt(pin: int, mode: int) -> None`
  - `def pullUpDnControl(pin: int, pud: int) -> None`
  - `def pwmSetClock(divisor: int) -> None`
  - `def pwmSetDuty(pin: int, duty_cycle_ns: int) -> None`
  - `def pwmSetFreq(pin: int, frequency: int) -> None`
  - `def pwmSetMode(mode: int) -> None`
  - `def pwmSetRange(range: int) -> None`
  - `def pwmToneWrite(pin: int, freq: int) -> None`
  - `def pwmWrite(pin: int, value: int) -> None`
  - `def serialClose(fd: int) -> None`
  - `def serialDataAvail(fd: int) -> int`
  - `def serialFlush(fd: int) -> None`
  - `def serialGetchar(fd: int) -> int`
  - `def serialOpen(device: str, baud: int) -> int`
  - `def serialPutchar(fd: int, c: int) -> None`
  - `def serialPuts(fd: int, message: str) -> None`
  - `def setPadDrive(group: int, value: int) -> None`
  - `def shiftIn(dPin: int, cPin: int, order: int) -> int`
  - `def shiftOut(dPin: int, cPin: int, order: int, val: int) -> None`
  - `def waitForInterrupt(pin: int, mS: int) -> int`
  - `def wiringPiFailure(fatal: int, message: str) -> int`
  - `def wiringPiI2CRead(fd: int) -> int`
  - `def wiringPiI2CReadReg16(fd: int, reg: int) -> int`
  - `def wiringPiI2CReadReg8(fd: int, reg: int) -> int`
  - `def wiringPiI2CSetup(devId: int) -> int`
  - `def wiringPiI2CSetupInterface(device: str, devId: int) -> int`
  - `def wiringPiI2CWrite(fd: int, data: int) -> int`
  - `def wiringPiI2CWriteReg16(fd: int, reg: int, data: int) -> int`
  - `def wiringPiI2CWriteReg8(fd: int, reg: int, data: int) -> int`
  - `def wiringPiISR(pin: int, mode: int, function) -> int`
  - `def wiringPiSPIDataRW(channel: int, data: int, len: int) -> int`
  - `def wiringPiSPIGetFd(channel: int) -> int`
  - `def wiringPiSPISetup(channel: int, speed: int) -> int`
  - `def wiringPiSPISetupMode(channel: int, speed: int, mode: int) -> int`
  - `def wiringPiSetup() -> int`
  - `def wiringPiSetupGpio() -> int`
  - `def wiringPiSetupPhys() -> int`
  - `def wiringPiSetupSys() -> int`
  - `def wiringPiVersion() -> tuple[int, int]`
  - `def wpiPinToGpio(wpiPin: int) -> int`
  - `GPIO_CLOCK: int`
  - `HIGH: int`
  - `INPUT: int`
  - `INT_EDGE_BOTH: int`
  - `INT_EDGE_FALLING: int`
  - `INT_EDGE_RISING: int`
  - `INT_EDGE_SETUP: int`
  - `LOW: int`
  - `NOTD: int`
  - `OUTPUT: int`
  - `PI_ALPHA: int`
  - `PI_GPIO_MASK: int`
  - `PI_MAKER_EGOMAN: int`
  - `PI_MAKER_EMBEST: int`
  - `PI_MAKER_HORIZON: int`
  - `PI_MAKER_SONY: int`
  - `PI_MAKER_UNKNOWN: int`
  - `PI_MODEL_07: int`
  - `PI_MODEL_2: int`
  - `PI_MODEL_3AP: int`
  - `PI_MODEL_3B: int`
  - `PI_MODEL_3BP: int`
  - `PI_MODEL_400: int`
  - `PI_MODEL_4B: int`
  - `PI_MODEL_A: int`
  - `PI_MODEL_AP: int`
  - `PI_MODEL_B: int`
  - `PI_MODEL_BP: int`
  - `PI_MODEL_CM: int`
  - `PI_MODEL_CM3: int`
  - `PI_MODEL_CM3P: int`
  - `PI_MODEL_CM4: int`
  - `PI_MODEL_RDKX3: int`
  - `PI_MODEL_RDKX3MD: int`
  - `PI_MODEL_RDKX3V1_2: int`
  - `PI_MODEL_RDKX3V2: int`
  - `PI_MODEL_SDB: int`
  - `PI_MODEL_ZERO: int`
  - `PI_MODEL_ZERO_2W: int`
  - `PI_MODEL_ZERO_W: int`
  - `PI_VERSION_1: int`
  - `PI_VERSION_1_1: int`
  - `PI_VERSION_1_2: int`
  - `PI_VERSION_2: int`
  - `PI_VERSION_3: int`
  - `PI_VERSION_4: int`
  - `PUD_DOWN: int`
  - `PUD_OFF: int`
  - `PUD_UP: int`
  - `PWM_MODE_BAL: int`
  - `PWM_MODE_MS: int`
  - `PWM_OUTPUT: int`
  - `PWM_TONE_OUTPUT: int`
  - `SOFT_PWM_OUTPUT: int`
  - `SOFT_TONE_OUTPUT: int`
  - `WPI_MODE_GPIO: int`
  - `WPI_MODE_GPIO_SYS: int`
  - `WPI_MODE_PHYS: int`
  - `WPI_MODE_PIFACE: int`
  - `WPI_MODE_PINS: int`
  - `WPI_MODE_UNINITIALISED: int`






