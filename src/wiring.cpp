#include "submodules.h"

#include <functional>

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>
#include <wiringShift.h>
#include <wpiExtensions.h>

void supplement_wiringpi(py::module_ &subm);
void supplement_wiringSerial(py::module_ &subm);
void supplement_wiringpii2c(py::module_ &subm);
void supplement_wiringpispi(py::module_ &subm);
void supplement_wiringpishift(py::module_ &subm);
void supplement_wiringpiext(py::module_ &subm);


void add_submodule_wiring(py::module_ &m)
{
  auto subm = m.def_submodule(
    "wiring", 
    R"pbdoc(
      wiringPi: Arduino look-a-like Wiring library for the Raspberry Pi
      wiringSerial: Handle a serial port
      wiringPiI2C: Simplified I2C access routines
      wiringPiSPI: Simplified SPI access routines

      Copyright (c) 2013 Gordon Henderson
    )pbdoc"
  );

  supplement_wiringpi(subm);
  supplement_wiringSerial(subm);
  supplement_wiringpii2c(subm);
  supplement_wiringpispi(subm);
  supplement_wiringpishift(subm);
  supplement_wiringpiext(subm);
  
}

void supplement_wiringpi(py::module_ &subm)
{
  // wiringPi
  subm.attr("PI_GPIO_MASK") = PI_GPIO_MASK;

  subm.attr("WPI_MODE_PINS") = WPI_MODE_PINS;
  subm.attr("WPI_MODE_GPIO") = WPI_MODE_GPIO;
  subm.attr("WPI_MODE_GPIO_SYS") = WPI_MODE_GPIO_SYS;
  subm.attr("WPI_MODE_PHYS") = WPI_MODE_PHYS;
  subm.attr("WPI_MODE_PIFACE") = WPI_MODE_PIFACE;
  subm.attr("WPI_MODE_UNINITIALISED") = WPI_MODE_UNINITIALISED;

  subm.attr("INPUT") = INPUT;
  subm.attr("OUTPUT") = OUTPUT;
  subm.attr("PWM_OUTPUT") = PWM_OUTPUT;
  subm.attr("GPIO_CLOCK") = GPIO_CLOCK;
  subm.attr("SOFT_PWM_OUTPUT") = SOFT_PWM_OUTPUT;
  subm.attr("SOFT_TONE_OUTPUT") = SOFT_TONE_OUTPUT;
  subm.attr("PWM_TONE_OUTPUT") = PWM_TONE_OUTPUT;

  subm.attr("LOW") = LOW;
  subm.attr("HIGH") = HIGH;
  subm.attr("NOTD") = NOTD;

  subm.attr("PUD_OFF") = PUD_OFF;
  subm.attr("PUD_DOWN") = PUD_DOWN;
  subm.attr("PUD_UP") = PUD_UP;

  subm.attr("PWM_MODE_MS") = PWM_MODE_MS;
  subm.attr("PWM_MODE_BAL") = PWM_MODE_BAL;

  ADD_MODULE_ATTR(subm, INT_EDGE_SETUP);
  ADD_MODULE_ATTR(subm, INT_EDGE_FALLING);
  ADD_MODULE_ATTR(subm, INT_EDGE_RISING);
  ADD_MODULE_ATTR(subm, INT_EDGE_BOTH);

  ADD_MODULE_ATTR(subm, PI_MODEL_A);
  ADD_MODULE_ATTR(subm, PI_MODEL_B);
  ADD_MODULE_ATTR(subm, PI_MODEL_AP);
  ADD_MODULE_ATTR(subm, PI_MODEL_BP);
  ADD_MODULE_ATTR(subm, PI_MODEL_2);
  ADD_MODULE_ATTR(subm, PI_ALPHA);
  ADD_MODULE_ATTR(subm, PI_MODEL_CM);
  ADD_MODULE_ATTR(subm, PI_MODEL_07);
  ADD_MODULE_ATTR(subm, PI_MODEL_3B);
  ADD_MODULE_ATTR(subm, PI_MODEL_ZERO);
  ADD_MODULE_ATTR(subm, PI_MODEL_CM3);
  ADD_MODULE_ATTR(subm, PI_MODEL_ZERO_W);
  ADD_MODULE_ATTR(subm, PI_MODEL_3BP);
  ADD_MODULE_ATTR(subm, PI_MODEL_3AP);
  ADD_MODULE_ATTR(subm, PI_MODEL_CM3P);
  ADD_MODULE_ATTR(subm, PI_MODEL_4B);
  ADD_MODULE_ATTR(subm, PI_MODEL_ZERO_2W);
  ADD_MODULE_ATTR(subm, PI_MODEL_400);
  ADD_MODULE_ATTR(subm, PI_MODEL_CM4);
  ADD_MODULE_ATTR(subm, PI_MODEL_SDB);
  // ADD_MODULE_ATTR(subm, PI_MODEL_X3PI);
  ADD_MODULE_ATTR(subm, PI_MODEL_RDKX3);
  ADD_MODULE_ATTR(subm, PI_MODEL_RDKX3V1_2);
  ADD_MODULE_ATTR(subm, PI_MODEL_RDKX3V2);
  ADD_MODULE_ATTR(subm, PI_MODEL_RDKX3MD);

  ADD_MODULE_ATTR(subm, PI_VERSION_1);
  ADD_MODULE_ATTR(subm, PI_VERSION_1_1);
  ADD_MODULE_ATTR(subm, PI_VERSION_1_2);
  ADD_MODULE_ATTR(subm, PI_VERSION_2);
  ADD_MODULE_ATTR(subm, PI_VERSION_3);
  ADD_MODULE_ATTR(subm, PI_VERSION_4);

  ADD_MODULE_ATTR(subm, PI_MAKER_SONY);
  ADD_MODULE_ATTR(subm, PI_MAKER_EGOMAN);
  ADD_MODULE_ATTR(subm, PI_MAKER_EMBEST);
  ADD_MODULE_ATTR(subm, PI_MAKER_HORIZON);
  ADD_MODULE_ATTR(subm, PI_MAKER_UNKNOWN);
  
  subm.def(
    "wiringPiFailure", 
    [](int fatal, const std::string &message) -> int
    {
      return wiringPiFailure(fatal, message.c_str());
    }, 
    py::arg("fatal"),
    py::arg("message"),
    R"pbdoc(
        Fail. Or not.
    )pbdoc"
  );

  subm.def(
    "wiringPiVersion", 
    []() -> std::tuple<int, int>
    {
      int major, minor;
      wiringPiVersion(&major, &minor);
      return std::make_tuple(major, minor);

    }, 
    R"pbdoc(
      Return our current version number
    )pbdoc"
  );

  subm.def(
    "wiringPiSetup", &wiringPiSetup, 
    R"pbdoc(
      Must be called once at the start of your program execution. Default setup: Initialises the system into wiringPi Pin mode and uses the memory mapped hardware directly. Changed now to revert to "gpio" mode if we're running on a Compute Module.
    )pbdoc"
  );

  subm.def(
    "wiringPiSetupSys", &wiringPiSetupSys, 
    R"pbdoc(
      Must be called once at the start of your program execution. Initialisation (again), however this time we are using the /sys/class/gpio interface to the GPIO systems - slightly slower, but always usable as a non-root user, assuming the devices are already exported and setup correctly.
    )pbdoc"
  );

  subm.def(
    "wiringPiSetupGpio", &wiringPiSetupGpio, 
    R"pbdoc(
      Must be called once at the start of your program execution. GPIO setup: Initialises the system into GPIO Pin mode and uses the memory mapped hardware directly.
    )pbdoc"
  );

  subm.def(
    "wiringPiSetupPhys", &wiringPiSetupPhys, 
    R"pbdoc(
      Must be called once at the start of your program execution.Phys setup: Initialises the system into Physical Pin mode and uses the memory mapped hardware directly.
    )pbdoc"
  );

  subm.def(
    "pinModeAlt", &pinModeAlt, 
    py::arg("pin"),
    py::arg("mode"),
    R"pbdoc(
      This is an un-documented special to let you set any pin to any mode
    )pbdoc"
  );

  subm.def(
    "pinMode", &pinMode, 
    py::arg("pin"),
    py::arg("mode"),
    R"pbdoc(
      Sets the mode of a pin to be input, output or PWM output
    )pbdoc"
  );

  subm.def(
    "pullUpDnControl", &pullUpDnControl, 
    py::arg("pin"),
    py::arg("pud"),
    R"pbdoc(
      Control the internal pull-up/down resistors on a GPIO pin.
    )pbdoc"
  );

  subm.def(
    "digitalRead", &digitalRead, 
    py::arg("pin"),
    R"pbdoc(
      Read the value of a given Pin, returning HIGH or LOW
    )pbdoc"
  );

  subm.def(
    "digitalWrite", &digitalWrite, 
    py::arg("pin"),
    py::arg("value"),
    R"pbdoc(
      Set an output bit
    )pbdoc"
  );

  // delete:
  // digitalRead8
  // digitalWrite8

  subm.def(
    "pwmWrite", &pwmWrite, 
    py::arg("pin"),
    py::arg("value"),
    R"pbdoc(
      Set an output PWM value
    )pbdoc"
  );

  subm.def(
    "analogRead", &analogRead, 
    py::arg("pin"),
    R"pbdoc(
      Read the analog value of a given Pin. There is no on-board Pi analog hardware, so this needs to go to a new node.
    )pbdoc"
  );

  subm.def(
    "analogWrite", &analogWrite, 
    py::arg("pin"),
    py::arg("value"),
    R"pbdoc(
      Write the analog value to the given Pin. There is no on-board Pi analog hardware, so this needs to go to a new node.
    )pbdoc"
  );

  subm.def(
    "piGpioLayout", &piGpioLayout, 
    R"pbdoc(
      Return a number representing the hardware revision of the board. This is not strictly the board revision but is used to check the layout of the GPIO connector.
    )pbdoc"
  );

  subm.def(
    "piBoardId",
    []() -> std::tuple<int, int, int, int, int>
    {
      int model, rev, mem, maker, overVolted;
      piBoardId(&model, &rev, &mem, &maker, &overVolted);
      return std::make_tuple(model, rev, mem, maker, overVolted);
    }, 
    R"pbdoc(
      Return the real details of the board we have.
      This is undocumented and really only intended for the GPIO command.
    )pbdoc"
  );

  subm.def(
    "wpiPinToGpio", &wpiPinToGpio, 
    py::arg("wpiPin"),
    R"pbdoc(
      Translate a wiringPi Pin number to native GPIO pin number.
      Provided for external support.
    )pbdoc"
  );

  subm.def(
    "physPinToGpio", &physPinToGpio, 
    py::arg("physPin"),
    R"pbdoc(
      Translate a physical Pin number to native GPIO pin number.
      Provided for external support.
    )pbdoc"
  );

  subm.def(
    "setPadDrive", &setPadDrive, 
    py::arg("group"),
    py::arg("value"),
    R"pbdoc(
      Set the PAD driver value
    )pbdoc"
  );

  subm.def(
    "getAlt", &getAlt, 
    py::arg("pin"),
    R"pbdoc(
      Returns the ALT bits for a given port. Only really of-use for the gpio readall command (I think)
    )pbdoc"
  );

  subm.def(
    "pwmToneWrite", &pwmToneWrite, 
    py::arg("pin"),
    py::arg("freq"),
    R"pbdoc(
      Pi Specific. Output the given frequency on the Pi's PWM pin
    )pbdoc"
  );

  subm.def(
    "pwmSetMode", &pwmSetMode, 
    py::arg("mode"),
    R"pbdoc(
      Select the native "balanced" mode, or standard mark:space mode
    )pbdoc"
  );

  subm.def(
    "pwmSetRange", &pwmSetRange, 
    py::arg("range"),
    R"pbdoc(
      Set the PWM range register. We set both range registers to the same value. If you want different in your own code, then write your own.
    )pbdoc"
  );

  subm.def(
    "pwmSetClock", &pwmSetClock, 
    py::arg("divisor"),
    R"pbdoc(
      Set/Change the PWM clock. Originally my code, but changed (for the better!) by Chris Hall, <chris@kchall.plus.com>  after further study of the manual and testing with a 'scope
    )pbdoc"
  );

  subm.def(
    "pwmSetFreq", &pwmSetFreq, 
    py::arg("pin"),
    py::arg("frequency"),
    R"pbdoc(
      pwmSetFreq
    )pbdoc"
  );

  subm.def(
    "pwmSetDuty", &pwmSetDuty, 
    py::arg("pin"),
    py::arg("duty_cycle_ns"),
    R"pbdoc(
      pwmSetDuty
    )pbdoc"
  );

  subm.def(
    "gpioClockSet", &gpioClockSet, 
    py::arg("pin"),
    py::arg("freq"),
    R"pbdoc(
      Set the frequency on a GPIO clock pin
    )pbdoc"
  );

  subm.def(
    "digitalReadByte", &digitalReadByte, 
    R"pbdoc(
      digitalReadByte
    )pbdoc"
  );

  subm.def(
    "digitalReadByte2", &digitalReadByte2, 
    R"pbdoc(
      digitalReadByte2
    )pbdoc"
  );

  subm.def(
    "digitalWriteByte", &digitalWriteByte, 
    py::arg("value"),
    R"pbdoc(
      digitalWriteByte
    )pbdoc"
  );

  subm.def(
    "digitalWriteByte2", &digitalWriteByte2, 
    py::arg("value"),
    R"pbdoc(
      digitalWriteByte2
    )pbdoc"
  );

  subm.def(
    "waitForInterrupt", &waitForInterrupt, 
    py::arg("pin"),
    py::arg("mS"),
    R"pbdoc(
      Pi Specific. Wait for Interrupt on a GPIO pin. This is actually done via the /sys/class/gpio interface regardless of the wiringPi access mode in-use. Maybe sometime it might get a better way for a bit more efficiency.
    )pbdoc"
  );


  subm.def(
    "wiringPiISR", 
    [](int pin, int mode, std::function<void()> &func) -> int
    {
      auto fun = func.target<void(*)()>();
      void (*cfun)() = *fun;
      assert(cfun != nullptr);
      return wiringPiISR(pin, mode, cfun);
    }, 
    py::arg("pin"),
    py::arg("mode"),
    py::arg("function"),
    R"pbdoc(
      Pi Specific. Take the details and create an interrupt handler that will do a call-back to the user supplied function.
    )pbdoc"
  ); // 补充py中断回调函数

  subm.def(
    "piThreadCreate", 
    [](std::function<void*(void *)> &func) -> int
    {
      auto fun = func.target<void*(*)(void *)>();
      void* (*cfun)(void *) = *fun;
      assert(cfun != nullptr);
      return piThreadCreate(cfun);
    }, 
    py::arg("fn"),
    R"pbdoc(
      Create and start a thread
    )pbdoc"
  );

  subm.def(
    "piLock", &piLock, 
    py::arg("key"),
    R"pbdoc(
      Activate a mutex.
      We're keeping things simple here and only tracking 4 mutexes which is more than enough for out entry-level pthread programming
    )pbdoc"
  );

  subm.def(
    "piUnlock", &piUnlock, 
    py::arg("key"),
    R"pbdoc(
      Deactivate a mutex.
      We're keeping things simple here and only tracking 4 mutexes which is more than enough for out entry-level pthread programming
    )pbdoc"
  );

  subm.def(
    "piHiPri", &piHiPri, 
    py::arg("pri"),
    R"pbdoc(
      Attempt to set a high priority schedulling for the running program
    )pbdoc"
  );

  subm.def(
    "delay", &delay, 
    py::arg("howLong"),
    R"pbdoc(
      Wait for some number of milliseconds
    )pbdoc"
  );

  subm.def(
    "delayMicroseconds", &delayMicroseconds, 
    py::arg("howLong"),
    R"pbdoc(
      delayMicroseconds
    )pbdoc"
  );

  subm.def(
    "millis", &millis, 
    R"pbdoc(
      Return a number of milliseconds as an unsigned int. Wraps at 49 days.
    )pbdoc"
  );

  subm.def(
    "micros", &micros, 
    R"pbdoc(
      Return a number of microseconds as an unsigned int. Wraps after 71 minutes.
    )pbdoc"
  );
}

void supplement_wiringSerial(py::module_ &subm)
{
  subm.def(
    "serialOpen", &serialOpen, 
    py::arg("device"),
    py::arg("baud"),
    R"pbdoc(
      Open and initialise the serial port, setting all the right port parameters - or as many as are required - hopefully!
    )pbdoc"
  );

  subm.def(
    "serialClose", &serialClose, 
    py::arg("fd"),
    R"pbdoc(
      Release the serial port
    )pbdoc"
  );

  subm.def(
    "serialFlush", &serialFlush, 
    py::arg("fd"),
    R"pbdoc(
      Flush the serial buffers (both tx & rx)
    )pbdoc"
  );

  subm.def(
    "serialPutchar", &serialPutchar, 
    py::arg("fd"),
    py::arg("c"),
    R"pbdoc(
      Send a single character to the serial port
    )pbdoc"
  );

  subm.def(
    "serialPuts", &serialPuts, 
    py::arg("fd"),
    py::arg("message"),
    R"pbdoc(
      Send a string to the serial port
    )pbdoc"
  );

  subm.def(
    "serialDataAvail", &serialDataAvail, 
    py::arg("fd"),
    R"pbdoc(
      Return the number of bytes of data avalable to be read in the serial port
    )pbdoc"
  );

  subm.def(
    "serialGetchar", &serialGetchar, 
    py::arg("fd"),
    R"pbdoc(
      Get a single character from the serial device. Note: Zero is a valid character and this function will time-out after 10 seconds.
    )pbdoc"
  );
}

void supplement_wiringpii2c(py::module_ &subm)
{
  // wiringPiI2C
  subm.def(
    "wiringPiI2CRead", &wiringPiI2CRead, 
    py::arg("fd"),
    R"pbdoc(
      Simple device read
    )pbdoc"
  );

  subm.def(
    "wiringPiI2CReadReg8", &wiringPiI2CReadReg8, 
    py::arg("fd"),
    py::arg("reg"),
    R"pbdoc(
      Read an 8 bit value from a regsiter on the device
    )pbdoc"
  );

  subm.def(
    "wiringPiI2CReadReg16", &wiringPiI2CReadReg16, 
    py::arg("fd"),
    py::arg("reg"),
    R"pbdoc(
      Read an 16 bit value from a regsiter on the device
    )pbdoc"
  );

  subm.def(
    "wiringPiI2CWrite", &wiringPiI2CWrite, 
    py::arg("fd"),
    py::arg("data"),
    R"pbdoc(
      Simple device write
    )pbdoc"
  );

  subm.def(
    "wiringPiI2CWriteReg8", &wiringPiI2CWriteReg8, 
    py::arg("fd"),
    py::arg("reg"),
    py::arg("data"),
    R"pbdoc(
      Write an 8 bit value to the given register
    )pbdoc"
  );

  subm.def(
    "wiringPiI2CWriteReg16", &wiringPiI2CWriteReg16, 
    py::arg("fd"),
    py::arg("reg"),
    py::arg("data"),
    R"pbdoc(
      Write an 16 bit value to the given register
    )pbdoc"
  );

  subm.def(
    "wiringPiI2CSetupInterface", &wiringPiI2CSetupInterface, 
    py::arg("device"),
    py::arg("devId"),
    R"pbdoc(
      Undocumented access to set the interface explicitly - might be used for the Pi's 2nd I2C interface...
    )pbdoc"
  );

  subm.def(
    "wiringPiI2CSetup", &wiringPiI2CSetup, 
    py::arg("devId"),
    R"pbdoc(
      Open the I2C device, and regsiter the target device
    )pbdoc"
  );
}

void supplement_wiringpispi(py::module_ &subm)
{
  // wiringPiSPI
  subm.def(
    "wiringPiSPIGetFd", &wiringPiSPIGetFd, 
    py::arg("channel"),
    R"pbdoc(
      Return the file-descriptor for the given channel
    )pbdoc"
  );

  subm.def(
    "wiringPiSPIDataRW", &wiringPiSPIDataRW, 
    py::arg("channel"),
    py::arg("data"),
    py::arg("len"),
    R"pbdoc(
      Write and Read a block of data over the SPI bus. Note the data ia being read into the transmit buffer, so will overwrite it! This is also a full-duplex operation.
    )pbdoc"
  );

  subm.def(
    "wiringPiSPISetupMode", &wiringPiSPISetupMode, 
    py::arg("channel"),
    py::arg("speed"),
    py::arg("mode"),
    R"pbdoc(
      Open the SPI device, and set it up, with the mode, etc.
    )pbdoc"
  );

  subm.def(
    "wiringPiSPISetup", &wiringPiSPISetup, 
    py::arg("channel"),
    py::arg("speed"),
    R"pbdoc(
      Open the SPI device, and set it up, etc. in the default MODE 0
    )pbdoc"
  );
}

void supplement_wiringpishift(py::module_ &subm)
{
  subm.def(
    "shiftIn", &shiftIn, 
    py::arg("dPin"),
    py::arg("cPin"),
    py::arg("order"),
    R"pbdoc(
      Shift data in from a clocked source
    )pbdoc"
  );

  subm.def(
    "shiftOut", &shiftOut, 
    py::arg("dPin"),
    py::arg("cPin"),
    py::arg("order"),
    py::arg("val"),
    R"pbdoc(
      Shift data out to a clocked source
    )pbdoc"
  );
}


void supplement_wiringpiext(py::module_ &subm)
{
  subm.def(
    "loadWPiExtension", 
    [](const std::string &progName, const std::string &extensionData, int verbose) -> bool
    {
      const int _prognum = progName.length() * 2 + 2;
      const int _extnum = extensionData.length() * 2 + 2;
      char *_progname = new char[_prognum];
      char *_ext = new char[_extnum];
      memset(_progname, 0, _prognum);
      memset(_ext, 0, _extnum);

      memcpy(_progname, progName.c_str(), progName.length());
      memcpy(_ext, extensionData.c_str(), extensionData.length());

      bool state = loadWPiExtension(_progname, _ext, verbose) > 0;

      delete[] _progname;
      delete[] _ext;

      return state;
    }, 
    py::arg("progName"),
    py::arg("extensionData"),
    py::arg("verbose"),
    R"pbdoc(
      Load in a wiringPi extension
      The extensionData always starts with the name, a colon then the pinBase number. Other parameters after that are decoded by the module in question.
    )pbdoc"
  );
}