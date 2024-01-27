#include "submodules.h"
#include <lcd.h>
#include <lcd128x64.h>
#include <tuple>

void supplement_lcd128x64(py::module_ &subm);

void add_submodule_lcd(py::module_ &m)
{
  auto subm = m.def_submodule(
    "lcd", 
    R"pbdoc(
        Text-based LCD driver.
          This is designed to drive the parallel interface LCD drivers based in the Hitachi HD44780U controller and compatables.

        lcd128x64: Graphics-based LCD driver.
          This is designed to drive the parallel interface LCD drivers based on the generic 12864H chips
        
          There are many variations on these chips, however they all mostly seem to be similar.

          This implementation has the Pins from the Pi hard-wired into it, in particular wiringPi pins 0-7 so that we can use digitalWriteByete() to speed things up somewhat.

        Copyright (c) 2016 Gordon Henderson
    )pbdoc"
  );

  subm.def(
    "lcdHome", 
    &lcdHome, 
    py::arg("fd"),
    R"pbdoc(
        Home the cursor.
    )pbdoc"
  );

  subm.def(
    "lcdClear", 
    &lcdClear,
    py::arg("fd"), 
    R"pbdoc(
        clear the screen.
    )pbdoc"
  );
  
  subm.def(
    "lcdDisplay", 
    &lcdDisplay, 
    py::arg("fd"), py::arg("state"), 
    R"pbdoc(
        Turn the display on/off
    )pbdoc"
  );

  subm.def(
    "lcdCursor", 
    &lcdCursor, 
    py::arg("fd"), py::arg("state"), 
    R"pbdoc(
        Turn the cursor on/off
    )pbdoc"
  );

  subm.def(
    "lcdCursorBlink", 
    &lcdCursorBlink, 
    py::arg("fd"), py::arg("state"), 
    R"pbdoc(
        Turn the cursor blinking on/off
    )pbdoc"
  );

  subm.def(
    "lcdSendCommand", 
    &lcdSendCommand, 
    py::arg("fd"), py::arg("command"), 
    R"pbdoc(
        Send any arbitary command to the display
    )pbdoc"
  );

  subm.def(
    "lcdPosition", 
    &lcdPosition, 
    py::arg("fd"), 
    py::arg("x"), 
    py::arg("y"), 
    R"pbdoc(
        Update the position of the cursor on the display.
        Ignore invalid locations.
    )pbdoc"
  );

  subm.def(
    "lcdCharDef", 
    [](int fd, int index, const py::array_t<unsigned char> &data)
    {
      unsigned char buffer[8];
      array1d_to_ptr(data, buffer, 8);
      lcdCharDef(fd, index, buffer);
    }, 
    py::arg("fd"), 
    py::arg("index"), 
    py::arg("data"), 
    R"pbdoc(
        Defines a new character in the CGRAM
    )pbdoc"
  );
  
  subm.def(
    "lcdPutchar", 
    &lcdPutchar, 
    py::arg("fd"), 
    py::arg("data"), 
    R"pbdoc(
        Send a data byte to be displayed on the display. We implement a very simple terminal here - with line wrapping, but no scrolling. Yet.
    )pbdoc"
  );
  
  subm.def(
    "lcdPuts", 
    lcdPuts,
    py::arg("fd"), 
    py::arg("string"), 
    R"pbdoc(
        Send a string to be displayed on the display
    )pbdoc"
  );
  
  subm.def(
    "lcdInit", 
    &lcdInit, 
    py::arg("rows"), py::arg("cols"), 
    py::arg("bits"), 
    py::arg("rs"),  py::arg("strb"), 
    py::arg("d0"), py::arg("d1"), 
    py::arg("d2"), py::arg("d3"), 
    py::arg("d4"), py::arg("d5"), 
    py::arg("d6"), py::arg("d7"), 
    R"pbdoc(
        Take a lot of parameters and initialise the LCD, and return a handle to that LCD, or -1 if any error.
    )pbdoc"
  );

  supplement_lcd128x64(subm);
}


void supplement_lcd128x64(py::module_ &subm)
{
  subm.def(
    "lcd128x64setOrigin", 
    &lcd128x64setOrigin, 
    py::arg("x"), py::arg("y"),
    R"pbdoc(
        Set the display offset origin
    )pbdoc"
  );

  subm.def(
    "lcd128x64setOrientation", 
    &lcd128x64setOrientation, 
    py::arg("orientation"),
    R"pbdoc(
        Set the display orientation:
        0: Normal, the display is portrait mode, 0,0 is top left
        1: Landscape
        2: Portrait, flipped
        3: Landscape, flipped
    )pbdoc"
  );

  subm.def(
    "lcd128x64orientCoordinates", 
    [](int x, int y) -> std::tuple<int, int>
    {
      lcd128x64orientCoordinates(&x, &y);
      return std::make_tuple(x, y);
    },
    py::arg("x"),
    py::arg("y"),
    R"pbdoc(
        Adjust the coordinates given to the display orientation
    )pbdoc"
  );

  subm.def(
    "lcd128x64getScreenSize", 
    []() -> std::tuple<int, int>
    {
      int x = 0, y = 0;
      lcd128x64getScreenSize(&x, &y);
      return std::make_tuple(x, y);
    }, 
    R"pbdoc(
        Return the max X & Y screen sizes. Needs to be called again, if you change screen orientation.
    )pbdoc"
  );

  subm.def(
    "lcd128x64point", 
    &lcd128x64point, 
    py::arg("x"), py::arg("y"),
    py::arg("colour"),
    R"pbdoc(
        Plot a pixel.
    )pbdoc"
  );

  // 这里可以优化一些的
  subm.def(
    "lcd128x64line", 
    &lcd128x64line, 
    py::arg("x0"), py::arg("y0"),
    py::arg("x1"), py::arg("y1"),
    py::arg("colour"),
    R"pbdoc(
        Classic Bressenham Line code
    )pbdoc"
  );

  subm.def(
    "lcd128x64lineTo", 
    &lcd128x64lineTo, 
    py::arg("x"), py::arg("y"),
    py::arg("colour"),
    R"pbdoc(
        lcd128x64lineTo
    )pbdoc"
  );

  subm.def(
    "lcd128x64rectangle", 
    &lcd128x64rectangle, 
    py::arg("x1"), py::arg("y1"),
    py::arg("x2"), py::arg("y2"),
    py::arg("colour"), py::arg("filled"),
    R"pbdoc(
        A rectangle is a spoilt days fishing
    )pbdoc"
  );

  subm.def(
    "lcd128x64circle", 
    &lcd128x64circle, 
    py::arg("x"), py::arg("y"),
    py::arg("r"), 
    py::arg("colour"), py::arg("filled"),
    R"pbdoc(
        This is the midpoint circle algorithm.
    )pbdoc"
  );

  subm.def(
    "lcd128x64ellipse", 
    &lcd128x64ellipse, 
    py::arg("cx"), py::arg("cy"),
    py::arg("xRadius"), py::arg("yRadius"),
    py::arg("colour"), py::arg("filled"),
    R"pbdoc(
        Fast ellipse drawing algorithm by John Kennedy
    )pbdoc"
  );

  subm.def(
    "lcd128x64putchar", 
    &lcd128x64putchar, 
    py::arg("x"), py::arg("y"),
    py::arg("c"), 
    py::arg("bgCol"), py::arg("fgCol"),
    R"pbdoc(
        Print a single character to the screen
    )pbdoc"
  );

  subm.def(
    "lcd128x64puts", 
    &lcd128x64puts, 
    py::arg("x"), py::arg("y"),
    py::arg("text"), 
    py::arg("bgCol"), py::arg("fgCol"),
    R"pbdoc(
        Send a string to the display. Obeys \n and \r formatting
    )pbdoc"
  );

  subm.def(
    "lcd128x64update", 
    &lcd128x64update, 
    R"pbdoc(
        Copy our software version to the real display
    )pbdoc"
  );

  // subm.def(
  //   "lcd128x64clear", 
  //   &lcd128x64clear, 
  //   py::arg("colour"), 
  //   R"pbdoc(
  //       lcd128x64clear
  //   )pbdoc"
  // );

  subm.def(
    "lcd128x64setup", 
    &lcd128x64setup, 
    R"pbdoc(
        lcd128x64setup
    )pbdoc"
  );
}
