// Copyright (C) Pololu Corporation.  See www.pololu.com for details.

/// @file PololuSH1106Main.c
#include <stdbool.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "OLED.h"
#include "SH1106.h"

// #define OLED_STREAM

#define textBufferWidth 21
#define textBufferHeight 8

static  bool initialized;
static  bool clearDisplayRamOnNextDisplay;
static  bool disableAutoDisplay;
static  uint8_t textBuffer[textBufferHeight * textBufferWidth];
static  uint8_t textCursorX;
static  uint8_t textCursorY;
static  uint8_t customChars[8][5];
static  const uint8_t * graphicsBuffer;

static const uint8_t pololuOledFont[][5] PROGMEM = {
  // First 32 characters omitted because they are custom or blank.

  // 0x20 space
  {
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000,
    0b00000000
  },

  // 0x21 !
  {
    0b00000000,
    0b00000000,
    0b01001111,
    0b00000000,
    0b00000000
  },

  // 0x22 "
  {
    0b00000000,
    0b00000111,
    0b00000000,
    0b00000111,
    0b00000000
  },

  // 0x23 #
  {
    0b00010100,
    0b01111111,
    0b00010100,
    0b01111111,
    0b00010100
  },

  // 0x24 $
  {
    0b00100100,
    0b00101010,
    0b01111111,
    0b00101010,
    0b00010010
  },

  // 0x25 %
  {
    0b00100011,
    0b00010011,
    0b00001000,
    0b01100100,
    0b01100010
  },

  // 0x26 &
  {
    0b00110110,
    0b01001001,
    0b01010101,
    0b00100010,
    0b01010000
  },

  // 0x27 '
  {
    0b00000000,
    0b00000000,
    0b00000111,
    0b00000000,
    0b00000000
  },

  // 0x28 (
  {
    0b00000000,
    0b00011100,
    0b00100010,
    0b01000001,
    0b00000000
  },

  // 0x29 )
  {
    0b00000000,
    0b01000001,
    0b00100010,
    0b00011100,
    0b00000000
  },

  // 0x2A *
  {
    0b00010100,
    0b00001000,
    0b00111110,
    0b00001000,
    0b00010100
  },

  // 0x2B +
  {
    0b00001000,
    0b00001000,
    0b00111110,
    0b00001000,
    0b00001000
  },

  // 0x2C ,
  {
    0b00000000,
    0b01010000,
    0b00110000,
    0b00000000,
    0b00000000
  },

  // 0x2D -
  {
    0b00001000,
    0b00001000,
    0b00001000,
    0b00001000,
    0b00001000
  },

  // 0x2E .
  {
    0b00000000,
    0b01100000,
    0b01100000,
    0b00000000,
    0b00000000
  },

  // 0x2F /
  {
    0b00100000,
    0b00010000,
    0b00001000,
    0b00000100,
    0b00000010
  },

  // 0x30 0
  {
    0b00111110,
    0b01010001,
    0b01001001,
    0b01000101,
    0b00111110
  },

  // 0x31 1
  {
    0b00000000,
    0b01000010,
    0b01111111,
    0b01000000,
    0b00000000
  },

  // 0x32 2
  {
    0b01000010,
    0b01100001,
    0b01010001,
    0b01001001,
    0b01000110
  },

  // 0x33 3
  {
    0b00100001,
    0b01000001,
    0b01000101,
    0b01001011,
    0b00110001
  },

  // 0x34 4
  {
    0b00011000,
    0b00010100,
    0b00010010,
    0b01111111,
    0b00010000
  },

  // 0x35 5
  {
    0b00100111,
    0b01000101,
    0b01000101,
    0b01000101,
    0b00111001
  },

  // 0x36 6
  {
    0b00111100,
    0b01001010,
    0b01001001,
    0b01001001,
    0b00110000
  },

  // 0x37 7
  {
    0b00000011,
    0b00000001,
    0b01110001,
    0b00001001,
    0b00000111
  },

  // 0x38 8
  {
    0b00110110,
    0b01001001,
    0b01001001,
    0b01001001,
    0b00110110
  },

  // 0x39 9
  {
    0b00000110,
    0b01001001,
    0b01001001,
    0b00101001,
    0b00011110
  },

  // 0x3A :
  {
    0b00000000,
    0b00110110,
    0b00110110,
    0b00000000,
    0b00000000
  },

  // 0x3B ;
  {
    0b00000000,
    0b01010110,
    0b00110110,
    0b00000000,
    0b00000000
  },

  // 0x3C <
  {
    0b00001000,
    0b00010100,
    0b00100010,
    0b01000001,
    0b00000000
  },

  // 0x3D =
  {
    0b00010100,
    0b00010100,
    0b00010100,
    0b00010100,
    0b00010100
  },

  // 0x3E >
  {
    0b00000000,
    0b01000001,
    0b00100010,
    0b00010100,
    0b00001000
  },

  // 0x3F ?
  {
    0b00000010,
    0b00000001,
    0b01010001,
    0b00001001,
    0b00000110
  },

  // 0x40 @
  {
    0b00110010,
    0b01001001,
    0b01111001,
    0b01000001,
    0b00111110
  },

  // 0x41 A
  {
    0b01111110,
    0b00010001,
    0b00010001,
    0b00010001,
    0b01111110
  },

  // 0x42 B
  {
    0b01111111,
    0b01001001,
    0b01001001,
    0b01001001,
    0b00110110
  },

  // 0x43 C
  {
    0b00111110,
    0b01000001,
    0b01000001,
    0b01000001,
    0b00100010
  },

  // 0x44 D
  {
    0b01111111,
    0b01000001,
    0b01000001,
    0b01000001,
    0b00111110
  },

  // 0x45 E
  {
    0b01111111,
    0b01001001,
    0b01001001,
    0b01001001,
    0b01000001
  },

  // 0x46 F
  {
    0b01111111,
    0b00001001,
    0b00001001,
    0b00001001,
    0b00000001
  },

  // 0x47 G
  {
    0b00111110,
    0b01000001,
    0b01001001,
    0b01001001,
    0b01111010
  },

  // 0x48 H
  {
    0b01111111,
    0b00001000,
    0b00001000,
    0b00001000,
    0b01111111
  },

  // 0x49 I
  {
    0b00000000,
    0b01000001,
    0b01111111,
    0b01000001,
    0b00000000
  },

  // 0x4A J
  {
    0b00100000,
    0b01000000,
    0b01000001,
    0b00111111,
    0b00000001
  },

  // 0x4B K
  {
    0b01111111,
    0b00001000,
    0b00010100,
    0b00100010,
    0b01000001
  },

  // 0x4C L
  {
    0b01111111,
    0b01000000,
    0b01000000,
    0b01000000,
    0b01000000
  },

  // 0x4D M
  {
    0b01111111,
    0b00000010,
    0b00001100,
    0b00000010,
    0b01111111
  },

  // 0x4E N
  {
    0b01111111,
    0b00000100,
    0b00001000,
    0b00010000,
    0b01111111
  },

  // 0x4F O
  {
    0b00111110,
    0b01000001,
    0b01000001,
    0b01000001,
    0b00111110
  },

  // 0x50 P
  {
    0b01111111,
    0b00001001,
    0b00001001,
    0b00001001,
    0b00000110
  },

  // 0x51 Q
  {
    0b00111110,
    0b01000001,
    0b01010001,
    0b00100001,
    0b01011110
  },

  // 0x52 R
  {
    0b01111111,
    0b00001001,
    0b00011001,
    0b00101001,
    0b01000110
  },

  // 0x53 S
  {
    0b01000110,
    0b01001001,
    0b01001001,
    0b01001001,
    0b00110001
  },

  // 0x54 T
  {
    0b00000001,
    0b00000001,
    0b01111111,
    0b00000001,
    0b00000001
  },

  // 0x55 U
  {
    0b00111111,
    0b01000000,
    0b01000000,
    0b01000000,
    0b00111111
  },

  // 0x56 V
  {
    0b00011111,
    0b00100000,
    0b01000000,
    0b00100000,
    0b00011111
  },

  // 0x57 W
  {
    0b00111111,
    0b01000000,
    0b00111000,
    0b01000000,
    0b00111111
  },

  // 0x58 X
  {
    0b01100011,
    0b00010100,
    0b00001000,
    0b00010100,
    0b01100011
  },

  // 0x59 Y
  {
    0b00000111,
    0b00001000,
    0b01110000,
    0b00001000,
    0b00000111
  },

  // 0x5A Z
  {
    0b01100001,
    0b01010001,
    0b01001001,
    0b01000101,
    0b01000011
  },

  // 0x5B [
  {
    0b00000000,
    0b01111111,
    0b01000001,
    0b01000001,
    0b00000000
  },

  // 0x5C backslash
  {
    0b00000010,
    0b00000100,
    0b00001000,
    0b00010000,
    0b00100000
  },

  // 0x5D ]
  {
    0b00000000,
    0b01000001,
    0b01000001,
    0b01111111,
    0b00000000
  },

  // 0x5E ^
  {
    0b00000100,
    0b00000010,
    0b00000001,
    0b00000010,
    0b00000100
  },

  // 0x5F _
  {
    0b01000000,
    0b01000000,
    0b01000000,
    0b01000000,
    0b01000000
  },

  // 0x60 `
  {
    0b00000000,
    0b00000001,
    0b00000010,
    0b00000100,
    0b00000000
  },

  // 0x61 a
  {
    0b00100000,
    0b01010100,
    0b01010100,
    0b01010100,
    0b01111000
  },

  // 0x62 b
  {
    0b01111111,
    0b01001000,
    0b01000100,
    0b01000100,
    0b00111000
  },

  // 0x63 c
  {
    0b00111000,
    0b01000100,
    0b01000100,
    0b01000100,
    0b00100000
  },

  // 0x64 d
  {
    0b00111000,
    0b01000100,
    0b01000100,
    0b01001000,
    0b01111111
  },

  // 0x65 e
  {
    0b00111000,
    0b01010100,
    0b01010100,
    0b01010100,
    0b00011000
  },

  // 0x66 f
  {
    0b00001000,
    0b01111110,
    0b00001001,
    0b00000001,
    0b00000010
  },

  // 0x67 g
  {
    0b00001100,
    0b01010010,
    0b01010010,
    0b01010010,
    0b00111110
  },

  // 0x68 h
  {
    0b01111111,
    0b00001000,
    0b00000100,
    0b00000100,
    0b01111000
  },

  // 0x69 i
  {
    0b00000000,
    0b01000100,
    0b01111101,
    0b01000000,
    0b00000000
  },

  // 0x6A j
  {
    0b00100000,
    0b01000000,
    0b01000100,
    0b00111101,
    0b00000000
  },

  // 0x6B k
  {
    0b01111111,
    0b00010000,
    0b00101000,
    0b01000100,
    0b00000000
  },

  // 0x6C l
  {
    0b00000000,
    0b01000001,
    0b01111111,
    0b01000000,
    0b00000000
  },

  // 0x6D m
  {
    0b01111100,
    0b00000100,
    0b00011000,
    0b00000100,
    0b01111000
  },

  // 0x6E n
  {
    0b01111100,
    0b00001000,
    0b00000100,
    0b00000100,
    0b01111000
  },

  // 0x6F o
  {
    0b00111000,
    0b01000100,
    0b01000100,
    0b01000100,
    0b00111000
  },

  // 0x70 p
  {
    0b01111100,
    0b00010100,
    0b00010100,
    0b00010100,
    0b00001000
  },

  // 0x71 q
  {
    0b00001000,
    0b00010100,
    0b00010100,
    0b00011000,
    0b01111100
  },

  // 0x72 r
  {
    0b01111100,
    0b00001000,
    0b00000100,
    0b00000100,
    0b00001000
  },

  // 0x73 s
  {
    0b01001000,
    0b01010100,
    0b01010100,
    0b01010100,
    0b00100000
  },

  // 0x74 t
  {
    0b00000100,
    0b00111111,
    0b01000100,
    0b01000000,
    0b00100000
  },

  // 0x75 u
  {
    0b00111100,
    0b01000000,
    0b01000000,
    0b00100000,
    0b01111100
  },

  // 0x76 v
  {
    0b00011100,
    0b00100000,
    0b01000000,
    0b00100000,
    0b00011100
  },

  // 0x77 w
  {
    0b00111100,
    0b01000000,
    0b00111000,
    0b01000000,
    0b00111100
  },

  // 0x78 x
  {
    0b01000100,
    0b00101000,
    0b00010000,
    0b00101000,
    0b01000100
  },

  // 0x79 y
  {
    0b00001100,
    0b01010000,
    0b01010000,
    0b01010000,
    0b00111100
  },

  // 0x7A z
  {
    0b01000100,
    0b01100100,
    0b01010100,
    0b01001100,
    0b01000100
  },

  // 0x7B {
  {
    0b00000000,
    0b00001000,
    0b00110110,
    0b01000001,
    0b00000000
  },

  // 0x7C |
  {
    0b00000000,
    0b00000000,
    0b01111111,
    0b00000000,
    0b00000000
  },

  // 0x7D }
  {
    0b00000000,
    0b01000001,
    0b00110110,
    0b00001000,
    0b00000000
  },

  // 0x7E ~
  {
    0b00001100,
    0b00000010,
    0b00000100,
    0b00001000,
    0b00000110
  },

  // 0x7F DEL - we put a left arrow here
  {
    0b00001000,
    0b00011100,
    0b00101010,
    0b00001000,
    0b00001000
  },
};

  //// Low-level helpers

static void clearDisplayRam()
  {
    sh1106TransferStart();
    sh1106CommandMode();
    sh1106Write(SH1106_SET_COLUMN_ADDR_LOW | 2);
    for (uint8_t page = 0; page < 8; page++)
    {
      sh1106CommandMode();
      sh1106Write(SH1106_SET_PAGE_ADDR | page);
      sh1106Write(SH1106_SET_COLUMN_ADDR_HIGH | 0);
      sh1106DataMode();
      for (uint8_t i = 0; i < 128; i++)
      {
        sh1106Write(0);
      }
    }
    sh1106TransferEnd();
    clearDisplayRamOnNextDisplay = false;
  }

static  void configureDefault()
  {
    sh1106TransferStart();
    sh1106CommandMode();
    sh1106Write(SH1106_SET_SEGMENT_REMAP | 1);  // flip horizontally
    sh1106Write(SH1106_SET_COM_SCAN_DIR | 8);   // flip vertically
    sh1106Write(SH1106_SET_CONTRAST);
    sh1106Write(0xFF);                // maximum brightness
    sh1106Write(SH1106_SET_DISPLAY_ON | 1);
    sh1106TransferEnd();
  }

  //////// Initialization and settings

static  void init2()
  {
    sh1106InitPins();
    sh1106Reset();
    clearDisplayRam();
    configureDefault();
    initialized = true;
  }

  //////// Display routines

static  uint8_t getGlyphColumn(uint8_t glyph, uint8_t pixelX)
  {
    if (glyph >= 0x20)
    {
      return pgm_read_byte(&pololuOledFont[glyph - 0x20][pixelX]);
    }
    else if (glyph < 8)
    {
      return customChars[glyph][pixelX];
    }
    else
    {
      return 0;
    }
  }

static void writeSegmentText(uint8_t page, uint8_t columnAddr,
    const uint8_t * text, uint8_t textLength)
  {
    sh1106CommandMode();
    sh1106Write(SH1106_SET_PAGE_ADDR | page);
    sh1106Write(SH1106_SET_COLUMN_ADDR_HIGH | (columnAddr >> 4));
    sh1106Write(SH1106_SET_COLUMN_ADDR_LOW | (columnAddr & 0xF));
    sh1106DataMode();
    for (uint8_t i = 0; i < textLength; i++)
    {
      uint8_t glyph = *text++;
      for (uint8_t pixelX = 0; pixelX < 5; pixelX++)
      {
        sh1106Write(getGlyphColumn(glyph, pixelX));
      }
      sh1106Write(0);
    }
  }

static void writeSegmentTextAndGraphics(uint8_t page, uint8_t columnAddr,
    const uint8_t * text, uint8_t textLength)
  {
    sh1106CommandMode();
    sh1106Write(SH1106_SET_PAGE_ADDR | page);
    sh1106Write(SH1106_SET_COLUMN_ADDR_HIGH | (columnAddr >> 4));
    sh1106Write(SH1106_SET_COLUMN_ADDR_LOW | (columnAddr & 0xF));
    sh1106DataMode();
    const uint8_t * g = graphicsBuffer + page * 128 + (columnAddr - 2);
    for (uint8_t i = 0; i < textLength; i++)
    {
      uint8_t glyph = *text++;
      for (uint8_t pixelX = 0; pixelX < 5; pixelX++)
      {
        sh1106Write(getGlyphColumn(glyph, pixelX) ^ *g++);
      }
      sh1106Write(*g++);
    }
  }

static void writePageTextAndGraphics(uint8_t page, const uint8_t * text,
    uint8_t leftMargin, uint8_t textLength)
  {
    sh1106CommandMode();
    sh1106Write(SH1106_SET_PAGE_ADDR | page);
    sh1106Write(SH1106_SET_COLUMN_ADDR_HIGH | 0);
    sh1106Write(SH1106_SET_COLUMN_ADDR_LOW | 2);
    sh1106DataMode();
    const uint8_t * g = graphicsBuffer + page * 128;
    for (uint8_t i = 0; i < leftMargin; i++) { sh1106Write(*g++); }
    for (uint8_t textX = 0; textX < textLength; textX++)
    {
      uint8_t glyph = *text++;
      for (uint8_t pixelX = 0; pixelX < 5; pixelX++)
      {
        sh1106Write(getGlyphColumn(glyph, pixelX) ^ *g++);
       }
       sh1106Write(*g++);
    }
    for (uint8_t x = leftMargin + textLength * 6; x < 128; x++)
    {
      sh1106Write(*g++);
    }
  }


  //// 21x8 layout /////////////////////////////////////////////////////////////
  //   Character size:               5x8
  //   Character horizontal margin:  1
  //   Screen left margin:           0
  //   Screen right margin:          3  (off center: more room for graphics)
  //   Line number = Page number

static void display21x8TextPartial(uint8_t x, uint8_t y, uint8_t width)
  {
    if (x >= 21 || y >= 8) { return; }
    if (width > (uint8_t)(21 - x)) { width = 21 - x; }
    if (width == 0) { return; }

    const uint8_t columnAddr = 2 + x * 6;
    const uint8_t * const textStart = getLinePointer(y) + x;

    sh1106TransferStart();
    writeSegmentText(y, columnAddr, textStart, width);
    sh1106TransferEnd();
  }

static void display21x8Text()
  {
    sh1106TransferStart();
    writeSegmentText(0, 2, getLinePointer(0), 21);
    writeSegmentText(1, 2, getLinePointer(1), 21);
    writeSegmentText(2, 2, getLinePointer(2), 21);
    writeSegmentText(3, 2, getLinePointer(3), 21);
    writeSegmentText(4, 2, getLinePointer(4), 21);
    writeSegmentText(5, 2, getLinePointer(5), 21);
    writeSegmentText(6, 2, getLinePointer(6), 21);
    writeSegmentText(7, 2, getLinePointer(7), 21);
    sh1106TransferEnd();
  }

static void display21x8TextAndGraphicsPartial(uint8_t x, uint8_t y, uint8_t width)
  {
    if (x >= 21 || y >= 8) { return; }
    if (width > (uint8_t)(21 - x)) { width = 21 - x; }
    if (width == 0) { return; }

    const uint8_t columnAddr = 2 + x * 6;
    uint8_t * const text = getLinePointer(y) + x;

    sh1106TransferStart();
    writeSegmentTextAndGraphics(y, columnAddr, text, width);
    sh1106TransferEnd();
  }

static void display21x8TextAndGraphics()
  {
    sh1106TransferStart();
    writePageTextAndGraphics(0, getLinePointer(0), 0, 21);
    writePageTextAndGraphics(1, getLinePointer(1), 0, 21);
    writePageTextAndGraphics(2, getLinePointer(2), 0, 21);
    writePageTextAndGraphics(3, getLinePointer(3), 0, 21);
    writePageTextAndGraphics(4, getLinePointer(4), 0, 21);
    writePageTextAndGraphics(5, getLinePointer(5), 0, 21);
    writePageTextAndGraphics(6, getLinePointer(6), 0, 21);
    writePageTextAndGraphics(7, getLinePointer(7), 0, 21);
    sh1106TransferEnd();
  }

void init() { if (!initialized) { init2(); } }

void reinitialize()
  {
    init2();
  }

  /// @brief Configures the OLED to invert all the pixels, resulting in
  /// black-on-white text.
  void invert()
  {
    init();
    sh1106TransferStart();
    sh1106CommandMode();
    sh1106Write(SH1106_SET_INVERT_DISPLAY | 1);
    sh1106TransferEnd();
  }

  /// @brief Configures the OLED to not invert its pixels (the default).
  void noInvert()
  {
    init();
    sh1106TransferStart();
    sh1106CommandMode();
    sh1106Write(SH1106_SET_INVERT_DISPLAY | 0);
    sh1106TransferEnd();
  }

  /// @brief Configures the OLED to rotate its display 180 degrees from normal.
  void rotate180()
  {
    init();
    sh1106TransferStart();
    sh1106CommandMode();
    sh1106Write(SH1106_SET_SEGMENT_REMAP);
    sh1106Write(SH1106_SET_COM_SCAN_DIR);
    sh1106TransferEnd();
  }

  /// @brief Configures the OLED to use the default orientation.
  ///
  /// This is the orientation that results in the text shown on the
  /// [Graphical OLED Display: 128x64, 1.3", White, SPI][3760]
  /// from Pololu being the same orientation as the pin labels.
  ///
  /// [3760]: https://www.pololu.com/product/3760
  void noRotate()
  {
    init();
    sh1106TransferStart();
    sh1106CommandMode();
    sh1106Write(SH1106_SET_SEGMENT_REMAP | 1);
    sh1106Write(SH1106_SET_COM_SCAN_DIR | 8);
    sh1106TransferEnd();
  }

  /// @brief Sets the contrast (i.e. brightness) of the OLED.
  ///
  /// @param contrast A number between 0 (darkest, but still visible) and
  /// 255 (brightest, default).
  void setContrast(uint8_t contrast)
  {
    init();
    sh1106TransferStart();
    sh1106CommandMode();
    sh1106Write(SH1106_SET_CONTRAST);
    sh1106Write(contrast);
    sh1106TransferEnd();
  }

static  void (*displayFunction)() = &display21x8Text;
static  void (*displayPartialFunction)(uint8_t, uint8_t, uint8_t) = &display21x8TextPartial;

  /// @brief Configures this library to use a layout with
  /// 21 columns and 8 rows of text.
  void setLayout21x8()
  {
    graphicsBuffer = NULL;
    displayFunction = &display21x8Text;
    displayPartialFunction = &display21x8TextPartial;
    clearDisplayRamOnNextDisplay = true;
  }

  /// @brief Configures this library to use a layout with 21 columns and
  /// 8 rows of text, XORed with a graphics buffer.
  ///
  /// @param graphics A pointer to a 1024-byte graphics buffer.
  void setLayout21x8WithGraphics(const uint8_t * graphics)
  {
    graphicsBuffer = graphics;
    displayFunction = &display21x8TextAndGraphics;
    displayPartialFunction = &display21x8TextAndGraphicsPartial;
    clearDisplayRamOnNextDisplay = true;
  }

  /// @brief Writes all of the text/graphics to the OLED.
  ///
  /// This also turns on auto display mode, undoing the effect of
  /// noAutoDisplay().
  ///
  /// Note that this function does not always write to the entire display:
  /// for text-only modes, it will normally only write to the portion of the
  /// screen containing text.
  void display()
  {
    init();
    if (clearDisplayRamOnNextDisplay) { clearDisplayRam(); }
    (*displayFunction)();
    disableAutoDisplay = false;
  }

  /// @brief Writes a certain region of text/graphics to the OLED.
  ///
  /// This function is like display(), but it only writes text/graphics to the
  /// OLED which are in a region corresponding to one or more consecutive
  /// characters in a line of text.
  /// This function cannot write arbitrary regions.
  ///
  /// Most users will not need to call this because the clear(), write(), and
  /// print() functions automatically write to the display as needed.
  /// You can also use the display() function to write all of the
  /// text/graphics.
  ///
  /// However, this function is useful to users who have disabled auto
  /// display mode, or are directly writing to the text buffer, and want an
  /// efficient way to update part of the screen.
  ///
  /// @param x The row number of the text to update (0 means top row).
  /// @param y The column number of the first character to update
  ///          (0 means left-most column).
  /// @param width The number of characters to update.
  void displayPartial(uint8_t x, uint8_t y, uint8_t width)
  {
    init();
    if (clearDisplayRamOnNextDisplay) { clearDisplayRam(); }
    (*displayPartialFunction)(x, y, width);
  }


  //////// Text

  /// @brief Turns off auto display mode.
  ///
  /// This causes the clear(), write(), and print() functions to not perform
  /// any I/O or write any data to the OLED.
  ///
  /// Calling display() will write to the display and turn auto display mode
  /// on again.
  void noAutoDisplay()
  {
    disableAutoDisplay = true;
  }

  /// @brief Gets a pointer to a line of text in this library's text buffer.
  ///
  /// This is for advanced users who want to use their own code to directly
  /// manipulate the text buffer.
  ///
  /// The returned pointer will point to a region of memory at least 21 bytes
  /// long that holds the specified line of text.  You can perform arbitrary
  /// operations on these bytes.
  ///
  /// Note that you should not assume anything about where the lines are in
  /// relation to each other, and you should not assume it is safe to write
  /// beyond the 21st byte of a line.
  ///
  /// Note that functions like snprintf will add a null (0) character at the
  /// end of their output.  This is probably undesirable if you have configured
  /// character 0 to be a custom character using loadCustomCharacter().
  /// Also, it means that the maximum content they can safely write to the text
  /// buffer is limited to 19 characters.
  uint8_t * getLinePointer(uint8_t line)
  {
    return textBuffer + line * textBufferWidth;
  }

  /// @brief Changes the location of the text cursor.
  ///
  /// This function changes the text cursor, which is the location of the text
  /// that will be overwritten by the next call to write() or print().
  ///
  /// @param x The column number (0 means left-most column).
  /// @param y The row number (0 means top row).
  void gotoXY(uint8_t x, uint8_t y)
  {
    textCursorX = x;
    textCursorY = y;
  }

  /// @brief Gets the X coordinate of the text cursor.
  uint8_t getX() { return textCursorX; }

  /// @brief Gets the Y coordinate of the text cursor.
  uint8_t getY() { return textCursorY; }

  /// @brief Moves all the text up one row. (Does not change the cursor position.)
  void scrollDisplayUp()
  {
    memmove(textBuffer, textBuffer + textBufferWidth, textBufferWidth * (textBufferHeight - 1));
    memset(textBuffer + textBufferWidth * (textBufferHeight - 1), ' ', textBufferWidth);
    if (!disableAutoDisplay) { display(); }
  }

  /// @brief Clears the text and resets the text cursor to the upper left.
  ///
  /// After calling this function, the text buffer will consist entirely of
  /// space characters.
  ///
  /// By default, this function also calls display() to write these changes
  /// to the OLED, but noAutoDisplay() disables that behavior.
  void clear()
  {
    memset(textBuffer, ' ', sizeof(textBuffer));
    gotoXY(0, 0);
    if (!disableAutoDisplay) { display(); }
  }

  /// @brief Writes a single character of text.
  ///
  /// This is equivalent to writing a single character using
  /// write(const uint8_t *, size_t).
  int OLED_write(char c)
  {
    if (textCursorY >= textBufferHeight) { return 0; }
    if (textCursorX >= textBufferWidth) { return 0; }

    *(getLinePointer(textCursorY) + textCursorX) = c;

    if (!disableAutoDisplay)
    {
      displayPartial(textCursorX, textCursorY, 1);
    }

    textCursorX++;
    return 0;
  }

  /// @brief Defines a custom character from RAM.
  /// @param picture A pointer to the character dot pattern, in RAM.
  /// @param number A character code between 0 and 7.
  void loadCustomCharacterFromRam(const uint8_t * picture, uint8_t number)
  {
    uint8_t * columns = customChars[number];
    for (uint8_t i = 0; i < 5; i++)
    {
      columns[i] = 0;
    }
    for (uint8_t i = 0; i < 8; i++)
    {
      uint8_t row = picture[i];
      uint8_t mask = 1 << i;
      if (row & (1 << 0)) { columns[4] |= mask; }
      if (row & (1 << 1)) { columns[3] |= mask; }
      if (row & (1 << 2)) { columns[2] |= mask; }
      if (row & (1 << 3)) { columns[1] |= mask; }
      if (row & (1 << 4)) { columns[0] |= mask; }
    }
  }

  /// @brief Defines a custom character.
  /// @param picture A pointer to the character dot pattern, in program space.
  /// @param number A character code between 0 and 7.
  void loadCustomCharacter(const uint8_t * picture, uint8_t number)
  {
    uint8_t ram_picture[8];
    for (uint8_t i = 0; i < 8; i++)
    {
      ram_picture[i] = pgm_read_byte(picture + i);
    }
    loadCustomCharacterFromRam(ram_picture, number);
  }

