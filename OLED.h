// Copyright (C) Pololu Corporation.  See www.pololu.com for details.

/// @file OLED.h

#define SH1106_SET_COLUMN_ADDR_LOW 0x00
#define SH1106_SET_COLUMN_ADDR_HIGH 0x10
#define SH1106_SET_CONTRAST 0x81
#define SH1106_SET_SEGMENT_REMAP 0xA0
#define SH1106_SET_INVERT_DISPLAY 0xA6
#define SH1106_SET_DISPLAY_ON 0xAE
#define SH1106_SET_PAGE_ADDR 0xB0
#define SH1106_SET_COM_SCAN_DIR 0xC0

/// @brief This class makes it easy to display text and graphics on a
/// 128x64 SH1106 OLED.
///
/// @section core Core class
///
/// Instead of passing pin numbers to this class to specify what pins your
/// OLED is on, you pass a core class as a first template parameter.  The
/// core class must implement the following public functions which take care of
/// communication with the SH1106 at a low level:
///
/// - void sh1106InitPins();
/// - void sh1106Reset();
/// - void sh1106TransferStart();
/// - void sh1106TransferEnd();
/// - void sh1106CommandMode();
/// - void sh1106DataMode();
/// - void sh1106Write(uint8_t);
///
/// For an example implementation of a core class, see PololuSH1106Core.
///
/// @section text-buffer Text buffer
///
/// This class holds a text buffer to keep track of what text to show on the
/// screen.  The clear(), write(), and print() functions all write to the
/// text buffer.
///
/// This class inherits from the Arduino Print class
/// (via PololuSH1106Main), so you can call print() with a variety of arguments.
/// See the [Arduino print() documentation](http://arduino.cc/en/Serial/Print)
/// for more information.
///
/// The default contents of the text buffer, and the contents after you call
/// clear(), consists entirely of space characters.
///
/// You can also use the getLinePointer() function to get a pointer
/// to a specific line in the text buffer, and then manipulate the characters
/// using arbitrary code.
///
/// @section font Font
///
/// This library maps each character code from 0 to 255 to a corresponding
/// 8x5 image.
///
/// Characters from 0 to 7 are custom characters: they are blank by default, but
/// you can use loadCustomCharacter() or loadCustomCharacterFromRam() to
/// specify their appearance.
///
/// Characters 8 through 31 are hardcoded to be blank.
///
/// Characters 32 through 255 come from pololuOledFont.
///
/// It is possible to change the appearance (but not the size) of the font used
/// for characters 32 through 255 without modifying this library.
/// To do so, simply copy the file font.cpp into your sketch directory,
/// remove `__attribute__((weak))`, and then make your changes.
///
/// @section graphics-buffer Graphics buffer
///
/// This class can be configured to hold a pointer to an external 1024-byte
/// graphics buffer that will get mixed (XORed) with the text.
/// Each byte of the graphics buffer represents a 1x8 vertical column of pixels,
/// with the least-significant bit holding the top-most pixel.
/// The first byte represents the pixels in the upper left corner of the screen,
/// and the bytes in the buffer are ordered left-to-right, then top-to-bottom.
/// (So byte at offset 128 is displayed immediately below the byte at offset 0.)
///
/// You can define and manipulate this graphics buffer using your own code, or
/// you can use a third-party library such as Adafruit_SSD1306 to do it.
///
/// @section lcd-compat HD44780 LCD class compatibility
///
/// This class implements clear(), gotoXY(), write(),
/// print() (provided by the Arduino Print class),
/// loadCustomCharacter(), and loadCustomCharacterFromRam(),
/// so a class based on PololuSH1106Main can be used as a
/// a drop-in replacement for a class based on PololuHD44780Base from the
/// [PololuHD44780 library](https://github.com/pololu/pololu-hd44780-arduino)
/// in most applications.
///
/// However, one difference between the SH1106 OLED and the HD44780 LCD is that
/// if you have an application that frequently clears the screen and then
/// redraws it, you will probably notice a flickering effect due to the fast
/// response time of the OLED.
/// To solve this, you can call noAutoDisplay() before clear() to tell
/// the library to refrain from writing to the display automatically.
/// Then, when you are done using functions such as clear(), gotoXY(), write(),
/// and print() to update the desired text on the screen, you can call
/// display() to write the new contents to the entire screen without flicker.
///
/// The display() function turns auto-display mode back on, so you will need to
/// call noAutoDisplay() again whenever you want to do a flickerless update.

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>

  /// @brief Reinitializes the OLED and its settings.
  ///
  /// This redoes the initialization that is automatically done by init() the
  /// first time any function is called that writes to the OLED.
  ///
  /// This can be useful if you want to plug the OLED into the robot after the
  /// library has already been initialized.
  ///
  /// The screen will be blank after calling this, so you might want to call
  /// display() to show the graphics/text that you were shown before.
extern void reinitialize();

void init(); 

  /// @brief Configures the OLED to invert all the pixels, resulting in
  /// black-on-white text.
extern void invert();

  /// @brief Configures the OLED to not invert its pixels (the default).
extern void noInvert();

  /// @brief Configures the OLED to rotate its display 180 degrees from normal.
extern void rotate180();

  /// @brief Configures the OLED to use the default orientation.
  ///
  /// This is the orientation that results in the text shown on the
  /// [Graphical OLED Display: 128x64, 1.3", White, SPI][3760]
  /// from Pololu being the same orientation as the pin labels.
  ///
  /// [3760]: https://www.pololu.com/product/3760
extern void noRotate();

  /// @brief Sets the contrast (i.e. brightness) of the OLED.
  ///
  /// @param contrast A number between 0 (darkest, but still visible) and
  /// 255 (brightest, default).
extern void setContrast(uint8_t contrast);

  /// @brief Configures this library to use a layout with
  /// 21 columns and 8 rows of text.
extern void setLayout21x8();

  /// @brief Configures this library to use a layout with 21 columns and
  /// 8 rows of text, XORed with a graphics buffer.
  ///
  /// @param graphics A pointer to a 1024-byte graphics buffer.
extern void setLayout21x8WithGraphics(const uint8_t * graphics);

  //////// Display routines


  /// @brief Writes all of the text/graphics to the OLED.
  ///
  /// This also turns on auto display mode, undoing the effect of
  /// noAutoDisplay().
  ///
  /// Note that this function does not always write to the entire display:
  /// for text-only modes, it will normally only write to the portion of the
  /// screen containing text.
extern void display();

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
extern void displayPartial(uint8_t x, uint8_t y, uint8_t width);


  //////// Text

  /// @brief Turns off auto display mode.
  ///
  /// This causes the clear(), write(), and print() functions to not perform
  /// any I/O or write any data to the OLED.
  ///
  /// Calling display() will write to the display and turn auto display mode
  /// on again.
extern void noAutoDisplay();

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
extern uint8_t * getLinePointer(uint8_t line);

  /// @brief Changes the location of the text cursor.
  ///
  /// This function changes the text cursor, which is the location of the text
  /// that will be overwritten by the next call to write() or print().
  ///
  /// @param x The column number (0 means left-most column).
  /// @param y The row number (0 means top row).
extern void gotoXY(uint8_t x, uint8_t y);

  /// @brief Gets the X coordinate of the text cursor.
extern uint8_t getX();

  /// @brief Gets the Y coordinate of the text cursor.
extern uint8_t getY();

  /// @brief Moves all the text up one row. (Does not change the cursor position.)
extern void scrollDisplayUp();

  /// @brief Clears the text and resets the text cursor to the upper left.
  ///
  /// After calling this function, the text buffer will consist entirely of
  /// space characters.
  ///
  /// By default, this function also calls display() to write these changes
  /// to the OLED, but noAutoDisplay() disables that behavior.
extern void clear();

  /// @brief Writes a single character of text.
  ///
  /// This function writes to the text buffer
  /// at the location specified by the text cursor (see gotoXY()), and moves
  /// the text cursor to the position immediately to the right.
  ///
  /// By default, this function also calls displayPartial() to write these
  /// changes to the OLED, but noAutoDisplay() disables that behavior.
  ///
  /// There is no limit to how much text you can pass to this function, but
  /// the text will be discarded when you reach the end of the current line.
  ///
  /// To advance to the next line, use gotoXY().  The newline and carriage
  /// return characters do *not* have any special effect on the text cursor
  /// position like they might have in a terminal emulator.
  ///
extern int OLED_write(char c);

  /// @brief Defines a custom character from RAM.
  /// @param picture A pointer to the character dot pattern, in RAM.
  /// @param number A character code between 0 and 7.
extern void loadCustomCharacterFromRam(const uint8_t * picture, uint8_t number);

  /// @brief Defines a custom character.
  /// @param picture A pointer to the character dot pattern, in program space.
  /// @param number A character code between 0 and 7.
extern void loadCustomCharacter(const uint8_t * picture, uint8_t number);

