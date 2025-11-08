// Copyright (C) Pololu Corporation.  See www.pololu.com for details.

#include <avr/io.h>
#include <util/delay.h>

#define SETBIT(reg, bit) ((reg) |=  _BV(bit))
#define CLRBIT(reg, bit) ((reg) &= ~_BV(bit))

// Pin assignments for the OLED:
// clk  : PD3
// reset: PD2
// MOSI : PD5
// dc   : PB0

  void sh1106InitPins()
  {
    CLRBIT(PORTD, PD3);     // low level on clk pin
    SETBIT(DDRD,  DDD3);    // make clk pin an output

    // disables USB interrupts because the USB interrupts use some of the OLED pins
    UDIEN = 0;
    UENUM = 0;
    UEIENX = 0;
  }

  void sh1106Reset()  // PD2 is the reset pin for the OLED
  {
    // do a reset pulse
    CLRBIT(PORTD, PD2);       // prepare a low level on reset pin
    SETBIT(DDRD, DDD2);       // make reset pin an output
    _delay_us(10);            // 10 µs negative pulse
    SETBIT(PORTD, PD2);       // back to idle high level
    _delay_us(10);            // be sure to wait a little before issueing commands
  }

  void sh1106TransferStart()
  {
    CLRBIT(PORTD, PD5);       // low level on "MOSI" pin
    SETBIT(DDRD,  DDD5);      // make "MOSI" pin an output
  }

  void sh1106TransferEnd() {}

  void sh1106CommandMode()
  {
    CLRBIT(PORTB, PB0);     // low level on dc pin
    SETBIT(DDRB,  DDB0);    // make dc pin an output
  }

  void sh1106DataMode()
  {
    SETBIT(PORTB, PB0);     // high level on dc pin
    SETBIT(DDRB,  DDB0);    // make dc pin an output
  }

  void sh1106Write(uint8_t d)
  {
// This asm inside the macro below is an optimized version of
//   FastGPIO::Pin<mosPin>::setOutputValue(d >> b & 1);
// It prevents the compiler from using slow branches to implement the
// conditional logic, and also ensures that the writing speed doesn't depend
// on the data.
#define _P3PP_OLED_SEND_BIT(d,b) \
  CLRBIT(PORTD, PD3); /* clock low */ \
  __asm__ volatile( \
    "sbrc %0, %1\n" "sbi 0x0B, 5\n" \
    "sbrs %0, %1\n" "cbi 0x0B, 5\n" \
    : : \
    "r" (d), \
    "I" (b)); \
  SETBIT(PORTD, PD3); /* clock high */

    _P3PP_OLED_SEND_BIT(d,7);
    _P3PP_OLED_SEND_BIT(d,6);
    _P3PP_OLED_SEND_BIT(d,5);
    _P3PP_OLED_SEND_BIT(d,4);
    _P3PP_OLED_SEND_BIT(d,3);
    _P3PP_OLED_SEND_BIT(d,2);
    _P3PP_OLED_SEND_BIT(d,1);
    _P3PP_OLED_SEND_BIT(d,0);
  }

