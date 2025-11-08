#include "print.h"

#ifdef USB_STREAM

int usb_tx_stream(char c, FILE *stream)
{
	if (c=='\0') {
		m_usb_tx_push();
		return 0;
	}
	else {
	if (c=='\n') {
	 	m_usb_tx_char('\r');
	}

		return (int)m_usb_tx_char(c);
	}
}

int usb_rx_stream(FILE *stream)
{
	while (!m_usb_rx_available());
	return (int)m_usb_rx_char();
}

static FILE USB_stream = FDEV_SETUP_STREAM(usb_tx_stream, usb_rx_stream, _FDEV_SETUP_RW);

__attribute__ ((__constructor__))
void USB_init()
{
    m_usb_init();
    stdout = &USB_stream;
	stdin = &USB_stream;
}
#else

int OLED_write_stream(char c, FILE *stream)
{
  if (c=='\n') {
    gotoXY(0, getY()+1);
    return 0;
  }
  else
    return OLED_write(c);
}

static FILE OLED_stream = FDEV_SETUP_STREAM(OLED_write_stream, NULL, _FDEV_SETUP_RW);

__attribute__ ((__constructor__))
void OLED_init()
{
    init();
    stdout = &OLED_stream;
}
#endif