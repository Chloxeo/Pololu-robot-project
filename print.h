/**
 * @file print.h
 * 
 * @brief redirection of stdin and stdout on OLED or USB
 * 
 * @author Arnaud Dion arnaud.dion@isae-supaero.fr
 * 
*/

#ifndef PRINT_H
#define PRINT_H

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "OLED.h"
#include "SH1106.h"
#include "m_usb.h"

/**
 * Documentation on the lib used :
 * https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
 */

/**
 * @brief Macro to define which redirection is used : USB or OLED
 * 
 */
//#define USB_STREAM 

#ifdef USB_STREAM
/**
 * @brief Wrapper function for FDEV_SETUP_STREAM for usb transmit
 *
 * @param c character to send
 * @param FILE a stream
 *
 * @return 0 if character was sent
 */
int usb_tx_stream(char c, FILE *stream);
/**
 * @brief Wrapper function for FDEV_SETUP_STREAM for usb receive
 *        Wait for a character to be received
 *
 * @param FILE a stream
 *
 * @return the character received
 */
int usb_rx_stream(FILE *stream);
/**
 * @brief Initialisation of the link and redirection of stdout to usb
 */
void USB_init(void);
#else
/**
 * @brief Wrapper function for FDEV_SETUP_STREAM for OLED display
 *
 * @param c character to send
 * @param FILE a stream
 *
 * @return 0 if character was sent
 */
int OLED_write_stream(char c, FILE *stream);
/**
 * @brief Initialisation of the OLED and redirection of stdout to usb
 */
void OLED_init(void);
#endif
#endif
