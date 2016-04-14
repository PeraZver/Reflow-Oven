#ifndef _STEP_RESPONSE_H
#define _STEP_RESPONSE_H
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>

#include "Descriptors.h"
#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Drivers/USB/USB.h>

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)
#define bit_set(p,m) ((p) |= (1<<m))
#define bit_clear(p,m) ((p) &= ~(1<<m))
#define _ICR1 6250/* Counts 0.2 seconds */
/* Clamp x between l and h */
#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))
#define SIGN(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))
#define MAXTEMP 340
//#define MIN(x,y) (((x)<(y) ? (x) : (y)))

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);

void set_profile(void);
void write_profile(void);

#endif
