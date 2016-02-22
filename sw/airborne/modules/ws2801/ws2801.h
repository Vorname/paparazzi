#pragma once

#include <stdint.h>

#define WS2801_LEDS 4

enum Errors {
	SUCCESS,
	NO_SUCH_LED,
	BUSY
};

extern uint8_t ws2801_ledNr;
extern uint8_t ws2801_red;
extern uint8_t ws2801_green;
extern uint8_t ws2801_blue;

extern enum Errors ws2801_setColor(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
int ws2801_activateLeds(void);
int ws2801_deactivateLeds(void);
extern void ws2801_init(void);
extern void ws2801_event(void);
extern void ws2801_update(uint8_t ledNr);
