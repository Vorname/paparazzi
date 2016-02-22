#include <modules/ws2801/ws2801.h>

#include <led.h>

enum RGB {
	RGB_START = 0,
	B=0,
	G,
	R,
	RGB_END
};

enum State {
	clockIdle,
	clockStart,
	clockHigh,
	clockLow
};

enum Logic {
	low = 0,
	high
};

static uint8_t colorBuffer[WS2801_LEDS][RGB_END];
static uint8_t currLed, currColor, currBit, clockState, startTime;

uint8_t ws2801_ledNr;
uint8_t ws2801_red;
uint8_t ws2801_green;
uint8_t ws2801_blue;

static void setClock(enum Logic value) {
	if(value)
		LED_OFF(6);
	else
		LED_ON(6);
}

static void setData(enum Logic value) {
	if(value)
		LED_OFF(7);
	else
		LED_ON(7);
}

static void start(void) {
	clockState = clockStart;
	startTime = 150;
}

void ws2801_init(void) {
	currLed = 0;
	currColor = 0;
	currBit = 8;
	clockState = clockIdle;
	ws2801_deactivateLeds();
}

void ws2801_event(void) {
	switch(clockState) {
		case(clockStart):
			setClock(low);
			if(startTime--==0) {
				setData(colorBuffer[currLed][currColor] & (1 << --currBit) );
				clockState = clockHigh;
			}
			break;
		case(clockHigh):
			setClock(high);
			clockState = clockLow; 
			if( currBit == 0 ) {
				currColor++;
				currBit=8;
			}

			if(currColor==RGB_END) {
				currLed++;
				currColor = RGB_START;
			}
			if(currLed == WS2801_LEDS) {
				currLed = 0;
				clockState = clockIdle;
			}

			break;
		case(clockLow): 
			setClock(low);
			setData(colorBuffer[currLed][currColor] & (1 << --currBit) );
			clockState = clockHigh;
			break;
		default: 
			setClock(high);
			setData(low);
			break;
	}

}

enum Errors ws2801_setColor(uint8_t led, uint8_t r, uint8_t g, uint8_t b) {
	if(led >= WS2801_LEDS)
		return NO_SUCH_LED;
	if(clockState != clockIdle)
		return BUSY;
	colorBuffer[led][R] = r;
	colorBuffer[led][G] = g;
	colorBuffer[led][B] = b;
	start();
	return SUCCESS;
}

int ws2801_activateLeds(void){
	colorBuffer[0][G]=255;
	colorBuffer[1][G]=255;
	colorBuffer[2][R]=255;
	colorBuffer[3][R]=255;
	start();
	return 0;
}

int ws2801_deactivateLeds(void){
	for(unsigned int i=0;i<WS2801_LEDS;i++)
		for(unsigned int j=0;j<RGB_END;j++)
			colorBuffer[i][j] = 0;
	start();
	return 0;
}


void ws2801_update(uint8_t ledNr) {
	ws2801_ledNr=ledNr;
	ws2801_setColor(ledNr, ws2801_red, ws2801_green, ws2801_blue);
}
