#include <Arduino.h>
#include "AiEsp32RotaryEncoder.h"
#include "AiEsp32RotaryEncoderNumberSelector.h"
#include <Adafruit_I2CDevice.h>	
#include <Adafruit_GFX.h>
#include <FastLED_NeoMatrix.h>	// FastLED_NeoMatrix example for single NeoPixel Shield. By Marc MERLIN <marc_soft@merlins.org> Contains code (c) Adafruit, license BSD
#include <FastLED.h>
#include <MIDI.h>  // Add Midi Library

//midi::SerialMIDI<SerialPort, _Settings>::SerialMIDI [mit SerialPort=HardwareSerial, _Settings=midi::DefaultSerialSettings]
HardwareSerial myHardwareSerial(0);
MIDI_CREATE_INSTANCE(HardwareSerial, myHardwareSerial, MIDI);

const static boolean DEBUG = true;

#define DATA_PIN_1          1 	// yulc channel 1
#define DATA_PIN_2          2 	// yulc channel 2
#define mw					22	// TODO: ausmerzen
#define mh					23	// TODO: ausmerzen
#define MATRIX_WIDTH       	22
#define MATRIX_HEIGHT      	23
#define DEFAULT_BRIGHTNESS	125
#define MATRIX_TYPE         HORIZONTAL_ZIGZAG_MATRIX
#define MATRIX_SIZE         (MATRIX_WIDTH * MATRIX_HEIGHT)
#define NUMMATRIX			(MATRIX_WIDTH * MATRIX_HEIGHT)	// TODO: ausmerzen
#define NUMPIXELS           MATRIX_SIZE // TODO: ausmerzen
#define COLOR_ORDER         RGB
#define CHIPSET             WS2812B
#define anz_LEDs			193 //fuer die stripe-git // git-board hat: 278
#define green2 				255	//byte green2;
#define center_x 			10	//byte center_x;
#define center_y 			10	//byte center_y;

FastLED_NeoMatrix* matrix;
CRGB leds[NUMMATRIX];
//=============================================

byte songID = 0; // 0 -> default loop
byte red2;
byte blue2;
int col1;
int col2;

volatile unsigned int millisToReduceCPUSpeed = 0;
volatile unsigned int millisCounterTimer = 0;	// wird von den progs fürs timing bzw. delay-ersatz verwendet
volatile unsigned int millisCounterForProgChange = 0;		// achtung!! -> kann nur bis 65.536 zaehlen!!
volatile unsigned int millisCounterForHalfSecond = 0;
volatile unsigned int millisCounterForSeconds = 0;
volatile unsigned int nextChangeMillis = 100000;		// start value = 10 sec
volatile boolean flag_processFastLED = false;
volatile boolean flag_switchToNextSongPart = false;
volatile boolean nextChangeMillisAlreadyCalculated = false;
volatile byte nextSongPart = 0;
volatile byte prog = 0;
volatile boolean encoderButtonPushedLEDsOFF = false;	// for rotary encoder button push
volatile boolean LEDsTurnedOff = false;	// übergeordnetes FLAG
unsigned int lastLEDchange = millis();
int ledState = LOW;             // ledState used to set the LED --TODO: nur test mit interner LED
int zaehler = 0;
int BRIGHTNESS	= DEFAULT_BRIGHTNESS; // 32 - Max is 255, 32 is a conservative value to not overload a USB power supply (500mA) for 12x12 pixels.

CRGBPalette16 currentPalette;
TBlendType    currentBlending;
//===============================================

/*
connecting Rotary encoder

Rotary encoder side    MICROCONTROLLER side  
-------------------    ---------------------------------------------------------------------
CLK (A pin)            any microcontroler intput pin with interrupt -> in this example pin 37
DT (B pin)             any microcontroler intput pin with interrupt -> in this example pin 36
SW (button pin)        any microcontroler intput pin with interrupt -> in this example pin 38
GND - to microcontroler GND
VCC                    microcontroler VCC (then set ROTARY_ENCODER_VCC_PIN -1) 
***OR in case VCC pin is not free you can cheat and connect:***
VCC                    any microcontroler output pin - but set also ROTARY_ENCODER_VCC_PIN 25 
                        in this example pin 25
*/
#define ROTARY_ENCODER_A_PIN 37
#define ROTARY_ENCODER_B_PIN 36
#define ROTARY_ENCODER_BUTTON_PIN 38
#define ROTARY_ENCODER_VCC_PIN -1 /* 27 put -1 of Rotary encoder Vcc is connected directly to 3,3V; else you can use declared output pin for powering rotary encoder */
#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder *rotaryEncoder = new AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoderNumberSelector numberSelector = AiEsp32RotaryEncoderNumberSelector();

//paramaters for button
unsigned int shortPressAfterMiliseconds = 50;   //how long short press shoud be. Do not set too low to avoid bouncing (false press events).
unsigned int longPressAfterMiliseconds = 1000;  //how long čong

void on_button_short_click() {
	if (encoderButtonPushedLEDsOFF) {
		encoderButtonPushedLEDsOFF = false;
	}
	else {
		encoderButtonPushedLEDsOFF = true;	// for rotary encoder button push
	}
	if (DEBUG) {
		Serial.print("encoderButtonPushedLEDsOFF: ");
		Serial.println(encoderButtonPushedLEDsOFF);
	}
} 

void on_button_long_click() {
	//... put some code here if you want
} 

void rotary_onButtonClick() {
  static unsigned long lastTimeButtonDown = 0;
  static bool wasButtonDown = false;
  bool isEncoderButtonDown = rotaryEncoder->isEncoderButtonDown();

  if (isEncoderButtonDown) {
    if (!wasButtonDown) {
      lastTimeButtonDown = millis();
    }
    wasButtonDown = true;	//else we wait since button is still down
    return;
  }
  if (wasButtonDown) {
    if (millis() - lastTimeButtonDown >= longPressAfterMiliseconds) {
      on_button_long_click();
    } 
	else if (millis() - lastTimeButtonDown >= shortPressAfterMiliseconds) {
      on_button_short_click();
    }
  }
  wasButtonDown = false;
}

void rotary_loop() {	
	int16_t encoderDelta = rotaryEncoder->encoderChanged();
	if (encoderDelta != 0) {
		BRIGHTNESS = numberSelector.getValue();
		FastLED.setBrightness(BRIGHTNESS);
  }
	rotary_onButtonClick();
} 
// Function required for interupts
void IRAM_ATTR readEncoderISR(){
	rotaryEncoder->readEncoder_ISR();
} 

//===============================================
// This could also be defined as matrix->color(255,0,0) but those defines
// are meant to work for adafruit_gfx backends that are lacking color()
#define LED_BLACK		0

#define LED_RED_VERYLOW 	(3 <<  11)
#define LED_RED_LOW 		(7 <<  11)
#define LED_RED_MEDIUM 		(15 << 11)
#define LED_RED_HIGH 		(31 << 11)

#define LED_GREEN_VERYLOW	(1 <<  5)   
#define LED_GREEN_LOW 		(15 << 5)  
#define LED_GREEN_MEDIUM 	(31 << 5)  
#define LED_GREEN_HIGH 		(63 << 5)  

#define LED_BLUE_VERYLOW	3
#define LED_BLUE_LOW 		7
#define LED_BLUE_MEDIUM 	15
#define LED_BLUE_HIGH 		31

#define LED_ORANGE_VERYLOW	(LED_RED_VERYLOW + LED_GREEN_VERYLOW)
#define LED_ORANGE_LOW		(LED_RED_LOW     + LED_GREEN_LOW)
#define LED_ORANGE_MEDIUM	(LED_RED_MEDIUM  + LED_GREEN_MEDIUM)
#define LED_ORANGE_HIGH		(LED_RED_HIGH    + LED_GREEN_HIGH)

#define LED_PURPLE_VERYLOW	(LED_RED_VERYLOW + LED_BLUE_VERYLOW)
#define LED_PURPLE_LOW		(LED_RED_LOW     + LED_BLUE_LOW)
#define LED_PURPLE_MEDIUM	(LED_RED_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_PURPLE_HIGH		(LED_RED_HIGH    + LED_BLUE_HIGH)

#define LED_CYAN_VERYLOW	(LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_CYAN_LOW		(LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_CYAN_MEDIUM		(LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_CYAN_HIGH		(LED_GREEN_HIGH    + LED_BLUE_HIGH)

#define LED_WHITE_VERYLOW	(LED_RED_VERYLOW + LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_WHITE_LOW		(LED_RED_LOW     + LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_WHITE_MEDIUM	(LED_RED_MEDIUM  + LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_WHITE_HIGH		(LED_RED_HIGH    + LED_GREEN_HIGH    + LED_BLUE_HIGH)

//=====================================================================
//=========== HELPER FUNCTIONS ========================================
//=====================================================================

int getRandomColorValue() {	// dies erzeugt einen random-farb-anteil rot, grün oder blau
    int farbZahl = random(1, 6);
    int farbe = 0;
    switch (farbZahl) {
    case 1:
        farbe = 5;	// 0 echtes schwarz vermeiden
        break;
    case 2:
        farbe = 63; 
        break;
    case 3:
        farbe = 127;
        break;
	case 4:
        farbe = 191;
        break;
	case 5:
        farbe = 255;
        break;
    }
    return farbe;
}

byte r = getRandomColorValue();
byte g = getRandomColorValue();
byte b = getRandomColorValue();

int getRandomColor() { // dies erzeugt einen random color wert für die indexed colors:
	int farbZahl = random(1, 7);
	int farbe = LED_BLACK;
	switch (farbZahl) {
	case 1:
		farbe = LED_WHITE_HIGH;
		break;
	case 2:
		farbe = LED_GREEN_HIGH;
		break;
	case 3:
		farbe = LED_BLUE_HIGH;
		break;
	case 4:
		farbe = LED_ORANGE_HIGH;
		break;
	case 5:
		farbe = LED_PURPLE_HIGH;
		break;
	case 6:
		farbe = LED_CYAN_HIGH;
		break;
	case 7:
		farbe = LED_RED_HIGH;
		break;
	}
	return farbe;
}

int getRandomColorIncludingBlack() {
	int farbZahl = random(1, 9);
	int farbe = LED_BLACK;
	switch (farbZahl) {
	case 1:
		farbe = LED_RED_HIGH;
		break;
	case 2:
		farbe = LED_GREEN_HIGH;
		break;
	case 3:
		farbe = LED_BLUE_HIGH;
		break;
	case 4:
		farbe = LED_ORANGE_HIGH;
		break;
	case 5:
		farbe = LED_PURPLE_HIGH;
		break;
	case 6:
		farbe = LED_CYAN_HIGH;
		break;
	case 7:
		farbe = LED_WHITE_HIGH;
		break;
	case 8:
		farbe = LED_BLACK;
		break;
	}
	return farbe;
}

void setDurationAndNextPart(unsigned int durationMillis, byte nextPart) {

	//--- standard-part um dauer und naechstes programm zu speichern ----
	if (!nextChangeMillisAlreadyCalculated) {
		FastLED.clear(true);
		nextChangeMillis = durationMillis;
		nextSongPart = nextPart;
		nextChangeMillisAlreadyCalculated = true;
	}
}

//==================================================================
//=========== FX programs ==========================================
//==================================================================

void progFullColors(unsigned int durationMillis, byte nextPart, unsigned int del) {

	//--- standard-part um dauer und naechstes programm zu speichern ----
	if (!nextChangeMillisAlreadyCalculated) {
		FastLED.clear(true);
		nextChangeMillis = durationMillis;
		nextSongPart = nextPart;
		nextChangeMillisAlreadyCalculated = true;
		millisCounterTimer = del; // workaround, damit beim ersten durchlauf immer sofort LEDs aktiviert werden und nicht erst nachdem del abgelaufen ist!
	}
	//---------------------------------------------------------------------

	if (millisCounterTimer >= del) {	// ersatz für delay()
		millisCounterTimer = 0;

		r = getRandomColorValue();
		g = getRandomColorValue();
		b = getRandomColorValue();

		if (r == 0 && g == 0 && b == 0) {
			r = getRandomColorValue();
			g = getRandomColorValue();
			b = getRandomColorValue();
		}

		if (!LEDsTurnedOff) {	// nur wenn LEDs an sind (for rotary encoder button push)
			// für LED-stripe-git einfach alle LEDs in loop manuell setzen:
			for (int i = 0; i < anz_LEDs; i++) {
				leds[i] = CRGB(r, g, b);
			}
			FastLED.show();
		}
	}
}

void progBlack(unsigned int durationMillis, byte nextPart) {

	//--- standard-part um dauer und naechstes programm zu speichern ----
	if (!nextChangeMillisAlreadyCalculated) {
		FastLED.clear(true);
		// workaround: die eigentlichen millis werden korrigiert auf die faktische dauer
		//nextChangeMillis = round((float)durationMillis / (float)1.0f);	// TODO: diesen wert eurieren und anpassen!!
		nextChangeMillis = durationMillis;
		nextSongPart = nextPart;
		nextChangeMillisAlreadyCalculated = true;
		//		Serial.println(nextChangeMillis);
	}
	//---------------------------------------------------------------------

	if (!LEDsTurnedOff) {	// nur wenn LEDs an sind (for rotary encoder button push)
		FastLED.show();
	}
}

// This function fills the palette with totally random colors.
void SetupTotallyRandomPalette()
{
	for (int i = 0; i < 16; i++) {
		currentPalette[i] = CHSV(random8(), 255, random8());
	}
}
// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void SetupBlackAndWhiteStripedPalette()
{
	// 'black out' all 16 palette entries...
	fill_solid(currentPalette, 16, CRGB::Black);
	// and set every fourth one to white.
	currentPalette[0] = CRGB::White;
	currentPalette[4] = CRGB::White;
	currentPalette[8] = CRGB::White;
	currentPalette[12] = CRGB::White;

}
// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette()
{
	CRGB purple = CHSV(HUE_PURPLE, 255, 255);
	CRGB green = CHSV(HUE_GREEN, 255, 255);
	CRGB black = CRGB::Black;

	currentPalette = CRGBPalette16(
		green, green, black, black,
		purple, purple, black, black,
		green, green, black, black,
		purple, purple, black, black);
}
// This example shows how to set up a static color palette
// which is stored in PROGMEM (flash), which is almost always more
// plentiful than RAM.  A static PROGMEM palette like this
// takes up 64 bytes of flash.
const TProgmemPalette16 myRedWhiteBluePalette_p =
{
	CRGB::Red,
	CRGB::Gray, // 'white' is too bright compared to red and blue
	CRGB::Blue,
	CRGB::Black,

	CRGB::Red,
	CRGB::Gray,
	CRGB::Blue,
	CRGB::Black,

	CRGB::Red,
	CRGB::Red,
	CRGB::Gray,
	CRGB::Gray,
	CRGB::Blue,
	CRGB::Blue,
	CRGB::Black,
	CRGB::Black
};

extern const TProgmemRGBPalette16 MatrixColors_p PROGMEM =
{
	0x001000, 0x003000, 0x005000, 0x007000,
	0x008000, 0x008000, 0x008000, 0x198d19,
	0x339933, 0x4da64d, 0x66b366, 0x80c080,
	0x99cc99, 0xb3d9b3, 0xcce6cc, 0xe6f2e6
};

void FillLEDsFromPaletteColors(uint8_t colorInd) {

	//0 rainbow slow
	//1 rainbow fast (ohne fades)
	//2 rainbow fast (mit fades)
	//3 lila/grün Fast mit fades
	//4 blau/lila/rot/orange mit fades Fast
	//5 white fast ohne fades
	//6 white fast mit fades
	//7 blau/weiss slow mit fades
	//8 blau/lila/rot/orange mit fades slow
	//9 weiss/blau/beige fast ohne fades (interessante farben)
	//10 weiss/blau/beige fast mit fades (interessante farben)
	//11 weiss/grün fast mit fades

	uint8_t brightness = 255;	// TODO: Achtung hier wird NICHT die allgemeine CONST für BRIGHTNESS genutzt (ggf. weil dann zu dunkel!?)

	for (int i = 0; i < anz_LEDs; i++) {
		if (!LEDsTurnedOff) leds[i] = ColorFromPalette(currentPalette, colorInd, brightness, currentBlending);
		colorInd += 3;	//3; / je hoeher dieser wert desto kuerzer sind die farbabschnitte (beeinflusst die subjektive geschwindigkeit)
	}
}
void progPalette(unsigned int durationMillis, uint8_t paletteID, byte nextPart) {

//0 rainbow slow
//1 rainbow fast (ohne fades)
//2 rainbow fast (mit fades)
//3 lila/grün Fast mit fades
//4 blau/lila/rot/orange mit fades Fast
//5 white fast ohne fades
//6 white fast mit fades
//7 blau/weiss slow mit fades
//8 blau/lila/rot/orange mit fades slow
//9 weiss/blau/beige fast ohne fades (interessante farben)
//10 weiss/blau/beige fast mit fades (interessante farben)
//11 weiss/grün fast mit fades

	//--- standard-part um dauer und naechstes programm zu speichern ----
	if (!nextChangeMillisAlreadyCalculated) {
		FastLED.clear(true);
		// workaround: die eigentlichen millis werden korrigiert auf die faktische dauer
		//nextChangeMillis = round((float)durationMillis / (float)5.85f);	// TODO: diesen wert eurieren und anpassen!!
		nextChangeMillis = durationMillis;
		nextSongPart = nextPart;
		nextChangeMillisAlreadyCalculated = true;

		// setup palette/Programm
		switch (paletteID) {
		case 0:
			currentPalette = RainbowColors_p;
			currentBlending = LINEARBLEND;
			break;
		case 1:
			currentPalette = RainbowStripeColors_p;   
			currentBlending = NOBLEND;
			break;
		case 2:
			currentPalette = RainbowStripeColors_p;   
			currentBlending = LINEARBLEND;
			break;
		case 3:
			SetupPurpleAndGreenPalette();   
			currentBlending = LINEARBLEND;
			break;
		case 4:
			SetupTotallyRandomPalette();   
			currentBlending = LINEARBLEND;
			break;
		case 5:
			SetupBlackAndWhiteStripedPalette();       
			currentBlending = NOBLEND;
			break;
		case 6:
			SetupBlackAndWhiteStripedPalette();       
			currentBlending = LINEARBLEND;
			break;
		case 7:
			currentPalette = CloudColors_p;           
			currentBlending = LINEARBLEND;
			break;
		case 8:
			currentPalette = PartyColors_p;           
			currentBlending = LINEARBLEND;
			break;
		case 9:
			currentPalette = myRedWhiteBluePalette_p; 
			currentBlending = NOBLEND;
			break;
		case 10:
			currentPalette = myRedWhiteBluePalette_p; 
			currentBlending = LINEARBLEND;
			break;
		case 11:
			currentPalette = MatrixColors_p;
			currentBlending = LINEARBLEND;
			break;
		}
	}
	//---------------------------------------------------------------------

	zaehler++;
	if (zaehler > 1000) zaehler = 0;	// der wert 1000 beinflusst  die geschwindigkeit
	FillLEDsFromPaletteColors(zaehler);	// hier wird schon intern LEDsTurnedOff abgefragt

	if (!LEDsTurnedOff) {	// nur wenn LEDs an sind (for rotary encoder button push)
		FastLED.show();
	}
}

extern const TProgmemRGBPalette16 matrixColors FL_PROGMEM =
{
	CRGB::LightGreen,
	CRGB::LightGreen,
	CRGB::LightGreen,
	CRGB::LightGreen,

	CRGB::Green,
	CRGB::Green,
	CRGB::Green,
	CRGB::Green,

	CRGB::LimeGreen,
	CRGB::LimeGreen,
	CRGB::LimeGreen,
	CRGB::LimeGreen,

	CRGB::DarkGreen,
	CRGB::DarkGreen,
	CRGB::DarkGreen,
	CRGB::DarkGreen
};

void switchToPart(byte part) {

	prog = part;
	nextChangeMillisAlreadyCalculated = false;	// bool wieder fuer naechstes programm freigeben
	millisCounterTimer = 0;
	millisCounterForProgChange = 0;
	zaehler = 0;	// globalen zaehler auf null
	//--- initializeValues ---
	flag_switchToNextSongPart = false;
}

void switchToSong(byte song) {
	songID = song;
	switchToPart(0);
}

// MidiDatenAuswerten is the function that will be called by the Midi Library
// when a Continuous Controller message is received.
// It will be passed bytes for Channel, Controller Number, and Value
// It checks if the controller number is within the 22 to 27 range
void MidiDatenAuswerten(byte channel, byte number, byte value) {
	// with midi byte 22 the song can be changed!
	if (number == 22 && value > 0) {
		switchToSong(value);
	}
	// with midi byte 23 the songpart can be changed!
	else if (number == 23 && value >= 0) {
		switchToPart(value);
	}
}
//====================================================

//#0
void defaultLoop()  {

 	switch (prog) { 

	case 0:
		progPalette(5000, 1, 5);
		break;

	case 5:
		progPalette(5000, 2, 10);
		break;

	case 10:
		progPalette(5000, 3, 15);
		break;

	case 15: // OK
		progPalette(5000, 4, 20);
		break;

	case 20: // OK
		progPalette(5000, 5, 25);
		break;

	case 25: // OK
		progPalette(5000, 6, 30);
		break;

	case 30:
		progPalette(5000, 7, 35);
		break;

	case 35: // OK
		progPalette(5000, 8, 40);
		break;

	case 40: // OK
		progPalette(5000, 9, 45);
		break;

	case 45: 
		progPalette(5000, 10, 50);
		break;

	case 50: 
		progPalette(5000, 11, 55);
		break;

	case 55:
		progPalette(5000, 12, 100);
		break;
		
	case 100:
		FastLED.clear();
		switchToSong(0);	// SongID 0 == DEFAULT loop
		break;
	}
}
//==============================================

//--- timer-interrupt every 2 ms so that fastLED can process uninterrupted (takes about ?? ms)
hw_timer_t *Timer0_Cfg = NULL;	// Timer Variable

#define INCREMENT	2	//5	 process FastLED-loops only every 5 ms
				//  => !!!! IMMER AUCH IN SETUP DEN CALLBACK AUFRUF ANPASSEN !!!!!

void IRAM_ATTR Timer0_ISR_callback() {	// TODO: timer könnte raus für eine exakte ms-genaue messung
    millisCounterTimer = millisCounterTimer + INCREMENT;	// wird von den progs fürs timing bzw. delay-ersatz verwendet
    millisCounterForHalfSecond = millisCounterForHalfSecond + INCREMENT;
	millisCounterForSeconds = millisCounterForSeconds + INCREMENT;
    millisCounterForProgChange = millisCounterForProgChange + INCREMENT;
	millisToReduceCPUSpeed = millisToReduceCPUSpeed + INCREMENT;

    flag_processFastLED = true;	// process FastLED-loops

    if (millisCounterForHalfSecond >= 500) {
		millisCounterForHalfSecond = 0;
        //HalfSecondHasPast = true;
    }

    if (millisCounterForSeconds >= 1000) {
        millisCounterForSeconds = 0;
        //OneSecondHasPast = true;
    }

	if (millisCounterForProgChange >= nextChangeMillis) flag_switchToNextSongPart = true;
}
//--------------------------------------------------

void setup() {
 
 	Serial.begin(115200);
	delay(500);	// Time for serial port to work?
	
	//--- Initialize rotary encoder
	rotaryEncoder->begin();
	rotaryEncoder->setup(readEncoderISR);
	rotaryEncoder->setAcceleration(0);
	rotaryEncoder->disableAcceleration();		
	numberSelector.attachEncoder(rotaryEncoder);
	numberSelector.setRange(255, 2, -1, false, 0);
    numberSelector.setValue(DEFAULT_BRIGHTNESS);
	//---------------------------------

	//--- interrupt-timer fuer callback
	//t1.begin(callback, 2ms); // !!!! IMMER AUCH define INCREMENT ANPASSEN !!!!!
	Timer0_Cfg = timerBegin(0, 80, true);	// divider/prescaler = 80
	// APB_CLK = 80 MHz = 80.000.000 Hz
	// 1 ms = TimerTicks * 80 (Prescaler) / 80.000.000 Hz
	// TimerTicks = 1000
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR_callback, true);
    timerAlarmWrite(Timer0_Cfg, 2000, true); // Interrupt alle 2 ms
    timerAlarmEnable(Timer0_Cfg);

	//---- MIDI ----------------
	MIDI.begin(10); // Initialize the Midi Library.
	// OMNI sets it to listen to all channels.. MIDI.begin(2) would set it
	// to respond to notes on channel 2 only.
	MIDI.setHandleControlChange(MidiDatenAuswerten); // This command tells the MIDI Library
	// the function you want to call when a Continuous Controller command
	// is received. In this case it's "MyCCFunction".

	//------- activate MOSFETs on YULC ----------------------------
  	pinMode(47, OUTPUT);      // switch on MOSFET for channel 1
  	digitalWrite(47, HIGH);   // switch on MOSFET for channel 1
  	pinMode(21, OUTPUT);    // switch on MOSFET for channel 2
  	digitalWrite(21, HIGH); // switch on MOSFET for channel 2

	//---- Define matrix width and height. --------
	matrix = new FastLED_NeoMatrix(leds, MATRIX_WIDTH, MATRIX_HEIGHT, NEO_MATRIX_TOP + NEO_MATRIX_RIGHT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG);

	//----- initialize LEDs ---------
	FastLED.addLeds<NEOPIXEL, DATA_PIN_1>(leds, NUMMATRIX).setCorrection(TypicalLEDStrip);
	//---use both yulc outputs:
	FastLED.addLeds<NEOPIXEL, DATA_PIN_2>(leds, NUMMATRIX).setCorrection(TypicalLEDStrip);

	//NEOPIXEL	//WS2812B
	matrix->begin();
	matrix->setBrightness(BRIGHTNESS);
	matrix->setTextWrap(false);

	//------ Setup Palette
	currentPalette = RainbowColors_p;
	currentBlending = LINEARBLEND;
	//-----------------
	
	switchToSong(0);  
	//switchToPart(0); // for testing
}
//====================================================

void loop() {

	rotary_loop();

	//--- midi immer checken, auch wenn voltage low, damit ja trotzdem marker LEDs setzen kann
	MIDI.read(); // Continuously check if Midi data has been received.
	//========================================

	if (flag_switchToNextSongPart) {
		switchToPart(nextSongPart);
	}

	//--- check if LEDs should be on ----
	if (encoderButtonPushedLEDsOFF) {
		LEDsTurnedOff = true;
		FastLED.clear();	// LEDs off durch rotary encoder button push
	}
	else {
		LEDsTurnedOff = false;
	}
	//=========================================

	//=== ab hier wird nur alle 2 ms ausgefuehrt ======
	if (flag_processFastLED) {	// LED loop only in certain time-slots to make ms-counter more accurate

		FastLED.setBrightness(BRIGHTNESS); // zur sicherheit for jedem loop neu auf default setzen. ggf. kann einzelner fx das überschreiben

		switch (songID) {
		case 0:
			defaultLoop();
			break;

		case 1:
			// put code here
			break;

		default:
			defaultLoop();
			break;
		}

		flag_processFastLED = false;
	}
}