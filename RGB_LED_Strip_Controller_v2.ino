/*
  -----------------------------------------------------------------------------------
                 LED_Strip Controller for Arduino UNO
                     v. 2.1 25 October 2022 D.L. Poole
  -----------------------------------------------------------------------------------

  This controller for PWM-controlled RGB LED strips allows selection of illumination
  with an NEC code IR remote using either a Red/Green/Blue or Hue/Saturation/Value
  Hue/Saturation/Brightness (HSB) color model.  The hues follow the FastLED library 
  CHSV object's hsv2rgb_rainbow color space rather than hsv2rgb_spectrum in order to
  produce a brighter and wider range of yellows.

  The 8-bit PWM resolution of analogwrite() is sufficient resolution for hue
  and saturation, but limits the available dimming range without producing hue shifts.
  This rev2 enables the timer overflow interrupts, counts and numbers PWM cycles as
  cycles 0-7. The resulting refresh time of 16 mS is sufficiently fast to avoid even
  peripheral (vision) flicker.  All eight PWM cycles are used at 100% at maximum
  maximum brightness. Gross dimming is governed by incrementing/decrementing the integer
  pwmCyclesON, producing eight 12.5% steps, beyond which fine dimming continues in
  PWM steps of a single 1:8 duty cycle down to blackout.

  Remote Key functions are as follows:
  HUE:    Select or modify hue, moving clockwise or counterclockwise along color circle
  BRIGHT: Raise total red+green+blue value without change of hue
  DIM:    Lower total red+green+blue value without change of hue
  DESAT:  Desaturate the selected hue towards white (center of the color circle)
  RESAT:    Resaturate towards a primary or secondary color (red==0 || green==0 || blue==0
  RESET:  Set a pure, saturated blue at 100% brightness. blue = 255
  VARY:   Start varying HUE around color circle or slows ongoing rate of variation by half
  SAT:    Return to the selected,fully saturated HSB, cancels variable hue
  RED+:   Increase the Red component and changes HSB accordingly
  GREEN+: Increase the Green component  and adjust HSB accordingly
  BLUE+:  Increase the Blue component and adjust HSB accordingly
  RED-:   Decrease the Red component and adjust HSB accordingly
  GREEN-: Decrease the Green component and adjust HSB accordingly
  BLUE-:  Decrease the Blue component and adjust HSB accordingly
  WARM WHITE:    Simulate a tungsten bulb at max brightness
  BRIGHT WHITE:  Simulate a halogen bulb at max brightness
  DAYLIGHT:      Simulate a daylight source at max brightness

  A single keypress makes a double step forward or single step back in HUE or BRIGHTNESS;
  pressing then holding a key for a half-second repeats the selected key function ten
  times per second.  When an edit has reached a practical limit and further keypresses
  accomplish nothing, a blink occurs instead.

  Edits to either HSB or RGB values are immediately transformed to the other model and
  output to the LEDS so that one can freely move between models. RGB edits to full ON (white)
  or full OFF leave underflow RGB values values alone, so the can brighten or resaturate
  a dimmed white or white towards its starting hue.

  ---------------------------------Color Models-----------------------------------------

  RGB and HSB values are maintained as floats for ease of interpolation. They are rounded to
  the nearest integer before a write to the PWM outputs, so they extinguish below
  bright = 1/254.5 = 0.0039 and turn full ON above 254.50  Edits to red, green, blue, and
  brightness are performed in progressively larger deltas rather than increments so as to
  produce visually similar steps at both ends of the range.

  The HSB color space lies along a line of 255 hues where 0=red, 32=orange, 64=green,
  128=cyan, 128=blue, 192 is purple, and 255 is also red. One primary always zero, as 
  The addition of any third primary will desaturate a pure hue towards white at sat=0 where
  sat=1.0 for all pure hues.

  Brightness is represented by the floating point variable 0.0 <= bright <=1.0 which is
  here defined as (red+green+blue)/255/3.  Brightness is thus a constant 1/3 along the
  pure hue range. Reducing bright dims the display and extinguishes it as bright approaches
  zero. Brightness will max out the two primaries of a secondary color at bright = 2/3 and
  the three primaries of a gray at bright = 1.0

  The HSV Spectrum hue space, of linear ramps on hue is not as good visually; what little
  yellow there is appears dim, and at lower brightnesses, almost brownish. The rainbow hue
  space is implemented here with ramp functions that implement the same intercepts rather 
  than calls to the FastLed library, which lacks a method for converting potentially edited
  rgb triplets back to HSV anyway. For a fuller discussion, see
  https://github.com/FastLED/FastLED/wiki/Pixel-reference

*/

#include <IRLibDecodeBase.h>
#include <IRLib_P01_NEC.h>
#include <IRLib_P07_NECx.h>
#include <IRLibCombo.h>
#include <IRLibRecvPCI.h>
#include <IRLibFreq.h>
#include <EEPROM.h>

// -----------------------------Hardware Inputs/Outputs:-------------------------------
// D2:       Input from Vishay TSOP34438 IR Receiver or equivalent
// 5 volt Gate drive to N-channel MOSFET driver.  I.R. IRLIZ44NPBF for example, should
// be good to about 8A at 100% ON without a heat sink or higher voltage gate driver,
// however YOYO and YRMV
// PWM 9:    Red PWM Pin; High = ON
// PWM 10:   Green PWM Pin; High = ON
// PWM 11:   Blue PWM Pin; High = ON

//----------------------------------IR Remote Rcvr------------------------------------
// This uses the IRLib2 library downloaded from https://github.com/cyborg5/IRLib2.
// With the Vishay TSOP34438 IR Receiver, NEC decoding was most reliable using the
// IRrecvPCI receiver with the enableAutoResume method and markExcess = 0  The PCI
// receiver also avoids conflict with the PWM on pin D11 for TIMER2.

IRrecvPCI MyReceiver(2);              //create the receiver on pin 2/INT0
uint16_t myBuffer[RECV_BUF_LENGTH];   //declare our own raw buffer
IRdecode MyDecoder;                   //create the decoder

//--------------------------------Debugging Aids----------------------------------------

#define  DUMP_IR_CODES 0                      //NZ = dump IR decodes to serial monitor
#define  DUMP_VARIABLES 0                     //NZ = dump HSV and RGB after changes

// -------------------------------Defined Constants-------------------------------------
const float MAX_RGB = 255.;                   //maximum RGB value
const float MIN_RGB = 1.0;                    //mininum RGB value
const float MIN_BRIGHT = MIN_RGB / 3.0 / MAX_RGB;
const int  RED_PIN  = 9;                      //PWM pin for red
const int GREEN_PIN  = 10;                    //PWM pin for green
const int BLUE_PIN   = 11;                    //PWM pin for blue
const unsigned int MIN_STEP_TIME = 100;       //mS per step
const unsigned int  MAX_STEP_TIME = 10000;    //mS per step
const int  HUE_STEPS = 400;                   //number of steps along hue circle
//const int  BRIGHT_STEPS = 100;                 //number of brightness steps
const int  RGB_STEPS = 60;                    //number of RGB increment/decrement steps

//---------------------------IR Keypad Definitions and Key Codes------------------------
// These consts assign the remote commands to key codes on the Adafruit Mini Remote
// Control (Product ID 389)  Modify this table for other remotes or codes
// A keypad overlay for this key layout and the Mini Remote is available on GitHub

const unsigned int IR_DECODE_TYPE =  1;             //NEC for the Adafruit Remote
const unsigned int KEYCODE_MASK = 0xffff;           //low bits of key value id keys
const unsigned int INIT  = 0x30cf;                  //select Hue Saturation Brightness mode
const unsigned int VAR_HUE = 0xb04f;                //start or slow varying : {
const unsigned int NEXT_HUE = 0x609f;               //one hue step forward
const unsigned int PREV_HUE = 0x20df;               //one hue step back
const unsigned int DESATURATE =  0xa05f;            //desaturate one step
const unsigned int SATURATE = 0x50af;               //saturate one step
const unsigned int PURE   =   0x708f;               //pure hue, saturation=1
const unsigned int DIM   =    0x906f;               //dim
const unsigned int BRIGHTEN = 0x807f;               //brighten
const unsigned int INCR_RED = 0x08f7;               //increase red
const unsigned int INCR_GREEN = 0x8877;             //increase green
const unsigned int INCR_BLUE   = 0x48b7;            //increase blue
const unsigned int DECR_RED  =  0x28d7;             //decrease red
const unsigned int DECR_GREEN  = 0xa857;            //decrease green
const unsigned int DECR_BLUE =  0x6897;             //decrease blue
const unsigned int WARM_WHITE = 0x18e7;             //warm white
const unsigned int BRIGHT_WHITE = 0x9867;           //bright white
const unsigned int DAYLIGHT  =  0x58a7;             //daylight

// Structure for storing HSB and variable color state in NVRAM

struct state {
  float  hue = 0.0;                        //hue = 0 = Blue-->Violet = 1
  float  sat = 1.0;                        //saturation 0 = white; 1 = pure hue;
  float  bright = 1.0 / 3.0;               //bright = red+green+blue/3*MAX_RGB
  int    pwmCyclesON = 7;                  //gross dimming (PWM cycles)
  bool   variableHue = false;              //if hue varying
  unsigned long hueStepTime;               //and variation rate
};


//-----------------------------Globals ---------------------------------------------

state nvram;                               //name structure in NVRAM
unsigned long hueStepTime;                 //time between variable hue steps
unsigned long hueStepTimer;                //timer for hueStepTime
float hue;
float sat;
float bright;                              //0-255
float red;                                 //currently displayed red
float green;
float blue;                                //currently displayed blue
unsigned int keyValue;                    //key code from IR remote
unsigned int lastKey;                     //last key pressed for repeating
unsigned int repeatCount;                 //repeat frame counter
unsigned long keyTimer;                   //repeat key timeout
bool variableHue;
unsigned int pwmCount;                    //PWM cycle count at which to use PWM
unsigned int pwmCounter1;                 //PWM cycle count for PWM pins 9,10
unsigned int pwmCounter2;                 //PWM cycle count for PWM pin 11
int pwmCyclesON;                          //highest PWM cycle number 0-7

//---------------------------------setup()--------------------------------------------
void setup()  {
  Serial.begin(115200);                   //for debugging purposes
  Serial.println(__FILE__);               //send file and version to show what sketch is loaded

  // Start IR receiver

  MyReceiver.enableAutoResume(myBuffer);  //best option for NEC code
  MyReceiver.markExcess = 0;              //best option for Vishay TSOP34438
  MyReceiver.enableIRIn();                //start receiving

  pinMode(RED_PIN, OUTPUT);               //PWM out for Red LEDs
  pinMode(GREEN_PIN, OUTPUT);             //PWM out for Green LEDs
  pinMode(BLUE_PIN, OUTPUT);              //PWM out for Blue LEDs

  pinMode(8, OUTPUT);

  EEPROM.get(0, nvram);                   //get the last settings saved
  hue = nvram.hue;                        //init the current HSB and state to last saved
  sat = nvram.sat;
  bright = nvram.bright;
  pwmCyclesON = nvram.pwmCyclesON;
  variableHue = nvram.variableHue;
  hueStepTime = nvram.hueStepTime;

  HSBtoRGB(hue, sat, bright, red, green, blue);   //convert stored HSB to RGB
  PrintVariables();

  TCCR2A = 0x81;              //8-bit PWM on OC2A for blue
  TCCR2B = 0x04;              //8-bit PWM on OC2A, clock/64

  TCCR1A = 0xa1;              //8-bit PWM on OC1A for red
  TCCR1B = 0x03;              //8-bit PWM on OC1B for green, clock/64

  //-----------Enable timer overflow interrupts for counting PWM cycles-------------
  // The interrupts also transfer red, green, and blue to the timer output compare
  // registers
  cli();
  TIMSK1 |= (1 << TOIE1);
  TIMSK2 |= (1 << TOIE2);
  sei();
}

void loop() {       //"You can check out any time you like, but you can never leave"
  //-----------------------Handle Remote Commands-------------------------------------
  // The NEC protocol sends a frame for the keypress followed by a first repeat
  // (0xffffffff) frame within 40mS unless the key is released first. This conveniently
  // provides an end-of-frame interrupt to the IRrecvPCI receiver for the keypress
  // frame which repeat must be ignored. If the key is held, a second repeat frame is
  // sent 108mS later, making a single keypress somewhat difficult to execute; thus,
  // five consecutive repeat frames are required to begin repeating a function. When
  // the key is released, the saved key is timed out.

  if (millis() - keyTimer > 500) {              //if idle 500 ms
    repeatCount = 0;                            //reset the repeat count
    lastKey = 0;                                //and reset the saved keycode
  }
  if (MyReceiver.getResults()) {                //if a frame is pending in buffer
    MyDecoder.decode();                         //decode it

#if DUMP_IR_CODES == 1
    MyDecoder.dumpResults();                    //dump raw decode for debug or test
#endif

    MyReceiver.enableIRIn();                    //restart the receiver for next key or repeat
    if (MyDecoder.value == 0xffffffff) {        //if a repeat frame
      keyTimer = millis();                      //restart the idle timer
      if (repeatCount++ > 5) {                  //if more than 5 consecutive repeat frames
        MyDecoder.value = lastKey;              //by repeating the last key
      }
    }
    if (MyDecoder.protocolNum == 1 && MyDecoder.value != 0x00000000 && MyDecoder.value != 0xffffffff ) {
      lastKey = MyDecoder.value & 0xffff;       //save key code in case of repeat(s)
      keyTimer = millis();                      //restart idle timer

      switch (MyDecoder.value & 0xffff) {       //handle keypad

        //-----------Re-init to next of the Primary or Secndary colors------------------
        case INIT:
          if (hue >= 192.) hue = 0.;         //if purple then red
          else if (hue >= 160.) hue = 192.;  //if blue then purple
          else if (hue >= 128.) hue = 160.;  //if if cyan then blue
          else if (hue >= 96.) hue = 128.;   //if if green then cyan
          else if (hue >= 64.) hue = 96.;    //if yellow then green
          else if (hue >= 0.) hue = 64.;     //if red then yellow
          sat = 1.0;
          bright = 1.0;                       //RGB is 100%
          pwmCyclesON = 11;                    //max out the LEDs
          variableHue = false;
          hueStepTimer = MIN_STEP_TIME;
          delay(100);     //one step at a time
          break;
        //------------------------Varying Hue----------------------------
        // Start varying hue at pre-existing hue, sat, brightness.  If a repeated
        // keypress, halve the rate at which it varies
        case VAR_HUE:
          if (variableHue) {                  //if already varying
            hueStepTime *= 2;                 //successive presses slow it x2
            blink();                          //to acknowledge the command
          }
          else {
            variableHue = true;               //start varying hue
          }
          break;
        //--------------------------- Next Hue-------------------------------
        case NEXT_HUE:
          hue += 2.;
          if (hue > 254.0) {                    // 0.0 <= hue <= 255.0
            hue = 0.0;
          }
          break;
        //---------------------------Previous Hue----------------------------
        case PREV_HUE:
          hue -= 1.;
          if (hue < 0. ) {
            hue = 254.0;
          }
          break;
        //------------------------------Desaturate----------------------
        case DESATURATE:
          sat -= .05;
          if (sat < .1) {
            sat = .0001;
            blink();
          }
          break;
        //-------------------------------(Re)Saturate------------------
        case SATURATE:
          sat += .05;
          if (sat > 1) {
            sat = .9999;
            blink();
          }
          break;
        //------------------------------Pure Color------------------
        // Starts from current hue
        case PURE:
          sat = 1.0;
          bright = 1.0;
          variableHue = false;
          break;
        case BRIGHTEN:
          if (bright == 0.) {
            bright = 0.004;
          }
          else {
            bright *= 1.414213;
          }
          if (bright > 1.) {
            bright = 1.;
            if (pwmCyclesON < 7) {
              pwmCyclesON++;
            }
            else {
              blink();
            }
          }
          break;
        case DIM:
          if (pwmCyclesON > 0) {
            pwmCyclesON--;
          }
          else {
            bright *= 0.707106;
            if (bright <= 0.004) {
              bright = 0;
              blink();
            }
          }
          break;
        case INCR_RED:
          Increment(red);
          break;
        case INCR_GREEN:
          Increment(green);
          break;
        case INCR_BLUE:
          Increment(blue);
          break;
        case DECR_RED:
          Decrement(red);
          break;
        case DECR_GREEN:
          Decrement(green);
          break;
        case DECR_BLUE:
          Decrement(blue);
          break;
        case WARM_WHITE:                           // 7 = Warm White 40W tungsten
          red = 0xff;
          green = 0x93;
          blue = 0x29;
          variableHue = false;
          RGBtoHSB(hue, sat, bright, red, green, blue);
          break;
        case BRIGHT_WHITE:                           // 8 = Bright White Halogen
          red = 0xff;
          green = 0xf1;
          blue = 0xaa;
          variableHue = false;
          RGBtoHSB(hue, sat, bright, red, green, blue);
          break;
        case DAYLIGHT:                                // 9 = Daylight
          red = 0xff;
          green = 0xf1;
          blue = 0xff;
          variableHue = false;
          RGBtoHSB(hue, sat, bright, red, green, blue);
        default: {
          }
      }

      HSBtoRGB(hue, sat, bright, red, green, blue);
#if DUMP_VARIABLES == 1
      PrintVariables();
#endif
      // ----------Save any change(s) to nvram structure in EEPROM.---------
      // The put function uses EEPROM.update() to perform the write,
      // so it does not rewrite the EEPROM if the structure in it didn't change.

      nvram.hue = hue;
      nvram.sat = sat;
      nvram.bright = bright;
      nvram.pwmCyclesON = pwmCyclesON;
      nvram.variableHue = variableHue;        //if hue varying
      nvram.hueStepTime = hueStepTime;        //and variation rate
      EEPROM.put(0, nvram);                   //save any changes
    }
  }

  //----------------- Vary Hue if enabled ----------------------------------
  if (millis() - hueStepTimer > 100 && variableHue == true) { //do slow animation
    hueStepTimer = millis();
    if (variableHue) {
      hue += 255. / HUE_STEPS;
      if (hue > 255.0) {
        hue = 0.0;
      }
      HSBtoRGB(hue, sat, bright, red, green, blue);
      //     DisplayRGB();
    }
  }   //end of variable color
}     //end of loop()

//------------------------------Convert HSB to RGB ---------------------------------
void HSBtoRGB(float hue, float sat, float & bright, float & red, float & green, float & blue) {
  // There are three steps to RGB generation:
  // 1. Generate a pure, unsaturated primary or secondary where 0. <= red,green,blue <=255.
  // and red | green | blue == 0. The ramps of the fastLED rainbow profile are piecewise
  // linear over 0. <= hue <=255. The sum red+green+blue is non-constant around yellow and cyan
  // to increase the yellow range and decrease the cyan range.
  // 2. Desaturate towards white at sat=0. to the extent the RGB LED gamut supports it without
  // a white emitter.
  // 3. Apply brightness to the desaturated color and return as the global red, green, and
  // blue.

  //RGB components of fully saturated hues
  float redPure;
  float greenPure;
  float bluePure;

  //RGB components of desaturated hues
  float redSat;
  float greenSat;
  float blueSat;

  // Validate the hue
  if (hue < 0.0) {
    hue = 0.0;
  }
  if (hue >= 255.0) {
    hue = 0.;
  }

  // Develop the RGB Components of a pure hue
  // Each section implements a piecewise linear segment of the FastLED "Rainbow" hue chart
  // 0. <= redPure, greenPure, bluePure <=255;  0. <= redPure + greenPure + bluePure <=340.
  if (hue < 32.) {   // 0. >= hue < 32.
    redPure = 255. - (255. - 170.) / 32.*hue;
    greenPure = (255. - 170.) * hue / 32.;
    bluePure = 0;
  }
  if (hue >= 32 && hue < 64.) {   // 32. >= hue < 64.
    redPure = 170.;
    greenPure = hue * 255. / 96.;
    bluePure = 0;
  }
  if (hue >= 64. && hue < 96.) {   // 64. >= hue < 96.
    redPure = 170. - 170. / 32.*(hue - 64.);
    greenPure = 170. + (hue - 64.) * 255. / 96.;
  }
  if (hue >= 96. && hue < 128.) {   // 96. >= hue < 128.
    redPure = 0.;
    greenPure = 255 - (hue - 96.) * (255. - 170.) / (128. - 96);
    bluePure = (hue - 96.) * 85. / (128. - 96.) ;
  }
  if (hue >= 128. && hue < 160.) {   // 128. >= hue < 160.
    redPure = 0.;
    greenPure = 170. - 170. / (160. - 128.) * (hue - 128.);
    bluePure = 85. + (255. - 85.) / (160. - 128.) * (hue - 128.);
  }
  if (hue >= 160.) {   // 160. >= hue <- 256.
    redPure = 255. / (256. - 160.) * (hue - 160.);
    greenPure = 0.;
    bluePure = 255. - 255. / (256. - 160.) * (hue - 160.);
  }

  // Desaturate towards white at sat==0
  if (redPure == 0.) {
    redSat = (1. - sat) * MAX_RGB;
    greenSat = sat * greenPure + (1. - sat) * MAX_RGB;
    blueSat = sat * bluePure + (1. - sat) * MAX_RGB;
  }
  else if (greenPure == 0.) {
    greenSat = (1. - sat) * MAX_RGB;
    redSat = sat * redPure + (1. - sat) * MAX_RGB;
    blueSat = sat * bluePure + (1. - sat) * MAX_RGB;
  }
  else if (bluePure == 0.) {
    blueSat = (1. - sat) * MAX_RGB;
    greenSat = sat * greenPure + (1. - sat) * MAX_RGB;
    redSat = sat * redPure + (1. - sat) * MAX_RGB;
  }

  // Apply brightness to the saturated hue as the global RGB
  // Validate any edge cases due to accumulated under/overflow
  float sumRGBSat = redSat + greenSat + blueSat;
  red = redSat * bright;
  green = greenSat *  bright;
  blue = blueSat * bright;
  if (red > 255.)red = 255.;
  if (green > 255.)green = 255.;
  if (blue > 255.)blue = 255.;
}
//-------------------------------Convert RGB to HSB ----------------------------------
void RGBtoHSB(float & hue, float & sat, float & bright, float red, float green, float blue) {
  // Inverse of HSBtoRGB.  If sat ==0 or bright==0, hue is undefined, so the hue passed
  // with the call is left unchanged, otherwise determine the original pure hue

  // Saturateds are in the call
  float redSat = red;
  float greenSat = green;
  float blueSat = blue;

  // RGB components of the parameters if fully saturated
  // Calculate them given the sat
  float redPure;
  float greenPure;
  float bluePure;

  if (red <= min(green, blue)) {
    sat = (1. - redSat / MAX_RGB);
    redPure = 0.;
    greenPure = (greenSat + 255.*sat - 255.) / sat;
    bluePure = (blueSat + 255.*sat - 255.) / sat;

  }
  else if (green <= min(red, blue)) {
    sat = (1. - greenSat / MAX_RGB);
    greenPure = 0.;
    bluePure = (blueSat + 255.*sat - 255.) / sat;
    redPure = (redSat + 255.*sat - 255.) / sat;
  }
  else if (blue <= min(red, green)) {
    sat = (1. - blueSat / MAX_RGB);
    bluePure = 0.;
    greenPure = (greenSat + 255.*sat - 255.) / sat;
    redPure = (redSat + 255.*sat - 255.) / sat;
  }

  // Reverse the piecewise interpolation to get hue from the pures
  // Eliminate the pure primaries first
  if (redPure == 0 & greenPure  == 0.) hue = 160.; //pure blue
  else if (greenPure == bluePure == 0.) hue = 0.; //pure red
  else if (bluePure == redPure == 0.) hue = 96.; //pure green

  if (bluePure <= (min(redPure, greenPure))) {
    if (redPure / greenPure >= 2.) {
      hue = 255.*32. / (255. - 170.) / (redPure / greenPure + 1.);
    }
    else if ((red / green) >= 1.) {
      hue = 64. / (redPure / (greenPure));
    }
    else {
      hue = 64. + (170. - 170.*(redPure / (greenPure))) / (256. / 96.*(redPure / (greenPure)) + 170. / 32.);
    }
  }
  else if (redPure < min(bluePure, greenPure)) {
    if (greenPure / (bluePure) > 2.) {
      hue = 96. + 255. / ((greenPure / (bluePure)) * 85. / (128. - 96.) + (255. - 170.) / (128. - 96.));
    }
    else {
      hue = 128. + (170. - 85. *greenPure / (bluePure)) / (170. / (160. - 128.) + greenPure / (bluePure) * (255. - 85.) / (160. - 128.));
    }
  }
  else if (greenPure < min(redPure, bluePure)) {
    hue = 160. + (256. - 160.) / (1. + (bluePure / redPure));
  }
  if (hue == 0.) hue = .1;   // preserve a bit of green to avoid a pure red lock
  if (hue >= 255) hue = .1;  // preserve a bit of blue to avoid a pure red lock
}


//-----------------blink 50mS to acknowledge command while at a limit--------------------
void blink() {
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 0;
  delay(50);
  OCR1A = 255;
  OCR1B = 255;
  OCR2A = 255;
  delay(50);
}

//-----------------Increase an RGB -----------------------------------------------------
// Update HSB for hue and saturation but preserve brightness
// Preserve RGBs to just below an overflow so as to preserve hue for reversal
// Divider is a fudge for similar steps at extremes of range
void Increment(float & rgb) {
  if (rgb < 1.0) {
    rgb = 1.0;
  }
  rgb /= (.90 - 0.3 * (1. - (rgb / 255.)));
  if (rgb >= 254.) {
    rgb = 254.;
    blink();
  }
  RGBtoHSB(hue, sat, bright, red, green, blue);
}

//-----------------Decrease an RGB ----------------------------------------------------
// Update HSB for hue and saturation but preserve brightness
// Preserve RGBs to just above zero so as to preserve hue for reversal
// Multiplier is a fudge for similar steps at extremes of range
void Decrement(float & rgb) {
  rgb *=  (.97 - 0.3 * (1. - (rgb / 255.)));
  if (rgb < 0.1) {
    rgb = 0.1;
    blink();
  }
  RGBtoHSB(hue, sat, bright, red, green, blue);
}

//-------------------------PWM Timer Interrupt Services----------------------
// The interrupts count PWM cycles add number them. Fine PWM and hue are controlled
// by the 8-bit float bright. Max brightness is comprised of all 100% PWM cycles.
// Gross dimming is performed by blanking progressive PWM cycles in reverse order
// per the int pwmCyclesOn. When cycle 0 is reached, Fine dimming continues by
// means of the float bright to control the remaining cycle in PWM.

ISR(TIMER1_OVF_vect) {
  pwmCounter1++;
  pwmCounter1 &= 0x0007;
  if (pwmCounter1 <= pwmCyclesON ) {
    //---------------PWM or 100% on Cycle-----------------------------
    OCR1A = byte(int(red));     //red 100% or PWM
    OCR1B = byte(int(green));   //green 100% or PWM
  }
  else {
    //----------------OFF Cycle------------------------
    OCR1A = 0;     //red OFF
    OCR1B = 0;     //green OFF
  }
}

ISR(TIMER2_OVF_vect) {
  pwmCounter2++;
  pwmCounter2 &= 0x0007;
  if (pwmCounter2 <= pwmCyclesON ) {
    //---------------100% or PWM Controlled Cycle-----------------------------------------------
    OCR2A = byte(int(blue));     //blue 100% or PWM
  }
  else {
    //-------------OFF Cycle-------------------------------
    OCR2A = 0;                  //blue OFF
  }
}

//-----------------------Diagnostic Saves reTyping--------------------------
void PrintVariables() {
  Serial.print(hue, 3);
  Serial.print("\t");
  Serial.print(sat, 3);
  Serial.print("\t");
  Serial.print(pwmCyclesON);
  Serial.print("\t");
  Serial.print(bright, 3);
  Serial.print("\t\t");
  Serial.print(red, 3);
  Serial.print("\t");
  Serial.print(green, 3);
  Serial.print("\t");
  Serial.print(blue, 3);
  Serial.print("\t");
  Serial.print(red + green + blue, 3);
  Serial.println("\t");
}
