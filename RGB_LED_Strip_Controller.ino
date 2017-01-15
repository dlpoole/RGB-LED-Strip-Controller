/*
  -----------------------------------------------------------------------------------
                 LED_Strip Controller for Arduino UNO
                 5 January 2017 D.L. Poole
  -----------------------------------------------------------------------------------

  This controller for PWM-controlled RGB LED strips allows selection of illumination
  with an NEC code IR remote using either a Hue/Saturation/Brightness (HSB) or
  Red/Green/Blue (RGB) color model.

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

  A single keypress makes a single step in HUE or BRIGHTNESs, etc; pressing then holding
  a key for a half-second repeats the selected key function ten times per second.  When an
  edit has reached a practical limit and further keypresses accomplish nothing, a blink \
  occurs instead.

  Edits to either HSB or RGB values are immediately transformed to the other model and
  output to the LEDS so that one can freely move between models. RGB edits to full ON (white)
  or full OFF leave hue undefined by the RGB values alone, so the conversion to HSB 
  preserves the prior hue so that the user can resaturate a white back towards its starting hue.

  ---------------------------------Color Models-----------------------------------------

  RGB values are maintained as floats for ease of interpolation.  They are rounded to
  the nearest integer before a write to the PWM outputs, so they extinguish below
  bright = 1/254.5 = 0.004 and turn full ON above 254.5  Edits to red, green, blue, and
  brightness are performed in logarithmic steps rather than increments so as to produce
  visually similar steps at both ends of the range.  The last few steps to blackout
  pass through some 8-bit underflow steps on which the LEDs will not change.

  The HSB color space lies around a unit circle of "pure" primary (RGB) and secondary (CMY)
  hues along which hue is a floating point variable 0.0 <= hue <=1.0   Using a clock
  metaphor where pure blue lies at 12:00, red at 4:00, and green at 8:00, the three
  primaries are linearly interpolated two at a time so that magenta lies at 2:00, yellow
  at 6:00, and cyan at 10:00. Along the hue circle, the SUM of RGB values (and the corresponding
  PWM outputs, red + green + blue equals 255, with one primary always zero.

  The addition of any third primary desaturates a pure hue towards white, which lies at the
  figurative center of the clock and at which hue is undefined. The radial dimension of
  any point in HSB space is represented by the floating point variable sat where
  0.0 <= sat <=1.0  Thus, sat is 1.0 along the pure hue circle and outside it and zero at its
  center.

  Brightness is represented by the floating point variable 0.0 <= bright <=1.0 which is
  here defined as (red+green+blue)/255/3.  Brightness is thus a constant 1/3 along the
  pure hue circle. Reducing bright dims the display and extinguishes it as bright approaches
  zero. Brightness will max out the two primaries of a secondary color at bright = 2/3 and
  the three primaries of a gray at bright = 1

  Although the polar coordinates of HSB space suggests trigonometric definitions for the
  primaries around the hue circle, linear interpolation (triangular ramps) was found to
  produce a much smoother range of hues.
*/

#include <IRLibDecodeBase.h>
#include <IRLib_P01_NEC.h>
#include <IRLib_P07_NECx.h>
#include <IRLibCombo.h>
#include <IRLibRecvPCI.h>
#include <IRLibFreq.h>

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
#define  DEBUG 0                              //NZ = output HSB/RGB to serial monitor
#define  DUMP_IR_CODES 0                      //NZ = dump IR decodes to serial monitor

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
const int  BRIGHT_STEPS = 60;                 //number of brightness steps
const int  RGB_STEPS = 60;                    //number of RGB increment/decrement steps
const int  SAT_STEPS = 50;                    //number of saturation steps along radius

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

//-----------------------------Globals ---------------------------------------------
float hue;                                //hue = 0 = Blue-->Violet = 1
float sat;                                //saturation 0 = white; 1 = pure hue;
float bright;                             //bright = red+green+blue/3*MAX_RGB
unsigned long hueStepTime;                 //time between variable hue steps
unsigned long hueStepTimer;                //timer for hueStepTime
float red;                                //floats simplify hue interpolation
float green;
float blue;
unsigned int keyValue;                    //key code from IR remote
unsigned int lastKey;                     //last key pressed for repeating
unsigned int repeatCount;                 //repeat frame counter
unsigned long keyTimer;                   //repeat key timeout
bool variableHue;

//---------------------------------setup()--------------------------------------------
void setup()  {
  Serial.begin(115200);                   //for debugging purposes

  // Start IR receiver

  MyReceiver.enableAutoResume(myBuffer);  //best option for NEC code
  MyReceiver.markExcess = 0;              //best option for Vishay TSOP34438
  MyReceiver.enableIRIn();                //start receiving

  pinMode(RED_PIN, OUTPUT);               //PWM out for Red LEDs
  pinMode(GREEN_PIN, OUTPUT);             //PWM out for Green LEDs
  pinMode(BLUE_PIN, OUTPUT);              //PWM out for Blue LEDs


  hue = 0.0;                              //initialize to primary blue
  sat = 1.0;
  bright = 1.0 / 3.0;
  variableHue = false;
  hueStepTime = MIN_STEP_TIME;            //init variable hue rate to max
  HSBtoRGB(hue, sat, bright, red, green, blue);
  DisplayRGB();
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

      switch (MyDecoder.value & 0xffff) {       //handle keys

        //-----------Re-init to Saturated Blue and reset variable speed-------------
        case INIT:                             //re-init to saturated blue
          hue = 0.0;
          sat = 1.0;
          bright = 1.0 / 3.0;
          HSBtoRGB(hue, sat, bright, red, green, blue);
          hueStepTimer = MIN_STEP_TIME;
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
        // Hue changes are most easily perceived around yellow.  Halve the step
        // size between red and green as an alternative to shifting the ramps
        case NEXT_HUE:
          if (hue > 1.0 && hue < 2.0) {
            hue += 1. / HUE_STEPS * 2.0;
          }
          else {
            hue += 1. / HUE_STEPS;
          }
          if (hue > 1.0) {                    // 0.0 <= hue <= 1.0
            hue = 0.0;
          }
          HSBtoRGB(hue, sat, bright, red, green, blue);
          break;

        //---------------------------Previous Hue----------------------------
        // Hue changes are most easily perceived around yellow.  Halve the step
        // size between red and green as an alternative to shifting the ramps
        case PREV_HUE:
          if (hue > 1.0 && hue < 2.0) {
            hue -= 1. / HUE_STEPS * 2.0;
          }
          else {
            hue -= 1. / HUE_STEPS;
          }
          if (hue < 1 / HUE_STEPS) {
            hue = 1.0;
          }
          HSBtoRGB(hue, sat, bright, red, green, blue);
          break;

        //------------------------------Desaturate----------------------
        case DESATURATE:
          sat -= 1.0 / SAT_STEPS;
          if (sat < 0.0) {
            sat = 0.0;
            blink();                //if totally desaturated
          }
          HSBtoRGB(hue, sat, bright, red, green, blue);
          break;

        //-------------------------------(Re)Saturate------------------
        case SATURATE:
          sat += 1.0 / SAT_STEPS;
          if (sat > 1.0) {
            sat = 1.0;
            blink();                //if totally saturated
          }
          HSBtoRGB(hue, sat, bright, red, green, blue);
          break;

        //------------------------------Pure Color------------------
        // Starts from current hue
        case PURE:
          sat = 1.0;
          bright = 1.0;
          variableHue = false;
          HSBtoRGB(hue, sat, bright, red, green, blue);
          break;

        //------------------------------ Dim------------------------------
        // Logarithmically MAX_RGB to MIN_RGB over BRIGHT_STEPS
        case DIM:
          bright *= pow(10.0, (log10(1.0 / MAX_RGB) / BRIGHT_STEPS));
          HSBtoRGB(hue, sat, bright, red, green, blue);
          if (bright < MIN_BRIGHT) {
            bright = MIN_BRIGHT;
          }
          break;

        //------------------------------Brighten--------------------------
        // Logarithmically MIN_RGB to MAX_RGB over BRIGHT_STEPS
        case BRIGHTEN:
          if (bright < MIN_BRIGHT) {
            bright = MIN_BRIGHT;
          }
          bright /= pow(10.0, (log10(1.0 / MAX_RGB) / BRIGHT_STEPS));
          HSBtoRGB(hue, sat, bright, red, green, blue);
          if (red == MAX_RGB || green == MAX_RGB || blue == MAX_RGB) {
            blink();
          }
          break;

        //------------------------------RGB Edits---------------------------
        // If channel is OFF, start at minimum
        case INCR_RED:
          if (red < MIN_RGB) {               //kickstart if OFF
            red = MIN_RGB;
          }
          else
          {
            red /= pow(10.0, (log10(1.0 / MAX_RGB) / RGB_STEPS));
          }
          if (red >= MAX_RGB) {
            red = MAX_RGB;
            blink();
          }
          variableHue = false;
          RGBtoHSB(hue, sat, bright, red, green, blue);
          break;
        case INCR_GREEN:
          if (green < MIN_RGB) {
            green = MIN_RGB;
          }
          else
          {
            green /= pow(10.0, (log10(1.0 / MAX_RGB) / RGB_STEPS));
          }
          if (green >= MAX_RGB) {
            green = MAX_RGB;
            blink();
          }
          variableHue = false;
          RGBtoHSB(hue, sat, bright, red, green, blue);
          break;
        case INCR_BLUE:
          if (blue < MIN_RGB) {
            blue = MIN_RGB;
          }
          else
          {
            blue /= pow(10.0, (log10(1.0 / MAX_RGB) / RGB_STEPS));
          }
          if (blue >= MAX_RGB) {
            blue = MAX_RGB;
            blink();
          }
          variableHue = false;
          RGBtoHSB(hue, sat, bright, red, green, blue);
          break;
        case DECR_RED:
          red *= pow(10.0, (log10(1.0 / MAX_RGB) / RGB_STEPS));
          if (red < MIN_RGB) {
            red = 0.0;
            blink();
          }
          variableHue = false;
          RGBtoHSB(hue, sat, bright, red, green, blue);
          break;
        case DECR_GREEN:
          green *= pow(10.0, (log10(1.0 / MAX_RGB) / RGB_STEPS));
          if (green < MIN_RGB) {
            green = 0.0;
            blink();
          }
          variableHue = false;
          RGBtoHSB(hue, sat, bright, red, green, blue);
          break;
        case DECR_BLUE:
          blue *= pow(10.0, (log10(1.0 / MAX_RGB) / RGB_STEPS));
          if (blue < MIN_RGB) {
            blue = 0.0;
            blink();
          }
          variableHue = false;
          RGBtoHSB(hue, sat, bright, red, green, blue);
          break;

        //-------------------------Reference Colors--------------------------------------

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
      }   //end of key  handlers

      //------------------------------Display---------------------------------

      DisplayRGB();
#if DEBUG == 1
      Serial.print(hue, 3);
      Serial.print("\t");
      Serial.print(sat, 3);
      Serial.print("\t");
      Serial.print(bright, 3);
      Serial.print("\t\t");
      Serial.print(red, 3);
      Serial.print("\t");
      Serial.print(green, 3);
      Serial.print("\t");
      Serial.println(blue, 3);
#endif
    }   //end of valid key
  }     //end of command handler

  //----------------- Variable Color ----------------------------------
  // Hue changes are most easily perceived around yellow.  Halve the step
  // size between red and green as an alternative to shifting the ramps

  if (millis() - hueStepTimer > 100) {   //do slow animation
    hueStepTimer = millis();
    if (variableHue) {
      if (hue > 1.0 && hue < 2.0) {
        hue += 1. / HUE_STEPS * 2.0;
      }
      else {
        hue += 1. / HUE_STEPS;
      }
      if (hue > 1.0) {
        hue = 0.0;
      }
      HSBtoRGB(hue, sat, bright, red, green, blue);
      DisplayRGB();
    }
  }   //end of variable color
}     //end of loop()

//-------------Display red, green, blue as 24-bit RGB------------------------
// Cast and round red, green, and blue to bytes and write PWM values to
// LED strip

void DisplayRGB() {
  analogWrite(RED_PIN, byte(round(red)));
  analogWrite(GREEN_PIN, byte(round(green)));
  analogWrite(BLUE_PIN, byte(round(blue)));
}

//------------------------------Convert HSB to RGB ---------------------------------
void HSBtoRGB(float hue, float sat, float & bright, float & red, float & green, float & blue) {
  // Generate the pure hue.  Three linear ramps interpolate between primaries.
  // Hue range is from 0.0 = blue; 0.166= magenta; 0.33 = red; 0.50 = yellow
  // 0.66 = green; 0.833 = cyan to 1.00 = blue, brightness = 0.333, one primary
  // will always equal zero, and sat = 1.0.

  if (hue < 0.0) {    //validate phase
    hue = 0.0;
  }
  if (hue > 1.0) {
    hue = 1.0;
  }
  hue *= 3.0;                //six phases within hue cycle
  if (hue >= 0.0 && hue <= 1.0 ) {
    red = MAX_RGB / 1.00 * (hue - 0.0);
  }
  else if (hue > 1.0 &&  hue <= 2.0) {
    red = MAX_RGB - MAX_RGB / 1.00 * (hue - 1.0);
  }
  else {
    red = 0.0;
  }
  if (hue >= 1.0 && hue <= 2.0 ) {
    green = MAX_RGB / 1.00 * (hue - 1.0);
  }
  else if (hue > 2.0 &&  hue <= 3.0) {
    green = MAX_RGB - MAX_RGB / 1.00 * (hue - 2.0);
  }
  else {
    green = 0.0;
  }
  if (hue >=  2.0 && hue <= 3.0) {
    blue = MAX_RGB / 1.00 * (hue - 2.0);
  }
  else if (hue >= 0.0 && hue <= 1.0) {
    blue = MAX_RGB - MAX_RGB / 1.00 * (hue - 0.0);
  }
  else {
    blue = 0.0;
  }
  // Desaturate the pure hue to the extent sat < 1.0 by introducing the missing
  // primary and interpolating all three primaries towards equality at sat = 0.0
  // The result will have constant bright = 1/3; R+G+B = MAX_RGB and R=G=B at
  // sat = 0.
  red = (1 - sat) * MAX_RGB / 3.0 + sat * red;
  green = (1 - sat) * MAX_RGB / 3.0 + sat * green;
  blue = (1 - sat) * MAX_RGB / 3.0 + sat * blue;
  // Brightness = (red+green+blue)/MAX_RGB/3 and is limited by the largest primary.
  // bright <= 1/3 for a saturated primary color; <= 2/3 for a saturated secondary
  // color and <= 1.0 for white (where sat = 0.) The calling parameter is passed
  // by address and is adjusted as required for a constant hue
  if (bright > MAX_RGB / max(red, max(green, blue)) / 3.0) {
    bright = MAX_RGB / max(red, max(green, blue)) / 3.0;
  }
  red *= bright * 3.0;
  green *=  bright * 3.0;
  blue *= bright * 3.0;
}   // end of HSBtoRGB

//-------------------------------Convert RGB to HSB ----------------------------------
void RGBtoHSB(float & hue, float & sat, float & bright, float red, float green, float blue) {
  bright = (red + green + blue) / MAX_RGB / 3.0;    //by definition
  // Re-normalize the local RGB to bright = 1/3. The global RGB is unaffected
  float sum = red + green + blue;
  if (sum > 0.0) {
    red *= MAX_RGB / sum;
    green *= MAX_RGB / sum;
    blue *= MAX_RGB / sum;
  }
  // Remove the desaturating (lowest contributing) primary and recalculate
  // the saturated values of the other two
  sat = (MAX_RGB / 3.0 - min(red, min(green, blue))) / (MAX_RGB / 3.0);
  if (red <= min(green, blue)) {
    red = 0.0;
    green = (green - MAX_RGB / 3.0 + MAX_RGB / 3.0 * sat) / sat;
    blue = (blue - MAX_RGB / 3.0 + MAX_RGB / 3.0 * sat) / sat;
  }
  else if (green <= min(red, blue)) {
    green = 0.0;
    red = (red - MAX_RGB / 3.0 + MAX_RGB / 3.0 * sat) / sat;
    blue = (blue - MAX_RGB / 3.0 + MAX_RGB / 3.0 * sat) / sat;
  }
  else if (blue <= min(green, red)) {
    blue = 0.0;
    green = (green - MAX_RGB / 3.0 + MAX_RGB / 3.0 * sat) / sat;
    red = (red - MAX_RGB / 3.0 + MAX_RGB / 3.0 * sat) / sat;
  }
  // Reverse the ramp interpolation to derive the hue.  If sat ==0 or
  // bright==0, hue is undefined, so the hue passed with the call is left unchanged
  if (sat > 0.0 && bright > 0.0) {
    if (red >= max(green, blue)) {
      if (blue > green) hue = red / MAX_RGB;
      else hue = 2 - red / MAX_RGB;
    }
    if (green >= max(red, blue)) {
      if (red > blue)
        hue = 1.0 + green / MAX_RGB;
      else hue = 3.0 - green / MAX_RGB;
    }
    if (blue >= max(green, red)) {
      if (green > red)
        hue = 2.0 + blue / MAX_RGB;
      else hue = 0.0 + red / MAX_RGB;
    }
    hue /= 3.0;
  }
}

//-----------------blink 20mS to acknowledge command at a limit--------------------
void blink() {
  analogWrite(RED_PIN, 0);
  analogWrite(GREEN_PIN, 0);
  analogWrite(BLUE_PIN, 0);
  delay(20);
  DisplayRGB();
}

