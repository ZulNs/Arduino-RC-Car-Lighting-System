/*
 * RC Car Lighting System Using WS2812B
 * 
 * 
 * Created 5 January 2020
 * @Gorontalo, Indonesia
 * by ZulNs
 */

//#include <FastLED.h>
#include <Adafruit_NeoPixel.h>

#define SWITCH_1_INPUT_PIN  1  // PC1 (A1)
#define SWITCH_2_INPUT_PIN  2  // PC2 (A2)
#define SWITCH_3_INPUT_PIN  3  // PC3 (A3)
#define THROTLE_INPUT_PIN   4  // PC4 (AA)

#define PWM_START_FWD 1600
#define PWM_START_REV 1400

#define MOTOR_STOP            0
#define MOTOR_FWD_SPIN        2
#define MOTOR_REV_SPIN        3
#define MOTOR_FWD_DECELERATED 4
#define MOTOR_REV_DECELERATED 5
#define MOTOR_FWD_BRAKING     8
#define MOTOR_REV_BRAKING     9

//#define LED_TYPE       WS2812B
//#define COLOR_ORDER    GRB
#define FRONT_LED_PIN  6
#define REAR_LED_PIN   5
#define AUX_LED_PIN    4
#define FRONT_LEDS     2
#define REAR_LEDS      2
#define AUX_LEDS       8

/*#define brakeLampOn()   rearLeds[0]=rearLeds[1]=CRGB(BRAKE_VAL,0,0)
#define rearLampOn()    rearLeds[0]=rearLeds[1]=CRGB(REAR_VAL,0,0)
#define rearLampOff()   rearLeds[0]=rearLeds[1]=0
#define frontLampHigh() frontLeds[0]=frontLeds[1]=CRGB(FRONT_HIGH_VAL,FRONT_HIGH_VAL,FRONT_HIGH_VAL)
#define frontLampLow()  frontLeds[0]=frontLeds[1]=CRGB(FRONT_LOW_VAL,FRONT_LOW_VAL,FRONT_LOW_VAL)
#define frontLampOff()  frontLeds[0]=frontLeds[1]=0*/

//CRGB frontLeds[FRONT_LEDS];
//CRGB rearLeds[REAR_LEDS];
//CRGB auxLeds[AUX_LEDS];
Adafruit_NeoPixel front = Adafruit_NeoPixel(FRONT_LEDS, FRONT_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel rear  = Adafruit_NeoPixel(REAR_LEDS,  REAR_LED_PIN,  NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel aux   = Adafruit_NeoPixel(AUX_LEDS,   AUX_LED_PIN,   NEO_GRB + NEO_KHZ800);

volatile int16_t throtlePwm;
volatile uint8_t switchState;

ISR (PCINT1_vect) {
  static uint32_t beginPwmTimer[4];
  static uint8_t  oldIntData;
  uint32_t curTime;
  int16_t timeDelta;
  uint8_t intData, intSrc, i;
  curTime = micros();
  intData = PINC & 0x1E;
  intSrc = oldIntData ^ intData;
  for (i = 1; i < 5; ++i) {
    if (bitRead(intSrc, i)) {
      if (bitRead(PINC, i))
        // Whenever pulse goes HIGH
        beginPwmTimer[i % 4] = curTime;
      else {
        // Whenever pulse goes LOW
        timeDelta = curTime - beginPwmTimer[i % 4];
        if (i == THROTLE_INPUT_PIN)
          throtlePwm = timeDelta;
        else
          bitWrite(switchState, i, timeDelta > 1500);
      }
    }
  }
  oldIntData = intData;
}

void setup() {
  //Serial.begin(115200);
  //Serial.println(F("*** RC Car Lighting System ***"));
  //Serial.println();
  
  cli();
  PCMSK1 |= 0x1E;       // Masking for PORTC: PCINT9, PCINT10, PCINT11, PCINT12
  PCIFR |= bit(PCIF1);  // Clears any outstanding interrupts on PORTC
  PCICR |= bit(PCIE1);  // Enables Pin Change Interrupts on PORTC
  sei();
  
  //FastLED.addLeds<LED_TYPE, FRONT_LED_PIN, COLOR_ORDER>(frontLeds, FRONT_LEDS);
  //FastLED.addLeds<LED_TYPE, REAR_LED_PIN, COLOR_ORDER>(rearLeds, REAR_LEDS);
  //FastLED.addLeds<LED_TYPE, AUX_LED_PIN, COLOR_ORDER>(auxLeds, AUX_LEDS);
  front.begin();
  front.setBrightness(255);
  front.show(); // Initialize all pixels to 'off'
  rear.begin();
  rear.setBrightness(255);
  rear.show(); // Initialize all pixels to 'off'
  aux.begin();
  aux.setBrightness(32);
  aux.show(); // Initialize all pixels to 'off'
  
  randomSeed(analogRead(0));
  
  while (throtlePwm <= PWM_START_REV || PWM_START_FWD <= throtlePwm);  // Wait until TX-RX get binded and TX throtle stick position at around the center
  brakeLampOn();
  frontLampOn();
  auxLampOn();
  delay(50);
  rearLampOff();
  frontLampOff();
  auxLampOff();
  delay(400);
  brakeLampOn();
  fogLampOn();
  auxLampOn();
  delay(50);
  rearLampOff();
  frontLampOff();
  auxLampOff();
}

void loop() {
  static uint8_t oldSwitchState, auxCtr;
  static int8_t  oldMotorStatus = MOTOR_STOP;
  static uint32_t throtleDeltaTimer, auxTimer;
  static boolean isBraking, isLampOn, isFogLamp, isAuxOn;
  uint8_t motorStatus = oldMotorStatus, i;
  int16_t thrPwm;
  PCICR &= ~bit(PCIE1);
  thrPwm = throtlePwm;
  PCICR |= bit(PCIE1);
  
  switch (oldMotorStatus) {
    case MOTOR_STOP:
      if (thrPwm >= PWM_START_FWD)
        motorStatus = MOTOR_FWD_SPIN;
      else if (thrPwm <= PWM_START_REV)
        motorStatus = MOTOR_REV_SPIN;
      break;
    case MOTOR_FWD_SPIN:
      if (PWM_START_REV < thrPwm && thrPwm < PWM_START_FWD) {
        throtleDeltaTimer = millis();
        motorStatus = MOTOR_FWD_DECELERATED;
      }
      else if (thrPwm <= PWM_START_REV)
        motorStatus = MOTOR_FWD_BRAKING;
      break;
    case MOTOR_REV_SPIN:
      if (PWM_START_REV < thrPwm && thrPwm < PWM_START_FWD) {
        throtleDeltaTimer = millis();
        motorStatus = MOTOR_REV_DECELERATED;
      }
      else if (thrPwm >= PWM_START_FWD) {
        throtleDeltaTimer = millis();
        motorStatus = MOTOR_REV_BRAKING;
      }
      break;
    case MOTOR_FWD_DECELERATED:
      if (thrPwm >= PWM_START_FWD)
        motorStatus = MOTOR_FWD_SPIN;
      else if (millis() >= throtleDeltaTimer + 2000)
        motorStatus = (thrPwm <= PWM_START_REV) ? MOTOR_REV_SPIN : MOTOR_STOP;
      else if (thrPwm <= PWM_START_REV)
        motorStatus = MOTOR_FWD_BRAKING;
      break;
    case MOTOR_REV_DECELERATED:
      if (thrPwm <= PWM_START_REV)
        motorStatus = MOTOR_REV_SPIN;
      else if (millis() >= throtleDeltaTimer + 1000)
        motorStatus = (thrPwm >= PWM_START_FWD) ? MOTOR_FWD_SPIN : MOTOR_STOP;
      else if (thrPwm >= PWM_START_FWD) {
        throtleDeltaTimer = millis();
        motorStatus = MOTOR_REV_BRAKING;
      }
      break;
    case MOTOR_FWD_BRAKING:
      if (thrPwm >= PWM_START_FWD)
        motorStatus = MOTOR_FWD_SPIN;
      else if (thrPwm > PWM_START_REV)
        motorStatus = MOTOR_STOP;
      break;
    case MOTOR_REV_BRAKING:
      if (thrPwm <= PWM_START_REV)
        motorStatus = MOTOR_REV_SPIN;
      else if (thrPwm < PWM_START_FWD)
        motorStatus = MOTOR_STOP;
      else if (millis() >= throtleDeltaTimer + 750)
        motorStatus = MOTOR_FWD_SPIN;
      break;
  }
  
  if (oldMotorStatus != MOTOR_FWD_BRAKING && oldMotorStatus != MOTOR_REV_BRAKING && (motorStatus == MOTOR_FWD_BRAKING || motorStatus == MOTOR_REV_BRAKING)) {
    isBraking = true;
    brakeLampOn();
  }
  else if ((oldMotorStatus == MOTOR_FWD_BRAKING || oldMotorStatus == MOTOR_REV_BRAKING) && motorStatus != MOTOR_FWD_BRAKING && motorStatus != MOTOR_REV_BRAKING) {
    isBraking = false;
    if (isLampOn)
      rearLampOn();
    else
      rearLampOff();
  }
  
  if (motorStatus != oldMotorStatus) {
    oldMotorStatus = motorStatus;
    //Serial.println("Throtle: " + String(throtlePwm));
    //Serial.print("MotorStatus: ");
    //Serial.println((int)motorStatus);
  }

  if (oldSwitchState != switchState) {
    uint8_t swDelta = oldSwitchState ^ switchState;
    isLampOn = bitRead(switchState, 1);
    isFogLamp = bitRead(switchState, 2);
    isAuxOn = bitRead(switchState, 3);
    if (bitRead(swDelta, 1)) {
      if (isLampOn) {
        if (bitRead(swDelta, 2) == 0) {
          if (isFogLamp)
            fogLampOn();
          else
            frontLampOn();
        }
        if (!isBraking)
          rearLampOn();
        if (bitRead(swDelta, 3) == 0 && isAuxOn) {
          auxLampOn();
          auxTimer = millis();
          auxCtr = 0;
        }
      }
      else {
        frontLampOff();
        if (!isBraking)
          rearLampOff();
        if (bitRead(swDelta, 3) == 0 && isAuxOn)
          auxLampOff();
      }
    }
    if (bitRead(swDelta, 2)) {
      if (isLampOn) {
        if (isFogLamp)
          fogLampOn();
        else
          frontLampOn();
      }
    }
    if (bitRead(swDelta, 3)) {
      if (isLampOn) {
        if (isAuxOn) {
          auxLampOn();
          auxTimer = millis();
          auxCtr = 0;
        }
        else
          auxLampOff();
      }
    }
    //Serial.println("Switches: " + String(switchState));
    oldSwitchState = switchState;
  }
  
  if (isLampOn && isAuxOn) {
    if (auxCtr < 56) {
      if (millis() >= auxTimer + 50) {
        ++auxCtr;
        rotateLeft(aux);
        if (auxCtr % 8 == 0)
          aux.setPixelColor(auxCtr / 8 - 1, 0);
        aux.show();
        auxTimer = millis();
      }
    }
    else if (auxCtr < 64) {
      if (millis() >= auxTimer + 50) {
        rotateLeft(aux);
        aux.show();
        auxTimer = millis();
        ++auxCtr;
      }
    }
    else if (auxCtr < 112) {
      if (millis() >= auxTimer + 50) {
        rotateLeft(aux);
        if (auxCtr % 8 == 0)
          aux.setPixelColor(7 - (auxCtr - 64) / 8, getRandomColor());
        aux.show();
        auxTimer = millis();
        ++auxCtr;
      }
    }
    else if (auxCtr == 112) {
      if (millis() >= auxTimer + 50) {
        aux.setPixelColor(0, getRandomColor());
        rotateLeft(aux);
        aux.show();
        auxTimer = millis();
        auxCtr = 0;
      }
    }
  }
}

void frontLampOn() {
  setLamp(front, 0x4090B4);
}

void fogLampOn() {
  setLamp(front, 0xC09000);
}

void frontLampOff() {
  setLamp(front, 0);
}

void brakeLampOn() {
  setLamp(rear, 0xC00000);
}

void rearLampOn() {
  setLamp(rear, 0x200000);
}

void rearLampOff() {
  setLamp(rear, 0);
}

void setLamp(Adafruit_NeoPixel & dev, uint32_t color) {
  dev.setPixelColor(0, color);
  dev.setPixelColor(1, color);
  dev.show();
}

void auxLampOn() {
  for (uint8_t i = 0; i < aux.numPixels(); ++i)
    aux.setPixelColor(i, getRandomColor());
  aux.show();
}

void auxLampOff() {
  for (uint8_t i = 0; i < aux.numPixels(); ++i)
    aux.setPixelColor(i, 0);
  aux.show();
}

uint32_t getRainbowColor(uint8_t n, uint8_t r) {
  int16_t norm = uint16_t((float)n / r * 255 * 6);
  uint8_t x = norm % 255;
  uint8_t y = 255 - x;
  uint8_t red = 0, green = 0, blue = 0;
  switch (norm / 255) {
    case 0: red = 255; green = x; break;
    case 1: red = y; green = 255; break;
    case 2: green = 255; blue = x; break;
    case 3: green = y; blue = 255; break;
    case 4: red = x; blue = 255; break;
    case 5: red = 255; blue = y;
  }
  green = green * (192/256.0);
  blue  = blue  * (240/256.0);
  return ((uint32_t)red << 16) | ((uint32_t)green << 8) | blue;
}

uint32_t getRandomColor() {
  uint8_t rnd = random(1, 7);
  return ((uint32_t)(bitRead(rnd, 0) * 255) << 16) | ((uint32_t)(bitRead(rnd, 1) * 192) << 8) | bitRead(rnd, 2) * 240;
}

void rotateRight(Adafruit_NeoPixel & dev) {
  uint8_t numLeds = dev.numPixels();
  uint8_t bright = dev.getBrightness();
  uint32_t tmp;
  dev.setBrightness(255);
  tmp = dev.getPixelColor(0);
  for (int8_t i = 1; i < numLeds; ++i)
    dev.setPixelColor(i - 1, dev.getPixelColor(i));
  dev.setPixelColor(numLeds - 1, tmp);
  dev.setBrightness(bright);
}

void rotateLeft(Adafruit_NeoPixel & dev) {
  uint8_t numLeds = dev.numPixels();
  uint8_t bright = dev.getBrightness();
  uint32_t tmp;
  dev.setBrightness(255);
  tmp = dev.getPixelColor(numLeds - 1);
  for (int8_t i = numLeds - 2; i >= 0; --i)
    dev.setPixelColor(i + 1, dev.getPixelColor(i));
  dev.setPixelColor(0, tmp);
  dev.setBrightness(bright);
}
