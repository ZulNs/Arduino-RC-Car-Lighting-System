/*
 * RC Car Lighting System
 * 
 * 
 * Created 17 August 2018
 * @Gorontalo, Indonesia
 * by ZulNs
 */


#define THROTLE_INPUT_PIN    0  // PC0 (A0)
#define SWITCH_1_INPUT_PIN   1  // PC1 (A1)
#define SWITCH_2_INPUT_PIN   2  // PC2 (A2)
#define SWITCH_3_INPUT_PIN   3  // PC3 (A3)

#define FRONT_LEFT_LED_PIN   4  // PD4 (D4)
#define FRONT_RIGHT_LED_PIN  7  // PD7 (D7)
#define REAR_LEFT_LED_PIN    5  // PD5 (D5); must be PWM supported
#define REAR_RIGHT_LED_PIN   6  // PD6 (D6); must be PWM supported

#define MOTOR_STOP           0
#define MOTOR_SPIN_FORWARD   1
#define MOTOR_SPIN_REVERSE   2
#define MOTOR_BRAKING        3

#define UPPER_MID_VAL    1500
#define LOWER_MID_VAL    1352
#define MIN_HIGHEST_VAL  1888

#define PWM_REAR_LIGHT_OFF            0
#define PWM_REAR_LIGHT_NORMAL_BRIGHT  50
#define PWM_REAR_LIGHT_BRIGHTEST      255

#define FLASH_LIGHT_ON_PERIODE   50
#define FLASH_LIGHT_OFF_PERIODE  450

#define throtlePWM sharedPWM[THROTLE_INPUT_PIN]

uint32_t startTime[4];
uint32_t pwm[4];
volatile uint32_t sharedPWM[4];
volatile uint8_t  throtleState, switchState;
uint8_t  oldThrotleState, oldSwitchState, oldIntData;
uint32_t oldThrotlePWM, flashTimer;
boolean  isBraking, isFlashBraking, isLightingOn, isFlashStateOn;

void frontLampOff()
{
  PORTD &= ~bit(FRONT_RIGHT_LED_PIN) & ~bit(FRONT_LEFT_LED_PIN);
}

void frontLampOn()
{
  PORTD |= bit(FRONT_RIGHT_LED_PIN) | bit(FRONT_LEFT_LED_PIN);
}

void rearLampOff()
{
  analogWrite(REAR_RIGHT_LED_PIN, PWM_REAR_LIGHT_OFF);
  analogWrite(REAR_LEFT_LED_PIN,  PWM_REAR_LIGHT_OFF);
}

void rearLampOn()
{
  analogWrite(REAR_RIGHT_LED_PIN, PWM_REAR_LIGHT_NORMAL_BRIGHT);
  analogWrite(REAR_LEFT_LED_PIN,  PWM_REAR_LIGHT_NORMAL_BRIGHT);
}

void brakeLampOn()
{
  analogWrite(REAR_RIGHT_LED_PIN, PWM_REAR_LIGHT_BRIGHTEST);
  analogWrite(REAR_LEFT_LED_PIN, PWM_REAR_LIGHT_BRIGHTEST);
}

void readPulseWidth()
{
  cli();
  memcpy(pwm, (const void *)sharedPWM, sizeof(sharedPWM));
  sei();
}

void decodeThrotleState()
{
  switch (throtleState)
  {
    case MOTOR_STOP:
      if (throtlePWM > UPPER_MID_VAL + 8)
      {
        throtleState = MOTOR_SPIN_FORWARD;
      }
      else if (throtlePWM < LOWER_MID_VAL - 8)
      {
        throtleState = MOTOR_SPIN_REVERSE;
      }
      break;
    
    case MOTOR_SPIN_FORWARD:
      if (UPPER_MID_VAL >= oldThrotlePWM && oldThrotlePWM >= LOWER_MID_VAL)
      {
        if (oldThrotlePWM - throtlePWM <= 8)
        {
          throtleState = MOTOR_STOP;
        }
      }
      else if (throtlePWM < LOWER_MID_VAL - 8)
      {
        throtleState = MOTOR_BRAKING;
      }
      break;
    
    case MOTOR_SPIN_REVERSE:
      if (UPPER_MID_VAL >= oldThrotlePWM && oldThrotlePWM >= LOWER_MID_VAL)
      {
        if (throtlePWM - oldThrotlePWM <= 8)
        {
          throtleState = MOTOR_STOP;
        }
      }
      else if (throtlePWM > UPPER_MID_VAL + 8)
      {
        throtleState = MOTOR_SPIN_FORWARD;
      }
      break;
    
    case MOTOR_BRAKING:
      if (UPPER_MID_VAL >= throtlePWM && throtlePWM >= LOWER_MID_VAL)
      {
        throtleState = MOTOR_STOP;
      }
      else if (throtlePWM > UPPER_MID_VAL + 8)
      {
        throtleState = MOTOR_SPIN_FORWARD;
      }
  }
}

ISR (PCINT1_vect)
{
  uint8_t intData, channels, channel;
  uint32_t currentTime = micros();
  intData = PINC & 0x0F;
  channels = oldIntData ^ intData;
  channel;
  for (channel = 0; channel < 4; channel++)
  {
    if (bitRead(channels, channel))
    {
      if (bitRead(PINC, channel))
      {
        // Whenever pulse goes HIGH
        startTime[channel] = currentTime;
      }
      else
      {
        // Whenever pulse goes LOW
        if (channel == THROTLE_INPUT_PIN)
        {
          oldThrotlePWM = sharedPWM[channel];
        }
        sharedPWM[channel] = currentTime - startTime[channel];
        if (channel == THROTLE_INPUT_PIN)
        {
          decodeThrotleState();
        }
        else
        {
          bitWrite(switchState, channel, sharedPWM[channel] > MIN_HIGHEST_VAL);
        }
      }
    }
  }
  oldIntData = intData;
}

void setup()
{
  /*Serial.begin(115200);
  Serial.println(F("*** RC Car Lighting System ***"));
  Serial.println();
  */
  
  DDRD |= bit(FRONT_RIGHT_LED_PIN) | bit(FRONT_LEFT_LED_PIN);
  frontLampOff();
  rearLampOff();
  
  cli();
  PCMSK1 |= 0x0F;       // Masking for PORTC: PCINT8, PCINT9, PCINT10, PCINT11
  PCIFR |= bit(PCIF1);  // Clears any outstanding interrupts on PORTC
  PCICR |= bit(PCIE1);  // Enables Pin Change Interrupts on PORTC
  throtlePWM = 0;
  sei();
  
  while (throtlePWM == 0);  // Wait until TX-RX binded and throtle pulse available
}

void loop()
{
  uint8_t changedSwitchState = oldSwitchState ^ switchState;
  
  if (oldThrotleState != throtleState)
  {
    if (throtleState == MOTOR_BRAKING)
    {
      isBraking = true;
      brakeLampOn();
      if (isFlashBraking)
      {
        flashTimer = millis();
        isFlashStateOn = true;
      }
    }
    else if (oldThrotleState == MOTOR_BRAKING)
    {
      isBraking = false;
      if (isLightingOn)
      {
        frontLampOn();
        rearLampOn();
      }
      else
      {
        frontLampOff();
        rearLampOff();
      }
    }
    oldThrotleState = throtleState;
    /*Serial.print(F("Throtle State: "));
    Serial.println(throtleState);
    Serial.print(F("Throtle Pulse: "));
    Serial.println(throtlePWM);
    */
  }
  
  if (changedSwitchState != 0)
  {
    if (bitRead(changedSwitchState, SWITCH_1_INPUT_PIN))
    {
      isLightingOn = bitRead(switchState, SWITCH_1_INPUT_PIN);
      if (isLightingOn)
      {
        if (!(isBraking && isFlashBraking && !isFlashStateOn))
        {
          frontLampOn();
        }
        if (!isBraking)
        {
          rearLampOn();
        }
      }
      else
      {
        frontLampOff();
        if (!isBraking)
        {
          rearLampOff();
        }
      }
    }
    if (bitRead(changedSwitchState, SWITCH_2_INPUT_PIN))
    {
      isFlashBraking = bitRead(switchState, SWITCH_2_INPUT_PIN);
      if (isBraking)
      {
        if (isFlashBraking)
        {
          flashTimer = millis();
          isFlashStateOn = true;
        }
        else
        {
          if (isLightingOn)
          {
            frontLampOn();
          }
          else
          {
            frontLampOff();
          }
          brakeLampOn();
        }
      }
    }
    oldSwitchState = switchState;
    /*Serial.print(F("Switch State: "));
    Serial.println(switchState);
    */
  }
  
  if (isFlashBraking && isBraking)
  {
    if (isFlashStateOn)
    {
      if (millis() > flashTimer + FLASH_LIGHT_ON_PERIODE)
      {
        flashTimer = millis();
        isFlashStateOn = false;
        rearLampOff();
        frontLampOff();
      }
    }
    else
    {
      if (millis() > flashTimer + FLASH_LIGHT_OFF_PERIODE)
      {
        flashTimer = millis();
        isFlashStateOn = true;
        brakeLampOn();
        if (isLightingOn)
        {
          frontLampOn();
        }
      }
    }
  }
  
  /*
  readPulseWidth();
  for (byte i = 0; i < 4; i++)
  {
    Serial.print(F("Ch"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(pwm[i]);
    Serial.print("uS");
    Serial.print(F("\t"));
  }
  Serial.println();
  delay(500);
  */
}

