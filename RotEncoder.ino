#include <EEPROM.h>

// EEPROM Addresses
#define ADDR_XY_SPEED_SET_MSB 0x00
#define ADDR_XY_SPEED_SET_LSB 0x01
#define ADDR_FOCUS_SPEED_SET_MSB 0x02
#define ADDR_FOCUS_SPEED_SET_LSB 0x03

// ROTATION_SPEED, ENCODER_POSITION_MAX, and
// ENCODER_POSITION_MIN determine how fast the circular bar graph
// fills up as the encoder is rotated.
#define ROTATION_SPEED 3  // MIN: 0, MAX: 5, 3 is a good value
// Use 208 as the reference max as there are 13 possible ring
// Number of LEDs turned on is 13-member set
//   {OFF, 2 on, 4 on, ... 12 on}
#define ENCODER_POSITION_MAX  (208 >> (ROTATION_SPEED - 1)) - 1
#define ENCODER_POSITION_MIN  0  // Don't go below 0

// LED ring lighting constants
#define LED_RING_OFF 0x0000
#define LED_RING_1   0x0400
#define LED_RING_2   0x0C00
#define LED_RING_3   0x1C00
#define LED_RING_4   0x3C00
#define LED_RING_5   0x7C00
#define LED_RING_6   0xFC00
#define LED_RING_7   0xFC01
#define LED_RING_8   0xFC03
#define LED_RING_9   0xFC07
#define LED_RING_10  0xFC0F
#define LED_RING_11  0xFC1F
#define LED_RING_12  0xFC3F

// Pin definitions - Encoder:
#define PIN_ENC_B 2     // Encoder B pin, D3 is external interrupt 0
#define PIN_ENC_A 3     // Encoder A pin, D2 is external interrupt 1
#define PIN_ENC_RLED 5 // Encoder's red LED - D11 is PWM enabled
#define PIN_ENC_BLED 6 // Encoder's blue LED- D10 is PWM enabled
#define PIN_ENC_GLED 7  // Encoder's green LED - D9 is PWM enabled
#define PIN_ENC_SWCH 8  // Encoder's switch pin

// Pin definitions - Shift registers:
#define PIN_ENC_DAT 9    // shift registers' SER pin
#define PIN_ENC_CLR 10    // shift registers' srclr pin
#define PIN_ENC_CLK 11    // Shift registers' srclk pin
#define PIN_ENC_LATCH 12 // Shift registers' rclk pin
#define PIN_ENC_EN 13    // Shift registers' Output Enable pin

// The encoderPosition variable stores the position of the encoder.
// It's either incremented or decremented in the encoder's 
// interrupt handler (readEncoder()). It's widely used in both the 
// loop() and ledRingFiller() and ledRingFollower() functions to
// control the LEDs.
signed int encoderPosition;      // Store the encoder's rotation counts
signed int encoderPosXYspeed;     // Store the encoder's rotation counts 
                                 // in XY_SPEED_SET state
signed int encoderPosFOCUSspeed;  // Store the encoder's rotation counts 
                                 // in FOCUS_SPEED_SET state
                                 
// Accessors
void setEncoderPosition(signed int position)
{
  encoderPosition = position;
}

signed int getEncoderPosXYspeed()
{
  return encoderPosXYspeed;
}

void setEncoderPosXYspeed(signed int speed)
{
  encoderPosXYspeed = speed;
}

signed int getEncoderPosFOCUSspeed()
{
  return encoderPosFOCUSspeed;
}

void setEncoderPosFOCUSspeed(signed int speed)
{
  encoderPosFOCUSspeed = speed;
}

// Persist parameters
void restoreFastSettings() {
  encoderPosXYspeed = restoreXY_SPEED_SETSetting();
  encoderPosFOCUSspeed = restoreFOCUS_SPEED_SETSetting();
}

signed int restoreXY_SPEED_SETSetting() {
  unsigned char msb = EEPROM.read(ADDR_XY_SPEED_SET_MSB);
  unsigned char lsb = EEPROM.read(ADDR_XY_SPEED_SET_LSB);
  signed int retVal = (signed int) (msb << 8 | lsb);
  return retVal;
}

signed int restoreFOCUS_SPEED_SETSetting() {
  unsigned char msb = EEPROM.read(ADDR_FOCUS_SPEED_SET_MSB);
  unsigned char lsb = EEPROM.read(ADDR_FOCUS_SPEED_SET_LSB);
  signed int retVal = (signed int) (msb << 8 | lsb);
  return retVal;  
}

void saveXY_SPEED_SETSetting(signed int setting) {
  unsigned char msb = (unsigned char) ((setting >> 8) & 0x00ff);
  unsigned char lsb = (unsigned char) (setting & 0x00ff);
  EEPROM.write(ADDR_XY_SPEED_SET_MSB,msb);
  EEPROM.write(ADDR_XY_SPEED_SET_LSB,lsb);
}

void saveFOCUS_SPEED_SETSetting(signed int setting) {
  unsigned char msb = (unsigned char) ((setting >> 8) & 0x00ff);
  unsigned char lsb = (unsigned char) (setting & 0x00ff);
  EEPROM.write(ADDR_FOCUS_SPEED_SET_MSB,msb);
  EEPROM.write(ADDR_FOCUS_SPEED_SET_LSB,lsb);
}

void encoder_init()
{
  // *********************************************
  // Encoder initialization
  // *********************************************
  // Setup encoder pins, they should both be set as inputs
  // and internally pulled-up
  pinMode(PIN_ENC_A, INPUT); 
  digitalWrite(PIN_ENC_A, HIGH);
  pinMode(PIN_ENC_B, INPUT);
  digitalWrite(PIN_ENC_B, HIGH);
  
  // Attach interrupts to encoder pins. Whenever one of the encoder
  // pins changes (rise or fall), we'll go to readEncoder()
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), readEncoder, CHANGE);
  
  // setup switch pins, set as an input, no pulled up
  pinMode(PIN_ENC_SWCH, INPUT);
  digitalWrite(PIN_ENC_SWCH, LOW);  // Disable internal pull-up
  
  // Setup led pins as outputs, and write their intial value.
  // initial value is defined by the ledValue global variable
  pinMode(PIN_ENC_RLED, OUTPUT);
  analogWrite(PIN_ENC_RLED, ledValue[RED]);  // Red off
  pinMode(PIN_ENC_GLED, OUTPUT);
  analogWrite(PIN_ENC_GLED, ledValue[GREEN]);  // Green off
  pinMode(PIN_ENC_BLED, OUTPUT);
  analogWrite(PIN_ENC_BLED, ledValue[BLUE]);  // Blue off
  
  // Setup shift register pins
  pinMode(PIN_ENC_EN, OUTPUT);  // Enable, active low, this'll always be LOW
  digitalWrite(PIN_ENC_EN, LOW);  // Turn all outputs on
  pinMode(PIN_ENC_LATCH, OUTPUT);  // this must be set before calling shiftOut16()
  digitalWrite(PIN_ENC_LATCH, LOW);  // start latch low
  pinMode(PIN_ENC_CLK, OUTPUT);  // we'll control this in shiftOut16()
  digitalWrite(PIN_ENC_CLK, LOW);  // start sck low
  pinMode(PIN_ENC_CLR, OUTPUT);  // master clear, this'll always be HIGH
  digitalWrite(PIN_ENC_CLR, HIGH);  // disable master clear
  pinMode(PIN_ENC_DAT, OUTPUT);  // we'll control this in shiftOut16()
  digitalWrite(PIN_ENC_DAT, LOW);  // start ser low
  
  // To begin, we'll turn all LEDs on the circular bar-graph OFF
  ledPartialRingSet(LED_RING_OFF);    

  // Read from EEPROM the old XY_SPEED_SET and FOCUS_SPEED_SET encoder positions
  restoreFastSettings();

  // NOTE: Do not enable interrupts since the first state is RUN
}

// Encoder switch state
bool encoderSwitchPressed()
{
  return (digitalRead(PIN_ENC_SWCH) == HIGH);
}

// Enable encoder
void enableEncoder()
{
  // Attach interrupts to encoder pins. Whenever one of the encoder
  // pins changes (rise or fall), we'll go to readEncoder()
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), readEncoder, CHANGE);
  delayMicroseconds(5000);
  interrupts();
  delayMicroseconds(5000);
}

void disableEncoder()
{
  noInterrupts();
  delayMicroseconds(5000);
  // Detach interrupts to encoder pins.
  detachInterrupt(digitalPinToInterrupt(PIN_ENC_A));
  detachInterrupt(digitalPinToInterrupt(PIN_ENC_B));
  delayMicroseconds(5000);
}

void encoderRGB(byte red, byte green, byte blue) {
  analogWrite(PIN_ENC_RLED, red);
  analogWrite(PIN_ENC_GLED, green);
  analogWrite(PIN_ENC_BLED, blue);
}

void ledPartialRingBounce() {
  static unsigned int bounceDelay = 20;
  ledPartialRingSet(LED_RING_OFF);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_1);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_2);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_3);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_4);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_5);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_6);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_7);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_8);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_9);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_10);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_11);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_12);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_11);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_10);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_9);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_8);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_7);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_6);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_5);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_4);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_3);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_2);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_1);
  delay(bounceDelay);
  ledPartialRingSet(LED_RING_OFF);
  delay(bounceDelay);
}

void ledPartialRingFiller() {
  unsigned int ledOutput = LED_RING_OFF;
//  Serial.print("switchState = ");
//  Serial.println(switchState);
  if ((switchState == XY_SPEED_SET) || (switchState == FOCUS_SPEED_SET)) {
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13)
      ledOutput = LED_RING_1;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*2)
      ledOutput = LED_RING_2;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*3)
      ledOutput = LED_RING_3;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*4)
      ledOutput = LED_RING_4;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*5)
      ledOutput = LED_RING_5;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*6)
      ledOutput = LED_RING_6;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*7)
      ledOutput = LED_RING_7;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*8)
      ledOutput = LED_RING_8;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*9)
      ledOutput = LED_RING_9;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*10)
      ledOutput = LED_RING_10;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*11)
      ledOutput = LED_RING_11;
    if (encoderPosition >= (ENCODER_POSITION_MAX + 1)/13*12)
      ledOutput = LED_RING_12;
  }
  ledPartialRingSet(ledOutput);
}

void ledPartialRingSet(unsigned int leds) {
  // Write to the shift registers and latch.
  digitalWrite(PIN_ENC_LATCH, LOW);  // first send latch low
  shiftOut16(leds);  // send the ledOutput value to shiftOut16
  digitalWrite(PIN_ENC_LATCH, HIGH);  // send latch high to indicate data is done sending 
}

// This function'll call shiftOut (a pre-defined Arduino function)
// twice to shift 16 bits out. Latch is not controlled here, so you
// must do it before this function is called.
//   data is sent 8 bits at a time, MSB first.
void shiftOut16(uint16_t data)
{
  byte datamsb;
  byte datalsb;
  
  // Isolate the MSB and LSB
  datamsb = (data&0xFF00)>>8;  // mask out the MSB and shift it right 8 bits
  datalsb = data & 0xFF;  // Mask out the LSB
  
  // First shift out the MSB, MSB first.
  shiftOut(PIN_ENC_DAT, PIN_ENC_CLK, MSBFIRST, datamsb);
  // Then shift out the LSB
  shiftOut(PIN_ENC_DAT, PIN_ENC_CLK, MSBFIRST, datalsb);
}

// readEncoder() is our interrupt handler for the rotary encoder.
// This function is called every time either of the two encoder
// pins (A and B) either rise or fall.
//   This code will determine the directino of rotation, and 
// update the global encoderPostion variable accordingly.
//   This code is adapted from Rotary Encoder code by Oleg.
void readEncoder()
{
  if (switchState == RUN) {
    return;
  }
  noInterrupts();  // don't want our interrupt to be interrupted
  // First, we'll do some software debouncing. Optimally there'd
  // be some form of hardware debounce (RC filter). If there is
  // feel free to get rid of the delay. If your encoder is acting
  // 'wacky' try increasing or decreasing the value of this delay.
  delayMicroseconds(5000);  // 'debounce'
  
  // enc_states[] is a fancy way to keep track of which direction
  // the encoder is turning. 2-bits of oldEncoderState are paired
  // with 2-bits of newEncoderState to create 16 possible values.
  // Each of the 16 values will produce either a CW turn (1),
  // CCW turn (-1) or no movement (0).
  int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t oldEncoderState = 0;
  static uint8_t newEncoderState = 0;

  // First, find the newEncoderState. This'll be a 2-bit value
  // the msb is the state of the B pin. The lsb is the state
  // of the A pin on the encoder.
  newEncoderState = (digitalRead(PIN_ENC_B)<<1) | (digitalRead(PIN_ENC_A));
  
  // Now we pair oldEncoderState with new encoder state
  // First we need to shift oldEncoder state left two bits.
  // This'll put the last state in bits 2 and 3.
  oldEncoderState <<= 2;
  // Mask out everything in oldEncoderState except for the previous state
  oldEncoderState &= 0xC0;
  // Now add the newEncoderState. oldEncoderState will now be of
  // the form: 0b0000(old B)(old A)(new B)(new A)
  oldEncoderState |= newEncoderState; // add filteredport value

  // Now we can update encoderPosition with the updated position
  // movement. We'll either add 1, -1 or 0 here.
  encoderPosition += enc_states[oldEncoderState];
  
  // This next bit will only happen if CONTINUOUS is not defined.
  // If CONTINUOUS is defined, encoderPosition will roll over from
  // -32768 (assuming it's a signed int) to to 32767 if decremented, 
  // or 32767 to -32768 if incremented.
  //   That can be useful for some applications. In this code, we
  // want the encoder value to stop at 255 and 0 (makes analog writing easier)
  #ifndef CONTINUOUS
    // If encoderPosition is greater than the MAX, just set it
    // equal to the MAX
    if (encoderPosition > ENCODER_POSITION_MAX)
      encoderPosition = ENCODER_POSITION_MAX;
    // otherwise, if encoderPosition is less than the MIN, set it
    // equal to the MIN.
    else if (encoderPosition < ENCODER_POSITION_MIN)
      encoderPosition = ENCODER_POSITION_MIN;
  #endif
  switch(switchState) {
    case XY_SPEED_SET:
      if (encoderPosXYspeed != encoderPosition) {
        encoderPosXYspeed = encoderPosition;
      }
    break;
    case FOCUS_SPEED_SET:
      if (encoderPosFOCUSspeed != encoderPosition) {
        encoderPosFOCUSspeed = encoderPosition;
      }
    break;
    default:
    break;
  }
  
  interrupts();  // re-enable interrupts before we leave

}



