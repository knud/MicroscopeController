/*
 * an arduino sketch to interface with a ps/2 mouse.
 * Also uses serial protocol to talk back to the host
 * and report what it finds.
 */


// Control is based on three states:
//   RUN is the main operational state. The microscope is used normally
//   XY_SPEED_SET state allows for the fast movement of the XY mechanical stage.
//     It is meant to allow the user to position the slide quickly. The
//     rotary encoder sets the speed X and Y movement.
//   FOCUS_SPEED_SET state allows for the fast movement of the fine focus.
//     It is meant to allow the user to focus more quickly. The
//     rotary encoder sets the focus speed.
enum SWITCH_STATE {RUN = 1, HOME_FAST = 2, XY_SPEED_SET = 3, FOCUS_SPEED_SET = 4};
byte switchState = RUN; // Starting state

enum RUN_STATE {SLOW = 1, XY_FAST = 2, FOCUS_FAST = 3};
byte runState = SLOW;

// RGB LEDs are common-anode; 255 is off, 0 is full on.
// ledValue[] stores the 8-bit analogue value for the LEDs.
enum LEDS_ADDR {RED = 0, GREEN = 1, BLUE = 2};
byte ledValue[3] = {255, 255, 255}; // all off default

// Mode button and LEDs
#define PIN_MODE_BUTTON 35
#define PIN_MODE_LED0 36
#define PIN_MODE_LED1 37

// Extra button
#define PIN_EXTRA_BUTTON 4

// Home sensors
#define HOMED 0

// digital pin 2 has a pushbutton attached to it. Give it a name:
int hallSensorX = 28;
int hallSensorY = 29;

 #include <AccelStepper.h>
#define HALFSTEP 8

// Motor pin definitions
#define PIN_MOTOR1_IN1  51      // IN1 on the ULN2003 driver 1
#define PIN_MOTOR1_IN2  50     // IN2 on the ULN2003 driver 1
#define PIN_MOTOR1_IN3  52      // IN3 on the ULN2003 driver 1
#define PIN_MOTOR1_IN4  53      // IN4 on the ULN2003 driver 1
#define PIN_MOTOR2_IN1  47      // IN1 on the ULN2003 driver 2
#define PIN_MOTOR2_IN2  46      // IN2 on the ULN2003 driver 2
#define PIN_MOTOR2_IN3  48      // IN3 on the ULN2003 driver 2
#define PIN_MOTOR2_IN4  49      // IN4 on the ULN2003 driver 2
#define PIN_MOTOR3_IN1  43      // IN1 on the ULN2003 driver 3
#define PIN_MOTOR3_IN2  42      // IN2 on the ULN2003 driver 3
#define PIN_MOTOR3_IN3  44      // IN3 on the ULN2003 driver 3
#define PIN_MOTOR3_IN4  45      // IN4 on the ULN2003 driver 3

#define STEPPER_MAX_SPEED 3000
#define STEPPER_FAST STEPPER_MAX_SPEED
#define STEPPER_SLOW (STEPPER_MAX_SPEED/2)

int speedMultiplier = 1;

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepperX(HALFSTEP, PIN_MOTOR1_IN1, PIN_MOTOR1_IN3, PIN_MOTOR1_IN2, PIN_MOTOR1_IN4);
AccelStepper stepperY(HALFSTEP, PIN_MOTOR2_IN1, PIN_MOTOR2_IN3, PIN_MOTOR2_IN2, PIN_MOTOR2_IN4);
AccelStepper stepperF(HALFSTEP, PIN_MOTOR3_IN1, PIN_MOTOR3_IN3, PIN_MOTOR3_IN2, PIN_MOTOR3_IN4);

long homing_step;
volatile long xpos = 0;
volatile long ypos = 0;
volatile long fpos = 0;

void setup()
{
  Serial.begin(115200);
  // *********************************************
  // Mouse and steppers initialization
  // *********************************************
  mouse_init();
  stepperX.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperX.setAcceleration(300.0);
  stepperX.setSpeed(STEPPER_FAST);
  stepperX.disableOutputs();
  stepperX.setCurrentPosition(xpos);
  stepperY.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperY.setAcceleration(300.0);
  stepperY.setSpeed(STEPPER_FAST);
  stepperY.disableOutputs();
  stepperY.setCurrentPosition(ypos);
  stepperF.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperF.setAcceleration(300.0);
  stepperF.setSpeed(STEPPER_FAST);
  stepperF.disableOutputs();
  stepperF.setCurrentPosition(fpos);

  encoder_init();

  // Set up the Mode button and LED pins
  pinMode(PIN_MODE_BUTTON, INPUT);
  digitalWrite(PIN_MODE_BUTTON, LOW);  // Disable internal pull-up
  pinMode(PIN_MODE_LED0, OUTPUT);  // upper LED
  digitalWrite(PIN_MODE_LED0, LOW);  // set off
  pinMode(PIN_MODE_LED1, OUTPUT);  // upper LED
  digitalWrite(PIN_MODE_LED1, LOW);  // set off

  // Set up extra button
  pinMode(PIN_EXTRA_BUTTON, INPUT_PULLUP); // Enable internal pull-up
  
  // set the encoder RGB to RUN state
  encoderRGB(0,0,0);
  delay(200);
  Serial.print("initial encoderPosXYspeed : ");
  Serial.println(getEncoderPosXYspeed());
  Serial.print("initial encoderPosFOCUSspeed : ");
  Serial.println(getEncoderPosFOCUSspeed());

  // bounce the LED ring
  ledPartialRingBounce();

  // flash the LEDs
  delay(100);
  digitalWrite(PIN_MODE_LED0, HIGH);  // set off
  delay(100);
  digitalWrite(PIN_MODE_LED1, HIGH);  // set off
  delay(100);
  digitalWrite(PIN_MODE_LED0, LOW);  // set off
  delay(100);
  digitalWrite(PIN_MODE_LED1, LOW);  // set off
  delay(100);
  digitalWrite(PIN_MODE_LED0, HIGH);  // set off
  delay(100);
  digitalWrite(PIN_MODE_LED1, HIGH);  // set off
  delay(100);
  digitalWrite(PIN_MODE_LED0, LOW);  // set off
  delay(100);
  digitalWrite(PIN_MODE_LED1, LOW);  // set off
  delay(100);  

  xyHome();
}
 
/*
 * get a reading from the mouse and report it back to the
 * host via the serial line.
 */

void NOTloop() {
  Serial.print("loop: ");
  Serial.println(xpos++);
  delay(1000);
}

void loop()
{
  int extraButton = digitalRead(PIN_EXTRA_BUTTON);
  if (extraButton == LOW)
  {
    ledPartialRingBounce();
    xyCentre();
  }
    
  // If the switch is pressed, switch state
  if (encoderSwitchPressed()) {
    delay(10);
    if (encoderSwitchPressed()) {
      Serial.println("HIGH");
      switch (switchState) {
        case RUN:
          stepperX.stop();
          stepperX.disableOutputs();
          stepperY.stop();
          stepperY.disableOutputs();
          stepperF.stop();
          stepperF.disableOutputs();
          switchState = HOME_FAST;
          Serial.println("State is now HOME_FAST");
          delay(100);
          encoderRGB(255,255,255);
        break;
        case HOME_FAST:
          switchState = XY_SPEED_SET;
          enableEncoder();
          Serial.println("State is now XY_SPEED_SET");
          delay(100);
          encoderRGB(255,0,255);
          setEncoderPosition(getEncoderPosXYspeed());
        break;
        case XY_SPEED_SET:
          switchState = FOCUS_SPEED_SET;
          Serial.println("State is now FOCUS_SPEED_SET");
          // interrupts already enabled by change from RUN to XY_SPEED_SET state
          // No need to enable here
          saveXY_SPEED_SETSetting(getEncoderPosXYspeed());
          delay(100);
          encoderRGB(255,255,0);
          setEncoderPosition(getEncoderPosFOCUSspeed());
        break;
        case FOCUS_SPEED_SET:
          switchState = RUN;
          Serial.println("State is now RUN");
          saveFOCUS_SPEED_SETSetting(getEncoderPosFOCUSspeed());
          delay(100);
          // However, because something gets masked or something else, have to
          // resort to a kludge to ensure the motors will still run...
          swReset();
        break;
        default:
        break;
      }
    }
         
    while(encoderSwitchPressed())
      ;  // do nothing
    // Update the partial circle bar graph
    ledPartialRingFiller();  
  }


  if (switchState == RUN) {

    // If the Home/Mode button is pressed, switch state
    if (digitalRead(PIN_MODE_BUTTON) == LOW) {
      delay(10);
      if (digitalRead(PIN_MODE_BUTTON) == LOW) {
        // TODO actually change the speed!
         Serial.println("HOME/MODE Pressed");
         switch (runState) {
           case SLOW:
           runState = XY_FAST;
           digitalWrite(PIN_MODE_LED0, HIGH);
           digitalWrite(PIN_MODE_LED1, LOW);
           Serial.println("New HOME mode is XY_FAST");
           speedMultiplier = 50;
           break;
           case XY_FAST:
           runState = FOCUS_FAST;
           digitalWrite(PIN_MODE_LED0, LOW);
           digitalWrite(PIN_MODE_LED1, HIGH);
           Serial.println("New HOME mode is FOCUS_FAST");
           break;
           case FOCUS_FAST:
           runState = SLOW;
           digitalWrite(PIN_MODE_LED0, LOW);
           digitalWrite(PIN_MODE_LED1, LOW);
           Serial.println("New HOME mode is SLOW");
           speedMultiplier = 1;
           break;
           default:
           break;
         }
       }
      while(digitalRead(PIN_MODE_BUTTON) == LOW)
      ;  // do nothing
    }

    // Mouse handling
    char mstat;
    int mx;
    int my;
  
    /* get a position and button reading from the mouse */
    mouse_write(0xeb);  /* give me data! */
    mouse_read();      /* ignore ack */
    mstat = mouse_read();
    mx = mouse_read()*speedMultiplier;
    my = mouse_read()*speedMultiplier;
  
    
    /* send the data back up */
    if (abs(mx) > 0 || abs (my) > 0 || abs(stepperX.distanceToGo()) > 0 || abs(stepperY.distanceToGo()) > 0 || (mstat & 0x03) != 0)
    {
      Serial.print(mstat, HEX);
      Serial.print("\tX=");
      Serial.print(mx, DEC);
      Serial.print("\tY=");
      Serial.print(my, DEC);
      Serial.print("\txpos=");
      Serial.print(xpos, DEC);
      Serial.print("\txRem=");
      Serial.print(stepperX.distanceToGo(), DEC);
      Serial.print("\typos=");
      Serial.print(ypos, DEC);
      Serial.print("\tyRem=");
      Serial.print(stepperY.distanceToGo(), DEC);
      Serial.println(""); 
    }

    if (stepperX.distanceToGo() != 0)
      stepperX.enableOutputs();
    else
      stepperX.disableOutputs();
    xpos += mx;
    if (xpos <= 0) xpos = 0;
    if (xpos > 46600) xpos = 46600;
    if (xpos != stepperX.currentPosition())
    {
      stepperX.moveTo(xpos);
      if (abs(stepperX.distanceToGo()) > 1000)
        while (abs(stepperX.distanceToGo()) > 1000) stepperX.run();
      else if (abs(stepperX.distanceToGo()) > 100)
        while (abs(stepperX.distanceToGo()) > 100) stepperX.run();
      else if (abs(stepperX.distanceToGo()) > 50)
        while (abs(stepperX.distanceToGo()) > 50) stepperX.run();
      else
        stepperX.run();
    }
    
    if (stepperY.distanceToGo() != 0)
      stepperY.enableOutputs();
    else
      stepperY.disableOutputs();
    ypos -= my;
    if (ypos >= 0) ypos = 0;
    if (ypos < -12600) ypos = -12600;
    if (ypos != stepperY.currentPosition())
    {
      stepperY.moveTo(ypos);
      if (abs(stepperY.distanceToGo()) > 1000)
        while (abs(stepperY.distanceToGo()) > 1000) stepperY.run();
      else if (abs(stepperY.distanceToGo()) > 100)
        while (abs(stepperY.distanceToGo()) > 100) stepperY.run();
      else if (abs(stepperY.distanceToGo()) > 50)
        while (abs(stepperY.distanceToGo()) > 50) stepperY.run();
      else
        stepperY.run();  
    }
  
    if (stepperF.distanceToGo() != 0) {
//      Serial.print("F enabled ");
//      Serial.println(stepperF.distanceToGo());
      stepperF.enableOutputs();
    }
    else {
//      Serial.println("F disabled");
      stepperF.disableOutputs();
    }
    if (fpos != stepperF.currentPosition())
    {
      stepperF.moveTo(fpos);
      if (abs(stepperF.distanceToGo()) > 100)
        while (abs(stepperF.distanceToGo()) > 100) stepperF.run();
      else if (abs(stepperF.distanceToGo()) > 10)
        while (abs(stepperF.distanceToGo()) > 10) stepperF.run();
      else if (abs(stepperF.distanceToGo()) > 5)
        while (abs(stepperF.distanceToGo()) > 5) stepperF.run();
      else
         stepperF.run();        
    }
    if ((mstat & 0x03) != 0) {
      if ((mstat & 0x01) != 0)
        fpos += 10;
      if ((mstat & 0x02) != 0)
        fpos -= 10;
    }
    
  }
//  else if (switchState == HOME_FAST) {
//    
//  } else
//    ledPartialRingFiller();  

}

void swReset() // Restarts program from beginning but does not reset the peripherals and registers
{
  asm volatile ("  jmp 0");  
}  

#define XCENTER_POS 18175
#define YCENTER_POS -2950
void xyCentre()
{
  // The center positions are empirical
  stepperX.enableOutputs();
  long pos = xpos; // stepperX.currentPosition();
  long s = (XCENTER_POS - pos) / 5;
  Serial.print("x pos = ");
  Serial.print(pos);
  Serial.print(" step = ");
  Serial.println(s);
  long x, y;
  for (x = pos; x <= XCENTER_POS; x += s)
  {
    Serial.print("X move to ");
    Serial.println(x);
    stepperX.moveTo(x);
    while (stepperX.distanceToGo() > 0) stepperX.run();
    delay(500);
  }
  stepperX.disableOutputs();
  xpos = XCENTER_POS;

  delay(500);

  stepperY.enableOutputs();
  pos = ypos; // stepperY.currentPosition();
  s = (YCENTER_POS - pos) / 5;
  Serial.print("y pos = ");
  Serial.print(pos);
  Serial.print(" step = ");
  Serial.println(s);
  for (y = pos; y > YCENTER_POS; y += s)
  {
    Serial.print("Y move to ");
    Serial.println(y);
    stepperY.moveTo(y);
    while (stepperY.distanceToGo() < 0) stepperY.run();
    delay(500);
  }
  stepperY.disableOutputs();
  ypos = YCENTER_POS;
}

void xyHome()
{
  //-----------------------------------------------------
  // X stage homing
  //-----------------------------------------------------
  // if already near home, move well base home and then
  // find the threshold position for sensor trigger. Then
  // move to true home
  int pos_sensor_homed = digitalRead(hallSensorX);
  homing_step = 1000;
  if (pos_sensor_homed == HOMED) {
    Serial.println("X -- Initially close to home");
    xpos = 0;
    stepperX.enableOutputs();
    stepperX.setCurrentPosition(xpos);
    // move well past home
    xpos = 4500;
    stepperX.moveTo(xpos);
    while (stepperX.distanceToGo() > 0) stepperX.run();
    stepperX.disableOutputs();
    delay(50);
    pos_sensor_homed = digitalRead(hallSensorX);
  } // if initially near home
  else {
    // not already near home and no idea how far away.
    // but, we can certainly move 4000 steps
    Serial.println("X -- Initially away from home");
    xpos = 0;
    stepperX.enableOutputs();
    stepperX.setCurrentPosition(xpos);
    while (pos_sensor_homed != HOMED) {
      Serial.println("move -4000");
      xpos -= 4000;
      stepperX.moveTo(xpos);
      while (stepperX.distanceToGo() < 0) stepperX.run();
      delay(50);
      pos_sensor_homed = digitalRead(hallSensorX);        
    }
  }
  // get closer to home
  stepperX.setSpeed(STEPPER_SLOW);
  while (abs(homing_step) > 15) {
    while (pos_sensor_homed == HOMED) {
      Serial.print("move past another ");
      Serial.println(homing_step);
      xpos += homing_step;
      stepperX.moveTo(xpos);
      while (stepperX.distanceToGo() > 0) stepperX.run();
      delay(50);
      pos_sensor_homed = digitalRead(hallSensorX);
    }
    homing_step = -homing_step / 2;
    while (pos_sensor_homed != HOMED) {
      Serial.print("move toward another ");
      Serial.println(homing_step);
      xpos += homing_step;
      stepperX.moveTo(xpos);
      while (stepperX.distanceToGo() < 0) stepperX.run();
      delay(50);
      pos_sensor_homed = digitalRead(hallSensorX);
    }
    homing_step = -homing_step / 2;
  }
  Serial.print("xpos = ");
  Serial.println(xpos);
      xpos -= 3250;
      stepperX.moveTo(xpos);
      while (stepperX.distanceToGo() < 0) stepperX.run();
  Serial.print("xpos = ");
  Serial.println(xpos);
  Serial.println("setting xpos to zero");
  stepperX.setCurrentPosition(0);
  xpos = 0;
  stepperX.disableOutputs();

  //-----------------------------------------------------
  // Y stage homing
  //-----------------------------------------------------
  // if already near home, move well base home and then
  // find the threshold position for sensor trigger. Then
  // move to true home

  // 1230 to 1240 step from home threshold to home
  
  pos_sensor_homed = digitalRead(hallSensorY);

  homing_step = -900;
  if (pos_sensor_homed == HOMED) {
    Serial.println("Y -- Initially close to home");
    ypos = 0;
    stepperY.enableOutputs();
    stepperY.setCurrentPosition(ypos);
    // move well past home
    ypos = -1500;
    stepperY.moveTo(ypos);
    while (stepperY.distanceToGo() < 0) stepperY.run();
    delay(50);
    pos_sensor_homed = digitalRead(hallSensorY);
    if (pos_sensor_homed == HOMED)
      Serial.println("Y -- HOMED");
    else
      Serial.println("Y -- not HOMED");
  } // if initially near home
  else {
    // not already near home and no idea how far away.
    // but, we can certainly move 4000 steps
    Serial.println("Y -- Initially away from home");
    ypos = 0;
    stepperY.enableOutputs();
    stepperY.setCurrentPosition(ypos);
    while (pos_sensor_homed != HOMED) {
      Serial.println("move +1100");
      ypos += 1100;
      stepperY.moveTo(ypos);
      while (stepperY.distanceToGo() > 0) stepperY.run();
      delay(50);
      pos_sensor_homed = digitalRead(hallSensorY);        
    }
  }
  // get closer to home
  stepperY.setSpeed(STEPPER_SLOW);
  while (abs(homing_step) > 15) {
    while (pos_sensor_homed == HOMED) {
      Serial.print("move past another ");
      Serial.println(homing_step);
      ypos += homing_step;
      stepperY.moveTo(ypos);
      while (stepperY.distanceToGo() < 0) stepperY.run();
      delay(50);
      pos_sensor_homed = digitalRead(hallSensorY);
    }
    homing_step = -homing_step / 2;
    while (pos_sensor_homed != HOMED) {
      Serial.print("move toward another ");
      Serial.println(homing_step);
      ypos += homing_step;
      stepperY.moveTo(ypos);
      while (stepperY.distanceToGo() > 0) stepperY.run();
      delay(50);
      pos_sensor_homed = digitalRead(hallSensorY);
    }
    homing_step = -homing_step / 2;
  }
  Serial.print("ypos = ");
  Serial.println(ypos);
      ypos += 1200;
      stepperY.moveTo(ypos);
      while (stepperY.distanceToGo() > 0) stepperY.run();
  Serial.print("ypos = ");
  Serial.println(ypos);
  Serial.println("setting ypos to zero");
  stepperY.setCurrentPosition(0);
  ypos = 0;
  stepperY.disableOutputs();  
}

