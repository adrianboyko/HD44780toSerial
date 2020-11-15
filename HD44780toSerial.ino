
#include <avr/io.h>

const int BUFFER_SIZE = 1024;
const int BUFFER_INDEX_MASK = BUFFER_SIZE - 1;

const int ROT_B_PIN          =  8;
const int ROT_A_PIN          =  9;
const int RIGHT_BUTTON_PIN   = 10;
const int LEFT_BUTTON_PIN    = 11;
const int ENCODER_BUTTON_PIN = 12;
const int LCD_ENABLE_PIN     =  2; // This is used for EN clocking, i.e. to catch uSDX sending cmd/data to LCD.
const int LCD_POWER_PIN      =  6; // This will be used to detect uSDX power up and power down.

const unsigned long BUTTON_CLICK_DURATION = 5;  // milliseconds

const byte CLICK_LEFT_BUTTON               = 1;
const byte CLICK_RIGHT_BUTTON              = 2;
const byte CLICK_ENCODER_BUTTON            = 3;
const byte ROTATE_ENCODER_CLOCKWISE        = 4;
const byte ROTATE_ENCODER_COUNTERCLOCKWISE = 5;

volatile bool usdxPowerOn = false;

volatile unsigned char *volatile lcdBuffer = malloc(BUFFER_SIZE);
volatile int idxOfNextLcdWrite = 0;
volatile int idxOfNextLcdRead = 0;

bool gestureInProgress = false;

void usdxPowerUpISR() {
  usdxPowerOn = true;
}

void usdxPowerDownISR() {
  usdxPowerOn = false;
}

void lcdActivityISR() {
  lcdBuffer[idxOfNextLcdWrite] = PORTD.IN;
  idxOfNextLcdWrite = (idxOfNextLcdWrite + 1) & BUFFER_INDEX_MASK;
  if (idxOfNextLcdWrite == idxOfNextLcdRead) { // Buffer overflowed. Alert user by lighting LED.
    digitalWrite(LED_BUILTIN, HIGH);  // REVIEW: is digitalWrite too slow for ISR?
  }
}

void performInputGesture(byte gesture) {
  gestureInProgress = true; 
  switch (gesture) {
    case CLICK_LEFT_BUTTON: startButtonClick(LEFT_BUTTON_PIN); break;
    case CLICK_RIGHT_BUTTON: startButtonClick(RIGHT_BUTTON_PIN); break;
    case CLICK_ENCODER_BUTTON: startButtonClick(ENCODER_BUTTON_PIN); break;
    case ROTATE_ENCODER_CLOCKWISE: startEncoderRotation(ROT_A_PIN, ROT_B_PIN); break;
    case ROTATE_ENCODER_COUNTERCLOCKWISE: startEncoderRotation(ROT_B_PIN, ROT_A_PIN); break;
  }
}

unsigned long timeOfClickStart = 0;
int pinOfButtonClicked = 0;

void startButtonClick(int buttonPin) {
  pinMode(buttonPin, OUTPUT);
  digitalWrite(buttonPin, HIGH);
  timeOfClickStart = millis();
  pinOfButtonClicked = buttonPin;
}

void endButtonClick() {
  digitalWrite(pinOfButtonClicked, LOW);
  pinMode(pinOfButtonClicked, INPUT);  // No PULLDOWN because there's already a pulldown on the uSDX side.
  timeOfClickStart = 0; // meaning "none"
  pinOfButtonClicked = 0; // meaning "none"
  gestureInProgress = false;
}

enum encoder_state_t { 
  ENC_STATE_IDLE = 0,  // Both encoder pins high
  ENC_STATE_1 = 1,     // First encoder pin low, 2nd still high
  ENC_STATE_2 = 2,     // Both encoder pins low
  ENC_STATE_3 = 3      // First encoder pin high, 2nd still low
};

unsigned long encoderStateTime = millis();
encoder_state_t encoderState = ENC_STATE_IDLE;
int firstRotPin = 0;
int secondRotPin = 0;

void startEncoderRotation(int _firstRotPin, int _secondRotPin) {
  firstRotPin = _firstRotPin;
  secondRotPin = _secondRotPin;
  dropFirstRotPin();  
}

void dropFirstRotPin() {
  pinMode(firstRotPin, OUTPUT);
  digitalWrite(firstRotPin, LOW); // SHould already be low, but just to be safe. HIGH would be very bad because physical encoder, if used, could possibly short it to ground.
  encoderState = ENC_STATE_1;
  encoderStateTime = millis();
}

void dropSecondRotPin() {
  pinMode(secondRotPin, OUTPUT);
  digitalWrite(secondRotPin, LOW); // SHould already be low, but just to be safe. HIGH would be very bad because physical encoder, if used, could possibly short it to ground.
  encoderState = ENC_STATE_2;
  encoderStateTime = millis(); 
}

void raiseFirstRotPin() {
  pinMode(firstRotPin, INPUT); // This is high Z on our side and there is a pull-up on the uSDX side
  encoderState = ENC_STATE_3;
  encoderStateTime = millis();   
}

void raiseSecondRotPin() {
  pinMode(secondRotPin, INPUT); // This is high Z on our side and there is a pull-up on the uSDX side
  encoderState = ENC_STATE_IDLE;
  encoderStateTime = millis();   
}

void emulateEncoder() {
  if (millis() - encoderStateTime < BUTTON_CLICK_DURATION) return;
  switch (encoderState) {
    case ENC_STATE_1: dropSecondRotPin(); break;
    case ENC_STATE_2: raiseFirstRotPin(); break;
    case ENC_STATE_3: raiseSecondRotPin(); gestureInProgress = false; break;
  }
}

void setup() {
  
  // These should remain INPUT at all times except when they are going to simulate a button press.
  // This protects them when hardware buttons are pushed, because there is no hardware protection on at least one of them.
  // Note that there's already a 10K pulldown on the uSDX side, so no point in doing so here.
  pinMode(RIGHT_BUTTON_PIN,   INPUT);
  pinMode(LEFT_BUTTON_PIN,    INPUT);
  pinMode(ENCODER_BUTTON_PIN, INPUT);
  pinMode(ROT_A_PIN,          INPUT);
  pinMode(ROT_B_PIN,          INPUT);
  
  pinMode(LCD_ENABLE_PIN,     INPUT_PULLUP  );
  pinMode(LCD_POWER_PIN,      INPUT_PULLDOWN);
 
  Serial.begin(500000);

  // LED will be used to indicate 1) waiting for power, and 2) buffer overflow.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Port D will be used to read the LCD interface   
  PORTD.DIRCLR = B00000000; // Set the entire port to INPUT

}

void loop() {

  // This waits for the uSDX to power up and then starts watching for lcd activity once the startup noise pass.
  digitalWrite(LED_BUILTIN, HIGH);
  attachInterrupt(digitalPinToInterrupt(LCD_POWER_PIN), usdxPowerUpISR, RISING);
  while (!usdxPowerOn);
  detachInterrupt(digitalPinToInterrupt(LCD_POWER_PIN));
  delay(100); // This skips noise from the startup.
  attachInterrupt(digitalPinToInterrupt(LCD_ENABLE_PIN), lcdActivityISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LCD_POWER_PIN), usdxPowerDownISR, FALLING);
  digitalWrite(LED_BUILTIN, LOW);

  // Will indicate power UP by sending RS=0 DATA=11111111 (i.e. Set DDRAM address to 127, which is invalid)
  Serial.write(B00100111); // This is RS=0 Nibble=1111
  Serial.write(B00100111); // This is RS=0 Nibble=1111

  // The uSDX is powered on at this point, so do the normal workflow.
  while (usdxPowerOn || idxOfNextLcdRead != idxOfNextLcdWrite) {  // Second condition allows for power off signal to be sent.

    if (idxOfNextLcdRead != idxOfNextLcdWrite) {
      Serial.write(lcdBuffer[idxOfNextLcdRead]);
      idxOfNextLcdRead = (idxOfNextLcdRead + 1) & BUFFER_INDEX_MASK;
    }

    if (!gestureInProgress && Serial.available() > 0) {
      byte gesture = Serial.read();
      performInputGesture(gesture);
    }

    if (pinOfButtonClicked > 0) {
      bool isTimeToEndClick = millis() > (timeOfClickStart + BUTTON_CLICK_DURATION);
      if (isTimeToEndClick) endButtonClick();
    }

    if (encoderState != ENC_STATE_IDLE) {
      emulateEncoder();
    }
  }  

  // uSDX power has been turned off, at this point.
  detachInterrupt(digitalPinToInterrupt(LCD_POWER_PIN));

  // Will indicate power DOWN by sending RS=0 DATA=11111110 (i.e. Set DDRAM address to 126, which is invalid)
  Serial.write(B00100111); // This is RS=0 Nibble=1111
  Serial.write(B00100110); // This is RS=0 Nibble=1110
  

}
