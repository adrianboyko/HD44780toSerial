
const int BUFFER_SIZE = 1024;
const int BUFFER_INDEX_MASK = BUFFER_SIZE - 1;

namespace Pin {
  const int ROT_B          =  8;
  const int ROT_A          =  9;
  const int RIGHT_BUTTON   = 10;
  const int LEFT_BUTTON    = 11;
  const int ENCODER_BUTTON = 12;
  const int LCD_ENABLE     =  2; // This is used for EN clocking, i.e. to catch uSDX sending cmd/data to LCD.
  const int LCD_POWER      =  6; // This will be used to detect uSDX power up and power down.
  const int PUSH_TO_TALK   =  7; 
  const int USDX_RESET     =  5; // This is connected to the uSDX's reset pin.
}

const unsigned long PULSE_DURATION = 5;  // milliseconds

enum gesture_t {
  CLICK_LEFT_BUTTON               = 1,
  CLICK_RIGHT_BUTTON              = 2,
  CLICK_ENCODER_BUTTON            = 3,
  ROTATE_ENCODER_CLOCKWISE        = 4,
  ROTATE_ENCODER_COUNTERCLOCKWISE = 5,
  PUSH_TO_TALK_START              = 6,
  PUSH_TO_TALK_END                = 7,
  RESET_THE_USDX                  = 8
};

volatile bool usdxPowerOn = false;
volatile unsigned char lcdBuffer[BUFFER_SIZE];
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

namespace Button {

  unsigned long timeOfClickStart = 0;
  int pinOfButtonClicked = 0;

  bool clickInProgress() {
    return pinOfButtonClicked > 0;
  }

  void startClick(int buttonPin, bool pushedState) {
    pinMode(buttonPin, OUTPUT);
    digitalWrite(buttonPin, pushedState);
    timeOfClickStart = millis();
    pinOfButtonClicked = buttonPin;
  }

  void endClick() {
    bool newState = !digitalRead(pinOfButtonClicked);
    digitalWrite(pinOfButtonClicked, newState);
    pinMode(pinOfButtonClicked, INPUT);  // No PULLDOWN because there's already a pulldown on the uSDX side.
    timeOfClickStart = 0; // meaning "none"
    pinOfButtonClicked = 0; // meaning "none"
    gestureInProgress = false;
  }
  
  void emulate() {
    bool isTimeToEndClick = millis() > (timeOfClickStart + PULSE_DURATION);
    if (isTimeToEndClick) endClick();
  }
} // namespace Button


namespace Encoder {

  enum state_t { 
    ENC_STATE_IDLE = 0,  // Both encoder pins high
    ENC_STATE_1 = 1,     // First encoder pin low, 2nd still high
    ENC_STATE_2 = 2,     // Both encoder pins low
    ENC_STATE_3 = 3      // First encoder pin high, 2nd still low
  };

  unsigned long stateTime = millis();
  state_t state = ENC_STATE_IDLE;
  int firstPin = 0;
  int secondPin = 0;

  bool rotationInProgress() {
    return state != ENC_STATE_IDLE;
  }

  void dropFirstPin() {
    pinMode(firstPin, OUTPUT);
    digitalWrite(firstPin, LOW); // SHould already be low, but just to be safe. HIGH would be very bad because physical encoder, if used, could possibly short it to ground.
    state = ENC_STATE_1;
    stateTime = millis();
  }

  void dropSecondPin() {
    pinMode(secondPin, OUTPUT);
    digitalWrite(secondPin, LOW); // SHould already be low, but just to be safe. HIGH would be very bad because physical encoder, if used, could possibly short it to ground.
    state = ENC_STATE_2;
    stateTime = millis(); 
  }

  void raiseFirstPin() {
    pinMode(firstPin, INPUT); // This is high Z on our side and there is a pull-up on the uSDX side
    state = ENC_STATE_3;
    stateTime = millis();   
  }

  void raiseSecondPin() {
    pinMode(secondPin, INPUT); // This is high Z on our side and there is a pull-up on the uSDX side
    state = ENC_STATE_IDLE;
    stateTime = millis();   
  }

  void startRotation(int _firstPin, int _secondPin) {
    firstPin = _firstPin;
    secondPin = _secondPin;
    dropFirstPin();  
  }

  void emulate() {
    if (millis() - stateTime < PULSE_DURATION) return;
    switch (state) {
      case ENC_STATE_1: dropSecondPin(); break;
      case ENC_STATE_2: raiseFirstPin(); break;
      case ENC_STATE_3: raiseSecondPin(); gestureInProgress = false; break;
    }
  }
} // namespace Encoder


void performInputGesture(byte gesture) {
  gestureInProgress = true; 
  switch (gesture) {
    case CLICK_LEFT_BUTTON: Button::startClick(Pin::LEFT_BUTTON, HIGH); break;
    case CLICK_RIGHT_BUTTON: Button::startClick(Pin::RIGHT_BUTTON, HIGH); break;
    case CLICK_ENCODER_BUTTON: Button::startClick(Pin::ENCODER_BUTTON, HIGH); break;
    case ROTATE_ENCODER_CLOCKWISE: Encoder::startRotation(Pin::ROT_A, Pin::ROT_B); break;
    case ROTATE_ENCODER_COUNTERCLOCKWISE: Encoder::startRotation(Pin::ROT_B, Pin::ROT_A); break;
    case PUSH_TO_TALK_START: startPushToTalk(); break;
    case PUSH_TO_TALK_END: endPushToTalk(); break;
    case RESET_THE_USDX: doUsdxReset(); break;
  }
}

void doUsdxReset() {
  Button::startClick(Pin::USDX_RESET, LOW);
}

void startPushToTalk() {
  pinMode(Pin::PUSH_TO_TALK, OUTPUT);
  digitalWrite(Pin::PUSH_TO_TALK, LOW);
  gestureInProgress = false;
}

void endPushToTalk() {
  pinMode(Pin::PUSH_TO_TALK, INPUT);
  gestureInProgress = false;
}


void setup() {
  
  // These should remain INPUT at all times except when they are going to simulate a button press.
  // This protects them when hardware buttons are pushed, because there is no hardware protection on at least one of them.
  // Note that there's already a 10K pulldown on the uSDX side, so no point in doing so here.
  pinMode(Pin::RIGHT_BUTTON,   INPUT);
  pinMode(Pin::LEFT_BUTTON,    INPUT);
  pinMode(Pin::ENCODER_BUTTON, INPUT);
  pinMode(Pin::ROT_A,          INPUT);
  pinMode(Pin::ROT_B,          INPUT);

  // This goes low when the LCD bits are ready to be read.
  pinMode(Pin::LCD_ENABLE,     INPUT_PULLUP  );
  
  // This will be high when uSDX is powered on.
  pinMode(Pin::LCD_POWER,      INPUT_PULLDOWN);

  // This is connected to an input pin on the uSDX, but we'll keep it hi-z except when pulling the uSDX pin low.
  pinMode(Pin::PUSH_TO_TALK,   INPUT);

  // This is connected to the reset pin on the uSDX's 328. Pulse low to reset the 328.
  pinMode(Pin::USDX_RESET,     OUTPUT);
  digitalWrite(LED_BUILTIN,    LOW);

  // LED will be used to indicate 0) startup/reset, 1) waiting for power, and 2) buffer overflow.
  pinMode(LED_BUILTIN,         OUTPUT);
  digitalWrite(LED_BUILTIN,    LOW);


  // Port D will be used to read the LCD interface   
  PORTD.DIRCLR = B00000000; // Set the entire port to INPUT

  Serial.begin(500000);

  // Flashing LED indicates startup or reset.
  for (int i = 1; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }

}

void loop() {

  // This waits for the uSDX to power up and then starts watching for lcd activity once the startup noise pass.
  digitalWrite(LED_BUILTIN, HIGH);
  attachInterrupt(digitalPinToInterrupt(Pin::LCD_POWER), usdxPowerUpISR, RISING);
  while (!usdxPowerOn);
  detachInterrupt(digitalPinToInterrupt(Pin::LCD_POWER));
  delay(100); // This skips noise from the startup.
  attachInterrupt(digitalPinToInterrupt(Pin::LCD_ENABLE), lcdActivityISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(Pin::LCD_POWER), usdxPowerDownISR, FALLING);
  digitalWrite(LED_BUILTIN, LOW);

  // Will indicate power UP by sending RS=0 DATA=11111111 (i.e. Set DDRAM address to 127, which is invalid)
  Serial.write(markAsLcdByte(B00100111)); // This is RS=0 Nibble=1111
  Serial.write(markAsLcdByte(B00100111)); // This is RS=0 Nibble=1111

  // The uSDX is powered on at this point, so do the normal workflow.
  while (usdxPowerOn || idxOfNextLcdRead != idxOfNextLcdWrite) {  // Second condition allows for power off signal to be sent.

    if (idxOfNextLcdRead != idxOfNextLcdWrite) {
      Serial.write(markAsLcdByte(lcdBuffer[idxOfNextLcdRead]));
      idxOfNextLcdRead = (idxOfNextLcdRead + 1) & BUFFER_INDEX_MASK;
    }

    if (!gestureInProgress && Serial.available() > 0) {
      byte gesture = Serial.read();
      performInputGesture(gesture);
    }

    if (Button::clickInProgress()) {
      Button::emulate();
    }

    if (Encoder::rotationInProgress()) {
      Encoder::emulate();
    }
  }  

  // uSDX power has been turned off, at this point.
  detachInterrupt(digitalPinToInterrupt(Pin::LCD_POWER));
  detachInterrupt(digitalPinToInterrupt(Pin::LCD_ENABLE));

  // Will indicate power DOWN by sending RS=0 DATA=11111110 (i.e. Set DDRAM address to 126, which is invalid)
  Serial.write(markAsLcdByte(B00100111)); // This is RS=0 Nibble=1111
  Serial.write(markAsLcdByte(B00100110)); // This is RS=0 Nibble=1110
  
}

inline byte markAsLcdByte(byte b) {
  return b & B01111111;
}

inline byte markAsCatByte(byte b) {
  return b | B10000000;
}
