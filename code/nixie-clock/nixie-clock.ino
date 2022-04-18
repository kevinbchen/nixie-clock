#include <Button.h>
#include <RotaryEncoder.h>
#include <RTClib.h>

// Pins
#define ENCODER1  4
#define ENCODER2  7
#define BUTTON1   A3
#define BUTTON2   8
#define BUTTON3   11

#define SHIFT_RCLK  A0
#define SHIFT_SRCLK A1
#define SHIFT_SER   A2

#define EN1 9
#define EN2 10
#define EN3 6
#define EN4 5
#define EN5 3

// Inputs
RotaryEncoder encoder(ENCODER1, ENCODER2, RotaryEncoder::LatchMode::FOUR3);
Button buttonEncoder(BUTTON1);
Button buttonA(BUTTON2);
Button buttonB(BUTTON3);
int lastEncoderPosition = 0;

// Outputs
uint8_t pwmPins[] = {EN4, EN3, EN5, EN2, EN1};

// RTC
RTC_DS3231 rtc;
DateTime dateTime;
uint8_t lastSecond = 0;

// Digits
uint16_t digitsData = 0x0000;
uint8_t digits[4] = {0};
uint8_t brightness = 5;

// Main display states
enum DisplayState {
  Time = 0,     // HH:mm
  TimeSeconds,  // mm:ss
  Date,         // MM:dd
  Year,         // yyyy
  Brightness,   // 1 :XX
  DePoison,     // de-poison
  NumStates
};

// PWM patterns (including separator). Each pattern lasts for 1 second.
uint8_t pwmPatterns[][2] = {
  {B11111, B11011},   // HH:mm
  {B11111, B11111},   // mm:ss
  {B11111, B11111},   // MM:dd
  {B11011, B11011},   // yyyy
  {B10111, B10111},   // 1 : X
  {B11111, B11011},   // de-poison
};
uint8_t pwmPatternIndex = 0;

// Value ranges when editing (inclusive)
struct EditRange {
  uint8_t min;
  uint8_t max;
} editRanges[][2] = {
  {{0, 23}, {0, 59}},   // HH:mm
  {{0, 59}, {0, 59}},   // mm:ss
  {{1, 12}, {1, 31}},   // MM:dd
  {{0, 0}, {0, 99}},    // yyyy
  {{0, 0}, {1, 9}},     // 1 : X
  {{0, 0}, {0, 0}},     // de-poison
};

DisplayState displayState = DisplayState::Time;
uint8_t displayValues[] = {0, 0};
unsigned long lastUpdateTime = 0;
unsigned long dePoisonTime = 0;
unsigned long dePoisonTimes[] = {1300, 2000, 2750, 3300, 5000};
uint8_t dePoisonTimeIndex = 0;
uint8_t dePoisonDigits[4];
uint8_t dePoisonPermutation[] = {0, 1, 2, 3};
bool dePoisonDone[4];

// Editing is done on 2-digit values (hours, minutes, etc.)
bool editMode = false;
uint8_t editIndex = 0;
uint8_t editValues[] = {0, 0};
uint8_t editPwmMask[] = {B11000, B00011};

void setup() {
  pinMode(SHIFT_RCLK, OUTPUT);
  pinMode(SHIFT_SRCLK, OUTPUT);
  pinMode(SHIFT_SER, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(EN4, OUTPUT);
  pinMode(EN5, OUTPUT);

  // Set Timer1 and Timer2 to fast PWM (8-bit) to match freq of Timer0 (976.5 Hz)
  // Increased freq helps reduce flickering of INS-1 indicator
  // Timer1 WGM1_3:0 = 0101
  TCCR1A = (TCCR1A & B11111100) | _BV(WGM10);
  TCCR1B = (TCCR1B & B11100111) | _BV(WGM12);
  // Timer2 WGM2_2:0 = 011
  TCCR2A = (TCCR2A & B11111100) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = (TCCR2B & B11110111);

  encoder.setPosition(lastEncoderPosition);
  buttonEncoder.begin();
  buttonA.begin();
  buttonB.begin();

  rtc.begin();
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  dateTime = rtc.now();

  Serial.begin(9600);
  Serial.println(dateTime.toString("Current time: YYMMDD-hh:mm:ss"));

  updateDisplay();
  updatePWM();
}

void updateDigits() {
  uint16_t newDigitsData = 0x0000;
  for (int i = 0; i < 4; i++) {
    newDigitsData |= ((digits[i] & 0xF) << (3 - i) * 4);
  }
  if (newDigitsData != digitsData) {
    digitsData = newDigitsData;
    digitalWrite(SHIFT_RCLK, LOW);
    shiftOut(SHIFT_SER, SHIFT_SRCLK, MSBFIRST, digitsData >> 8);
    shiftOut(SHIFT_SER, SHIFT_SRCLK, MSBFIRST, digitsData & 0xFF);
    digitalWrite(SHIFT_RCLK, HIGH);
  }
}

void toDigitsArray(uint8_t number1, uint8_t number2, uint8_t out[4]) {
  out[0] = number1 / 10;
  out[1] = number1 % 10;
  out[2] = number2 / 10;
  out[3] = number2 % 10;
}

void setDigits(uint8_t number1, uint8_t number2) {
  toDigitsArray(number1, number2, digits);
  updateDigits();
}

void updatePWM() {
  if (displayState == DisplayState::DePoison) {
    for (int i = 0; i < 5; i++) {
      analogWrite(pwmPins[i], 255);
    }
    return;
  }
  uint8_t pwmBrightness = brightness * 255 / 9;
  if (editMode && displayState == DisplayState::Brightness) {
    pwmBrightness = editValues[1] * 255 / 9;
  }
  uint8_t pwmPattern = pwmPatterns[displayState][pwmPatternIndex];
  for (int i = 0; i < 5; i++) {
    uint8_t mask = 1 << (4 - i);
    if (pwmPattern & mask) {
      if (editMode && !(editPwmMask[editIndex] & mask)) {
        // In edit mode, set non-selected digits to be slightly dimmer
        analogWrite(pwmPins[i], pwmBrightness / 4);
      } else {
        analogWrite(pwmPins[i], pwmBrightness);
      }
    } else {
      analogWrite(pwmPins[i], 0);
    }
  }
}

void getDisplayValues(uint8_t values[2]) {
  switch (displayState) {
    case DisplayState::Time:
      values[0] = dateTime.hour();
      values[1] = dateTime.minute();
      break;
    case DisplayState::TimeSeconds:
      values[0] = dateTime.minute();
      values[1] = dateTime.second();
      break;
    case DisplayState::Date:
      values[0] = dateTime.month();
      values[1] = dateTime.day();
      break;
    case DisplayState::Year:
      values[0] = 20;
      values[1] = dateTime.year() - 2000;
      break;
    case DisplayState::Brightness:
      values[0] = 10;
      values[1] = brightness;
      break;
    case DisplayState::DePoison:
      values[0] = dateTime.hour();
      values[1] = dateTime.minute();
      break;
  }
}


void updateDisplay() {
  if (editMode) {
    setDigits(editValues[0], editValues[1]);
  } else if (displayState == DisplayState::DePoison) {
    unsigned long deltaTime = millis() - lastUpdateTime;
    if (deltaTime > 20) {
      lastUpdateTime = millis();
      dePoisonTime += deltaTime;
      if (dePoisonTime >= dePoisonTimes[dePoisonTimeIndex]) {
        if (dePoisonTimeIndex < 4) {
          dePoisonDone[dePoisonPermutation[dePoisonTimeIndex]] = true;
          toDigitsArray(dateTime.hour(), dateTime.minute(), digits);
          dePoisonTimeIndex++;
        } else {
          dePoisonReset();
          return;
        }
      }
      for (int i = 0; i < 4; i++) {
        if (!dePoisonDone[i]) {
          digits[i] = random(0, 10);
        }
      }
      updateDigits();
    }
  } else {
    getDisplayValues(displayValues);
    setDigits(displayValues[0], displayValues[1]);
  }
}

void enterEditMode() {
  getDisplayValues(editValues);
  editMode = true;
  setEditIndex(1);
}

void exitEditMode(bool save) {
  if (save) {
    if (displayState == DisplayState::Brightness) {
      brightness = editValues[1];
    } else {
      dateTime = rtc.now();
      uint16_t year = dateTime.year();
      uint8_t month = dateTime.month();
      uint8_t day = dateTime.day();
      uint8_t hour = dateTime.hour();
      uint8_t minute = dateTime.minute();
      uint8_t second = dateTime.second();
      switch (displayState) {
        case DisplayState::Time:
          hour = editValues[0];
          minute = editValues[1];
          break;
        case DisplayState::TimeSeconds:
          minute = editValues[0];
          second = editValues[1];
          break;
        case DisplayState::Date:
          month = editValues[0];
          day = editValues[1];
          break;
        case DisplayState::Year:
          year = 2000 + editValues[1];
          break;
      }
      dateTime = DateTime(year, month, day, hour, minute, second);
      rtc.adjust(dateTime);
    }
  }
  editMode = false;
  updateDisplay();
  updatePWM();
}

void setEditIndex(int index) {
  // Use first valid edit index
  for (int i = 0; i < 2; i++) {
    if (editRanges[displayState][index].min == 0 &&
        editRanges[displayState][index].max == 0) {
      index = (index + 1) % 2;
      continue;
    }
    break;
  }

  editIndex = index;
  updatePWM();
}

void setDisplayState(DisplayState newDisplayState) {
  displayState = newDisplayState;
  if (displayState == DisplayState::DePoison) {
    dePoisonReset();
  }
  updateDisplay();
  updatePWM();
}

void dePoisonReset() {
  for (int i = 0; i < 4; i++) {
    dePoisonDone[i] = false;
  }
  lastUpdateTime = millis();
  dePoisonTime = 0;
  dePoisonTimeIndex = 0;
  // Randomize permutation (i.e. order that digits finish)
  for (int i = 0; i < 3; i++) {
    int j = random(i, 4);
    int tmp = dePoisonPermutation[j];
    dePoisonPermutation[j] = dePoisonPermutation[i];
    dePoisonPermutation[i] = tmp;
  }
}

void loop() {
  // Update time
  dateTime = rtc.now();
  bool timeChanged = lastSecond != dateTime.second();
  lastSecond = dateTime.second();

  // Check encoder
  encoder.tick();
  int encoderDelta = (encoder.getPosition() - lastEncoderPosition);
  lastEncoderPosition = encoder.getPosition();

  // Update pwm pattern
  if (timeChanged) {
    const int numPatterns = (sizeof(pwmPatterns[0]) / sizeof(pwmPatterns[0][0]));
    pwmPatternIndex = (pwmPatternIndex + 1) % numPatterns;
    updatePWM();
  }

  if (!editMode) {
    if (timeChanged || displayState == DisplayState::DePoison) {
      updateDisplay();
    }
    if (buttonA.pressed() && displayState != DisplayState::DePoison) {
      enterEditMode();
      return;
    }
    if (encoderDelta != 0) {
      if (encoderDelta < 0) {
        encoderDelta = encoderDelta - encoderDelta * DisplayState::NumStates;
      }
      setDisplayState((displayState + encoderDelta) % DisplayState::NumStates);
    }
  } else {
    if (buttonA.pressed()) {
      exitEditMode(true);
      return;
    }
    if (buttonB.pressed()) {
      exitEditMode(false);
      return;
    }
    if (buttonEncoder.pressed()) {
      setEditIndex((editIndex + 1) % 2);
      return;
    }

    if (encoderDelta != 0) {
      int value = editValues[editIndex];
      int min = editRanges[displayState][editIndex].min;
      int max = editRanges[displayState][editIndex].max;
      int mod = max + 1 - min;
      if (encoderDelta < 0) {
        encoderDelta = encoderDelta - encoderDelta * mod;
      }
      editValues[editIndex] = (value - min + encoderDelta) % mod + min;
      updateDisplay();
      if (displayState == DisplayState::Brightness) {
        updatePWM();
      }
    }
  }
}
