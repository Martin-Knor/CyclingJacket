// Gods should strike me down for this monstrosity of a code

#include <Adafruit_NeoPixel.h>

// -----------------------------
// Zipper (variable resistor)
// -----------------------------
const int ZIPPER_PIN = A2;

// Calibration (from your measurements)
const int ZIPPER_CLOSED = 850;   // fully closed → bright
const int ZIPPER_OPEN   = 990;   // fully open   → dark

// Noise handling
const float ALPHA = 0.15;   // EMA smoothing factor
const int DEADBAND = 3;     // ADC counts


/////////////////////
// LED Segments    //
/////////////////////

struct LedSegment {
  uint8_t start;
  uint8_t end;
  bool forward;
};

enum SegmentID {
  HAZARD_LIGHTS,
  TURN_RIGHT,
  TURN_LEFT,
};

const LedSegment LED_SEGMENTS[] = {
  { 0, 23, true  },   // LEDs 13–30
  { 24, 30, false },  // LEDs 1–6
  { 31, 37, true  },  // LEDs 7–12
};

constexpr uint8_t LED_RIGHT_INDICATOR  = 38;
constexpr uint8_t LED_HAZARD_INDICATOR = 39;
constexpr uint8_t LED_LEFT_INDICATOR   = 40;


/////////////////////
// NeoPixel config //
/////////////////////

constexpr uint8_t PIN = 4;
constexpr uint16_t NUM_LEDS = 41;

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

/////////////////////
//    Constants    //
/////////////////////

const int CAP_THRESHOLD = 500;
const int RES_THRESHOLD = 250;

const unsigned long CROSS_WINDOW   = 1000;
const unsigned long WIPE_STEP_TIME = 80;
const unsigned long WIPE_DURATION  = 3000;

const unsigned long RES_HOLD_TIME  = 500;

bool hazardActive = false;
bool hazardPhase = false;
unsigned long hazardLastToggle = 0;

const unsigned long HAZARD_BLINK_INTERVAL = 400;



/////////////////////
// Sensor handling //
/////////////////////

struct ZPatch {
  uint8_t pinA = 0;
  uint8_t pinB = 0;

  int baseline = 0;
  int capValue = 0;
  int capPrev  = 0;
  float smoothFactor = 0.2f;
  bool capAbove = false;

  int resValue = 0;
  bool resBelow = false;
  bool resConsumed = false;
  unsigned long resBelowSince = 0;
    ZPatch(uint8_t a, uint8_t b)
    : pinA(a), pinB(b) {}
};

ZPatch patch1{ A6, A7 };
ZPatch patch2{ A4, A5 };

struct CrossDetector {
  bool pending = false;
  uint8_t first = 0;
  unsigned long time = 0;
};

CrossDetector cross;

struct WipeAnimator {
  bool active = false;
  bool finishing = false;
  unsigned long until = 0;
  uint8_t start = 0;
  uint8_t end = 0;
  bool forward = true;
};


WipeAnimator animation;

/////////////////////
// Arduino setup   //
/////////////////////

void setup() {
  Serial.begin(9600);
  delay(100);

  calibrateSensor(patch1);
  calibrateSensor(patch2);

  Serial.println("");
  Serial.print("baseline 1: ");
  Serial.println(patch1.baseline);
  Serial.print("baseline 2: ");
  Serial.println(patch2.baseline);

  strip.begin();
  strip.show();
}

/////////////////////
// Main loop       //
/////////////////////

int loopCount = 0;

void loop() {

  loopCount++;

  unsigned long now = millis();
  uint8_t brightness = readZipperBrightness();
  strip.setBrightness(brightness);
  updateWipeAnimation();   // affects 1–12 only
  updateHazardLights();    // affects 13–30 only
  

  updateSensor(patch1);
  updateSensor(patch2);

  if (loopCount==30)
  {
    patch1.baseline += patch1.capValue;
    patch2.baseline += patch2.capValue;
  }

  if (resistiveHeld(patch2, now)) {
    hazardActive = !hazardActive;
    clearStripPart(LED_SEGMENTS[HAZARD_LIGHTS].start, LED_SEGMENTS[HAZARD_LIGHTS].end);
  }

  handleCross(patch1, 1, 2, TURN_RIGHT, now);
  handleCross(patch2, 2, 1, TURN_LEFT,  now);

  if (cross.pending && now - cross.time > CROSS_WINDOW) {
    cross.pending = false;
  }

  // optional serial plotting
  Serial.print(CAP_THRESHOLD);
  Serial.print(", ");
  Serial.print(RES_THRESHOLD);
  Serial.print(", ");
  Serial.print(patch1.capValue);
  Serial.print(", ");
  Serial.print(patch1.resValue);
  Serial.print(", ");
  Serial.print(patch2.capValue);
  Serial.print(", ");
  Serial.println(patch2.resValue);
}

void calibrateSensor(ZPatch& zp) {
  zp.baseline = capacitiveRead(zp.pinA, zp.pinB, 10);
}

void updateSensor(ZPatch& zp) {
  int raw = capacitiveRead(zp.pinA, zp.pinB, 10) - zp.baseline;
  zp.capValue = zp.capPrev + zp.smoothFactor * (raw - zp.capPrev);
  zp.capPrev  = zp.capValue;

  zp.resValue = dualAnalogRead(zp.pinA, zp.pinB, 3);
}

/////////////////////
// Resistive logic //
/////////////////////

bool resistiveHeld(ZPatch& zp, unsigned long now) {
  if (zp.resValue < RES_THRESHOLD) {
    if (!zp.resBelow) {
      zp.resBelow = true;
      zp.resBelowSince = now;
      zp.resConsumed = false;
    }

    if (!zp.resConsumed && now - zp.resBelowSince >= RES_HOLD_TIME) {
      zp.resConsumed = true;
      return true;
    }
  } else {
    zp.resBelow = false;
    zp.resConsumed = false;
  }
  return false;
}

/////////////////////
// Cross detection //
/////////////////////

void handleCross(ZPatch& current, uint8_t currentId, uint8_t otherId, SegmentID segment, unsigned long now) 
{
  bool nowAbove = current.capValue > CAP_THRESHOLD;

  if (nowAbove && !current.capAbove) {
    if (!cross.pending) {
      cross.pending = true;
      cross.first = currentId;
      cross.time = now;
    } else if (cross.first == otherId && now - cross.time <= CROSS_WINDOW) {
      startWipeAnimation(segment, WIPE_DURATION);
      
      cross.pending = false;
    }
  }

  current.capAbove = nowAbove;
}

/////////////////////
// LED animation   //
/////////////////////

void startWipeAnimation(SegmentID seg, unsigned long duration) 
{

  //stop current animation
  clearStripPart(animation.start, animation.end);

  animation.start   = LED_SEGMENTS[seg].start;
  animation.end     = LED_SEGMENTS[seg].end;
  animation.forward = LED_SEGMENTS[seg].forward;
  animation.active  = true;
  animation.until   = millis() + duration;
}

void updateWipeAnimation() 
{
  if (!animation.active) return;

  unsigned long now = millis();

  int length = animation.end - animation.start + 1;
  int step   = (now / WIPE_STEP_TIME) % length;

  // Mark expiration but allow loop to finish
  if (!animation.finishing && now > animation.until) {
    animation.finishing = true;
    strip.setPixelColor(LED_RIGHT_INDICATOR, strip.Color(0, 0, 0));
    strip.setPixelColor(LED_LEFT_INDICATOR, strip.Color(0, 0, 0));
  }

  // End only at loop boundary
  if (animation.finishing && step == length - 1) {
    animation.active = false;
    animation.finishing = false;
    clearStripPart(animation.start, animation.end);
    strip.setPixelColor(LED_RIGHT_INDICATOR, strip.Color(0, 0, 0));
    strip.setPixelColor(LED_LEFT_INDICATOR, strip.Color(0, 0, 0));
    return;
  }

  // Clear segment
  for (int i = animation.start; i <= animation.end; i++) {
    strip.setPixelColor(i, 0);
  }

  int a, b;
  if (animation.forward) {
    a = animation.start + step;
    b = animation.start + ((step + 1) % length);
  } else {
    a = animation.end - step;
    b = animation.end - ((step + 1) % length);
  }

  strip.setPixelColor(a, strip.Color(255, 60, 0));
  strip.setPixelColor(b, strip.Color(255, 60, 0));
  updateTurnIndicatorLeds();
  strip.show();
}


void updateHazardLights() {
  const LedSegment& seg = LED_SEGMENTS[HAZARD_LIGHTS];
  
  if (!hazardActive) {
    strip.setPixelColor(LED_HAZARD_INDICATOR, 0);
    strip.show();
    return;
  }

  unsigned long now = millis();
  if (now - hazardLastToggle < HAZARD_BLINK_INTERVAL) return;
  hazardLastToggle = now;

  hazardPhase = !hazardPhase;

  uint32_t color = hazardPhase ? strip.Color(255, 5, 0) : 0;

  for (int i = seg.start; i <= seg.end; i++) {
    strip.setPixelColor(i, color);
  }

  // 🔴 extra hazard LED
  strip.setPixelColor(LED_HAZARD_INDICATOR, color);

  strip.show();
}

void updateTurnIndicatorLeds() {
  uint32_t off = 0;

  strip.setPixelColor(LED_RIGHT_INDICATOR, off);
  strip.setPixelColor(LED_LEFT_INDICATOR,  off);

  if (!animation.active) return;

  // reuse hazard blink timing for consistency
  bool blinkOn = (millis() / HAZARD_BLINK_INTERVAL) % 2;
  if (!blinkOn) return;

  if (animation.start == LED_SEGMENTS[TURN_RIGHT].start) {
    strip.setPixelColor(LED_RIGHT_INDICATOR, strip.Color(255, 100, 0));
  }

  if (animation.start == LED_SEGMENTS[TURN_LEFT].start) {
    strip.setPixelColor(LED_LEFT_INDICATOR, strip.Color(255, 100, 0));
  }
}



void clearStripPart(int start, int end)
{
  for (int i = start; i <= end; i++) {  
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show();
}

/////////////////////
// Sensor internals//
/////////////////////

int capacitiveRead(int pinA, int pinB, int number) 
{
  int sum = 0;
  for (int i = 0; i < number; i++) {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT_PULLUP);
    ADMUX |= 0b11111;
    ADCSRA |= (1 << ADSC);
    while (!(ADCSRA & (1 << ADIF)));
    ADCSRA |= (1 << ADIF);
    pinMode(pinB, INPUT);
    sum += analogRead(pinB);

    pinMode(pinB, INPUT);
    pinMode(pinA, INPUT_PULLUP);
    ADMUX |= 0b11111;
    ADCSRA |= (1 << ADSC);
    while (!(ADCSRA & (1 << ADIF)));
    ADCSRA |= (1 << ADIF);
    pinMode(pinA, INPUT);
    sum += analogRead(pinA);
  }
  return sum;
}

int dualAnalogRead(int pinA, int pinB, int number) 
{
  int resistance = 0;
  for (int i = 0; i < number; i++) {
    pinMode(pinA, OUTPUT);
    digitalWrite(pinA, LOW);
    pinMode(pinB, INPUT_PULLUP);
    int rb = analogRead(pinB); 

    pinMode(pinB, OUTPUT);
    digitalWrite(pinB, LOW);
    pinMode(pinA, INPUT_PULLUP);
    int ra = analogRead(pinA);

    resistance += (ra + rb) / 2;
  }
  return resistance / number;
}

// -----------------------------
// Zipper → brightness function
// -----------------------------
uint8_t readZipperBrightness() {
  static float filtered = 0;
  static int lastStable = 0;
  static bool initialized = false;

  int raw = analogRead(ZIPPER_PIN);

  // Initialize filter on first run
  if (!initialized) {
    filtered = raw;
    lastStable = raw;
    initialized = true;
  }

  // Exponential moving average
  filtered = ALPHA * raw + (1.0 - ALPHA) * filtered;
  int f = (int)(filtered + 0.5);

  // Deadband / hysteresis
  if (abs(f - lastStable) < DEADBAND) {
    f = lastStable;
  } else {
    lastStable = f;
  }

  // Clamp to calibrated range
  f = constrain(f, ZIPPER_CLOSED, ZIPPER_OPEN);

  // Map: closed → 255, open → 0
  return (uint8_t)map(f, ZIPPER_CLOSED, ZIPPER_OPEN, 255, 0);
}