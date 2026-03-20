/*
  =====================================================
  4-Way Intersection — Ambulance Priority + Countdowns
  =====================================================
  Ambulance direction : SOUTH
  Normal cycle        : N → E → S → W (10s each)

  STATES:
    NORMAL         — cycling N→E→S→W, displays count down
    AMBUL_WARNING  — ambulance detected, 5s countdown
                     before South goes green
    AMBUL_ACTIVE   — South green, others red + dashes
    AMBUL_COOLDOWN — ambulance passed, 5s cooldown,
                     then normal cycle resumes

  ── SAFE PIN MAP (ESP32-C6) ──────────────────────────
  Pot SIG        → 4   (ADC, safe for analog in)

  North  GREEN   → 1   RED → 2
  East   GREEN   → 3   RED → 6
  South  GREEN   → 7   RED → 10
  West   GREEN   → 11  RED → 18

  TM1637 North   CLK → 19   DIO → 20
  TM1637 East    CLK → 21   DIO → 22
  TM1637 South   CLK → 23   DIO → 0
  TM1637 West    CLK → 5    DIO → 15
  (pins 5 & 15 are strapping but safe for output after boot)
  =====================================================
*/

#include <TM1637Display.h>

// ─── POT ───────────────────────────────────────────
#define POT_PIN          4
const int NUM_SAMPLES  = 20;
const int DEADBAND     = 30;
const int ALERT_THRESHOLD = 2100;

// ─── LED PINS ──────────────────────────────────────
#define N_GREEN  1
#define N_RED    2
#define E_GREEN  3
#define E_RED    6
#define S_GREEN  7
#define S_RED    10
#define W_GREEN  11
#define W_RED    18

const int GREEN_PINS[4] = { N_GREEN, E_GREEN, S_GREEN, W_GREEN };
const int RED_PINS[4]   = { N_RED,   E_RED,   S_RED,   W_RED   };

#define IDX_N 0
#define IDX_E 1
#define IDX_S 2
#define IDX_W 3

// ─── TM1637 DISPLAYS ───────────────────────────────
//                    CLK  DIO
TM1637Display dispN( 19,  20 );
TM1637Display dispE( 21,  22 );
TM1637Display dispS( 23,   0 );
TM1637Display dispW(  5,  15 );

TM1637Display* displays[4] = { &dispN, &dispE, &dispS, &dispW };
const char*    DIR_NAMES[4] = { "NORTH", "EAST", "SOUTH", "WEST" };

// ─── TIMING ────────────────────────────────────────
const unsigned long NORMAL_PHASE_MS = 10000;
const unsigned long WARNING_MS      =  5000;
const unsigned long COOLDOWN_MS     =  5000;

// ─── STATE MACHINE ─────────────────────────────────
enum State { NORMAL, AMBUL_WARNING, AMBUL_ACTIVE, AMBUL_COOLDOWN };
State state = NORMAL;

int           normalPhase    = IDX_N;
unsigned long phaseStartMs   = 0;
unsigned long stateStartMs   = 0;
int           lastStableADC  = -1;
bool          ambulanceNear  = false;

// ───────────────────────────────────────────────────

int smoothRead(int pin) {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delay(1);
  }
  return (int)(sum / NUM_SAMPLES);
}

void setOnlyGreen(int idx) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(GREEN_PINS[i], i == idx ? HIGH : LOW);
    digitalWrite(RED_PINS[i],   i == idx ? LOW  : HIGH);
  }
}

void allRed() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(GREEN_PINS[i], LOW);
    digitalWrite(RED_PINS[i],   HIGH);
  }
}

void showDashes(TM1637Display* d) {
  uint8_t seg[] = { SEG_G, SEG_G, SEG_G, SEG_G };
  d->setSegments(seg);
}

void showSeconds(TM1637Display* d, int secs) {
  if (secs < 0) secs = 0;
  d->showNumberDec(secs, false);
}

// ─── DISPLAY UPDATE ────────────────────────────────
void updateDisplays() {
  unsigned long now     = millis();
  unsigned long elapsed = now - phaseStartMs;

  switch (state) {

    case NORMAL: {
      unsigned long phaseLeft = 0;
      if (elapsed < NORMAL_PHASE_MS)
        phaseLeft = (NORMAL_PHASE_MS - elapsed) / 1000;

      for (int i = 0; i < 4; i++) {
        if (i == normalPhase) {
          showSeconds(displays[i], (int)phaseLeft);
        } else {
          int phasesAway = (i - normalPhase + 4) % 4;
          long waitSecs  = (long)phaseLeft + (long)(phasesAway - 1) * (NORMAL_PHASE_MS / 1000);
          showSeconds(displays[i], (int)waitSecs);
        }
      }
      break;
    }

    case AMBUL_WARNING: {
      unsigned long warnElapsed = now - stateStartMs;
      int secsLeft = (int)((WARNING_MS - warnElapsed) / 1000);
      for (int i = 0; i < 4; i++) {
        if (i == IDX_S) showSeconds(displays[i], secsLeft);
        else            showDashes(displays[i]);
      }
      break;
    }

    case AMBUL_ACTIVE: {
      for (int i = 0; i < 4; i++) {
        if (i == IDX_S) showSeconds(displays[i], 0);
        else            showDashes(displays[i]);
      }
      break;
    }

    case AMBUL_COOLDOWN: {
      unsigned long cdElapsed = now - stateStartMs;
      int secsLeft = (int)((COOLDOWN_MS - cdElapsed) / 1000);
      for (int i = 0; i < 4; i++) {
        if (i == IDX_S) showSeconds(displays[i], secsLeft);
        else            showDashes(displays[i]);
      }
      break;
    }
  }
}

// ───────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(GREEN_PINS[i], OUTPUT);
    pinMode(RED_PINS[i],   OUTPUT);
  }

  for (int i = 0; i < 4; i++) {
    displays[i]->setBrightness(5);
    showDashes(displays[i]);
  }

  allRed();
  delay(500);

  phaseStartMs = millis();
  setOnlyGreen(IDX_N);
  state = NORMAL;

  Serial.println("=== 4-Way Intersection — Ambulance from SOUTH ===");
  Serial.println("Pot RIGHT → ambulance <100m approaching");
  Serial.println("Pot LEFT  → normal cycle N→E→S→W (10s each)");
  Serial.println("-------------------------------------------------");
  Serial.println("[NORMAL] NORTH GREEN");
}

// ───────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  // ── Potentiometer read ─────────────────────────
  int adc = smoothRead(POT_PIN);
  if (abs(adc - lastStableADC) > DEADBAND) {
    lastStableADC = adc;
    bool newAmbul = (adc >= ALERT_THRESHOLD);
    int  dist     = constrain(map(adc, 0, 4095, 200, 0), 0, 200);

    if (newAmbul != ambulanceNear) {
      ambulanceNear = newAmbul;
      Serial.print("ADC: ");   Serial.print(adc);
      Serial.print("  dist: ~"); Serial.print(dist);
      Serial.println(ambulanceNear
        ? "m  → 🚨 AMBULANCE DETECTED"
        : "m  → 🟢 Ambulance cleared");
    }
  }

  // ── State machine ──────────────────────────────
  switch (state) {

    case NORMAL:
      if (now - phaseStartMs >= NORMAL_PHASE_MS) {
        normalPhase  = (normalPhase + 1) % 4;
        phaseStartMs = now;
        setOnlyGreen(normalPhase);
        Serial.print("[NORMAL] "); Serial.print(DIR_NAMES[normalPhase]); Serial.println(" GREEN");
      }
      if (ambulanceNear) {
        Serial.println("[WARNING] Ambulance <100m — South green in 5s...");
        allRed();
        state        = AMBUL_WARNING;
        stateStartMs = now;
      }
      break;

    case AMBUL_WARNING:
      if (!ambulanceNear) {
        Serial.println("[NORMAL] Ambulance retreated — resuming");
        state        = NORMAL;
        phaseStartMs = now;
        setOnlyGreen(normalPhase);
        break;
      }
      if (now - stateStartMs >= WARNING_MS) {
        Serial.println("[AMBUL] SOUTH GREEN — ambulance crossing!");
        setOnlyGreen(IDX_S);
        state        = AMBUL_ACTIVE;
        stateStartMs = now;
      }
      break;

    case AMBUL_ACTIVE:
      if (!ambulanceNear) {
        Serial.println("[COOLDOWN] Ambulance passed — 5s cooldown...");
        allRed();
        state        = AMBUL_COOLDOWN;
        stateStartMs = now;
      }
      break;

    case AMBUL_COOLDOWN:
      if (ambulanceNear) {
        Serial.println("[WARNING] Ambulance back — restarting warning");
        state        = AMBUL_WARNING;
        stateStartMs = now;
        break;
      }
      if (now - stateStartMs >= COOLDOWN_MS) {
        normalPhase  = (IDX_S + 1) % 4;
        phaseStartMs = now;
        setOnlyGreen(normalPhase);
        state        = NORMAL;
        Serial.print("[NORMAL] Resuming → "); Serial.print(DIR_NAMES[normalPhase]); Serial.println(" GREEN");
      }
      break;
  }

  updateDisplays();
  delay(50);
}
