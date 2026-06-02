/*
  ukulele_riptide.ino
  Self-playing ukulele — ESP32-S3 standalone performer ("Riptide" by Vance Joy)
  ============================================================================
  TARGET: ESP32-S3-WROOM-1  (NOT classic ESP32 / ESP32-S2 — pins differ!)
  LIBRARY: ESP32Servo (madhephaestus) on Arduino-ESP32 core v3.0.0+.
           The S3 has only 8 LEDC channels; recent ESP32Servo routes the
           overflow servos through the S3's MCPWM units so all 8 work.

  This sketch is fully self-contained: on boot it plays the Riptide chord
  progression on a loop, driving the same hardware as ukulele_controller.ino.
  No host PC required.

  ┌──────────────────────────────────────────────────────────────────────────┐
  │ ESP32-S3-WROOM-1  WIRING TABLE                                            │
  ├──────────────────────────────────────────────────────────────────────────┤
  │ STEPPERS — 28BYJ-48 via ULN2003 (one driver board per string)            │
  │   String 0 (G):  IN1→GPIO4    IN2→GPIO5    IN3→GPIO6    IN4→GPIO7         │
  │   String 1 (C):  IN1→GPIO15   IN2→GPIO16   IN3→GPIO17   IN4→GPIO18        │
  │   String 2 (E):  IN1→GPIO8    IN2→GPIO9    IN3→GPIO10   IN4→GPIO11        │
  │   String 3 (A):  IN1→GPIO12   IN2→GPIO13   IN3→GPIO14   IN4→GPIO21        │
  │                                                                          │
  │ FRET SERVOS — press string down (signal wire only)                       │
  │   String 0 (G): GPIO1     String 1 (C): GPIO2                            │
  │   String 2 (E): GPIO41    String 3 (A): GPIO42                           │
  │                                                                          │
  │ STRUM SERVOS — alternating strum (signal wire only)                      │
  │   String 0 (G): GPIO39    String 1 (C): GPIO40                           │
  │   String 2 (E): GPIO47    String 3 (A): GPIO48                           │
  │                                                                          │
  │ POWER: drive the ULN2003 boards + all 8 servos from an EXTERNAL 5 V      │
  │        supply. Tie its GND to the ESP32-S3 GND. Do NOT run motors off    │
  │        the board's 3V3 pin.                                              │
  │                                                                          │
  │ PINS DELIBERATELY AVOIDED on the S3:                                     │
  │   0,3,45,46 = strapping   19,20 = native USB   43,44 = UART0 console     │
  │   26–37 = SPI flash / octal PSRAM (WROOM-1 N16R8)   38 = onboard RGB LED │
  │   (GPIO 22,23,24,25 do not physically exist on the ESP32-S3.)            │
  └──────────────────────────────────────────────────────────────────────────┘

  Strings are tuned g-C-E-A (re-entrant), so string index → open pitch:
    0 = G   1 = C   2 = E   3 = A
*/

#include <Arduino.h>
#include <ESP32Servo.h>

// ── Pin definitions (ESP32-S3-WROOM-1 — see wiring table above) ───────────────

const uint8_t N_STRINGS = 4;

const uint8_t STEP_PIN[N_STRINGS][4] = {
    {  4,  5,  6,  7 },   // String 0 (G) — ULN2003 #0
    { 15, 16, 17, 18 },   // String 1 (C) — ULN2003 #1
    {  8,  9, 10, 11 },   // String 2 (E) — ULN2003 #2
    { 12, 13, 14, 21 },   // String 3 (A) — ULN2003 #3
};

const uint8_t FRET_SERVO_PIN[N_STRINGS]  = {  1,  2, 41, 42 };
const uint8_t STRUM_SERVO_PIN[N_STRINGS] = { 39, 40, 47, 48 };

// ── Fret step lookup table ────────────────────────────────────────────────────
// Absolute carriage position (28BYJ-48 half-steps) per fret, with equal-
// temperament spacing (steps[n] = K·(1 − 2^(−n/12)), fret 12 → 1024 steps).
const long FRET_STEPS[] = {
    0, 115, 223, 326, 423, 514, 600, 681, 758, 830, 899, 963, 1024
};
const uint8_t MAX_FRET = (sizeof(FRET_STEPS) / sizeof(FRET_STEPS[0])) - 1;

// ── Servo / motion constants ──────────────────────────────────────────────────
const int FRET_REST   = 0;
const int FRET_PRESS  = 90;
const int STRUM_A     = 60;
const int STRUM_B     = 120;

const uint16_t STEP_DELAY_US   = 1200;
const uint16_t FRET_SETTLE_MS  = 120;
const uint16_t STRUM_SETTLE_MS = 90;

// ── Hardware objects + state ──────────────────────────────────────────────────
Servo fretServo[N_STRINGS];
Servo strumServo[N_STRINGS];

long    carriagePos[N_STRINGS] = { 0, 0, 0, 0 };
uint8_t phaseIdx[N_STRINGS]    = { 0, 0, 0, 0 };
bool    strumToggle[N_STRINGS] = { false, false, false, false };

const uint8_t HALF_STEP[8] = {
    0b1000, 0b1100, 0b0100, 0b0110,
    0b0010, 0b0011, 0b0001, 0b1001
};

// ── Stepper + fret + strum helpers (shared with the controller sketch) ────────

void applyPhase(uint8_t s, uint8_t idx) {
    uint8_t bits = HALF_STEP[idx & 7];
    for (uint8_t p = 0; p < 4; p++)
        digitalWrite(STEP_PIN[s][p], (bits >> (3 - p)) & 0x01);
}

void releaseCoils(uint8_t s) {
    for (uint8_t p = 0; p < 4; p++) digitalWrite(STEP_PIN[s][p], LOW);
}

void moveCarriage(uint8_t s, long target) {
    long delta = target - carriagePos[s];
    int  dir   = (delta >= 0) ? 1 : -1;
    long steps = labs(delta);
    for (long i = 0; i < steps; i++) {
        phaseIdx[s] = (uint8_t)((phaseIdx[s] + dir + 8) & 7);
        applyPhase(s, phaseIdx[s]);
        delayMicroseconds(STEP_DELAY_US);
        carriagePos[s] += dir;
    }
    releaseCoils(s);
}

void fretString(uint8_t s, int fret) {
    if (fret < 0) { fretServo[s].write(FRET_REST); return; }   // open string
    if (fret > MAX_FRET) fret = MAX_FRET;
    moveCarriage(s, FRET_STEPS[fret]);
    fretServo[s].write(FRET_PRESS);
}

// Each call flips the strum servo to the opposite side — never the same
// direction twice in a row.
void strumString(uint8_t s) {
    strumToggle[s] = !strumToggle[s];
    strumServo[s].write(strumToggle[s] ? STRUM_B : STRUM_A);
    delay(STRUM_SETTLE_MS);
}

// One full strum across the instrument, low string → high string (a roll).
void strumChord() {
    for (uint8_t s = 0; s < N_STRINGS; s++) strumString(s);
}

// ── Riptide chord sequence ────────────────────────────────────────────────────
// Each chord lists the fretted position of every string (-1 = open string is
// not used here; open notes are fret 0). `strums` = how many times to strum it.
//   Am = G2 C0 E0 A0   |   G = G0 C2 E3 A2
//   C  = G0 C0 E0 A3   |   F = G2 C0 E1 A0
struct Chord {
    const char* name;
    int         frets[N_STRINGS];   // {G, C, E, A}
    uint8_t     strums;
};

const Chord RIPTIDE[] = {
    // ── Verse: Am – G – C (×2) ──
    { "Am", { 2, 0, 0, 0 }, 4 },
    { "G",  { 0, 2, 3, 2 }, 4 },
    { "C",  { 0, 0, 0, 3 }, 4 },
    { "C",  { 0, 0, 0, 3 }, 4 },
    { "Am", { 2, 0, 0, 0 }, 4 },
    { "G",  { 0, 2, 3, 2 }, 4 },
    { "C",  { 0, 0, 0, 3 }, 4 },
    { "C",  { 0, 0, 0, 3 }, 4 },
    // ── Chorus: F – C – G – Am ──
    { "F",  { 2, 0, 1, 0 }, 4 },
    { "C",  { 0, 0, 0, 3 }, 4 },
    { "G",  { 0, 2, 3, 2 }, 4 },
    { "Am", { 2, 0, 0, 0 }, 4 },
    { "F",  { 2, 0, 1, 0 }, 4 },
    { "C",  { 0, 0, 0, 3 }, 4 },
    { "G",  { 0, 2, 3, 2 }, 8 },   // hold the turnaround
};
const uint16_t N_CHORDS = sizeof(RIPTIDE) / sizeof(RIPTIDE[0]);

// Tempo: Riptide sits around 102 BPM. One strum per beat.
const float    BPM       = 102.0f;
const uint16_t BEAT_MS   = (uint16_t)(60000.0f / BPM);

void playChord(const Chord& c) {
    Serial.printf("[chord] %s\n", c.name);
    for (uint8_t s = 0; s < N_STRINGS; s++) fretString(s, c.frets[s]);
    delay(FRET_SETTLE_MS);

    for (uint8_t i = 0; i < c.strums; i++) {
        strumChord();
        // wait out the remainder of the beat (strumChord already burned some ms)
        long remaining = (long)BEAT_MS - (long)(N_STRINGS * STRUM_SETTLE_MS);
        if (remaining > 0) delay(remaining);
    }
}

// ── Arduino setup & loop ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    for (uint8_t s = 0; s < N_STRINGS; s++) {
        for (uint8_t p = 0; p < 4; p++) {
            pinMode(STEP_PIN[s][p], OUTPUT);
            digitalWrite(STEP_PIN[s][p], LOW);
        }
        fretServo[s].setPeriodHertz(50);
        fretServo[s].attach(FRET_SERVO_PIN[s], 500, 2400);
        strumServo[s].setPeriodHertz(50);
        strumServo[s].attach(STRUM_SERVO_PIN[s], 500, 2400);

        fretServo[s].write(FRET_REST);
        strumServo[s].write(STRUM_A);
        moveCarriage(s, 0);          // home the carriage
    }

    Serial.println("READY — playing Riptide");
}

void loop() {
    for (uint16_t i = 0; i < N_CHORDS; i++) playChord(RIPTIDE[i]);
    delay(1500);                     // breath before looping the song again
}
