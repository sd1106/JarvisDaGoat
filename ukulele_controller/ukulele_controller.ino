/*
  ukulele_controller.ino
  Self-playing ukulele — ESP32-S3 firmware (serial-driven instrument driver)
  ===========================================================================
  TARGET: ESP32-S3-WROOM-1  (NOT classic ESP32 / ESP32-S2 — pins differ!)
  LIBRARY: ESP32Servo (madhephaestus, install via Library Manager)
           Requires Arduino-ESP32 core v3.0.0+. The S3 has only 8 LEDC
           channels (vs 16 on classic ESP32); recent ESP32Servo spills the
           extra servos onto the S3's MCPWM units so all 8 work.

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

  Each string has THREE actuators:
    • a 28BYJ-48 stepper that slides the fretting carriage along the neck
    • a fret servo that presses the string onto the fretboard at that position
    • a strum servo that plucks the string (alternates side each strum)

  Serial commands (115200 baud, newline-terminated)
  -------------------------------------------------
    FRET <str> <fret>          move carriage to <fret> (0..12) and press
    RELEASE <str>              lift the fret servo (open string)
    STRUM <str>                strum one string (auto-alternates direction)
    CHORD <f0> <f1> <f2> <f3>  fret all four strings at once (-1 = open)
    STRUMALL                   strum all four strings low→high
    HOME                       carriages to step 0, all frets released
    CAL <str> <steps>          jog carriage to absolute <steps> (calibration)
    CAL <str> ZERO             define current carriage position as step 0
    DUMP                       print live state + pin map
  (unknown commands are silently ignored)

  On boot the firmware homes/zeroes all carriages then prints "READY\n"
  (kept identical so the existing Python host's open_serial() still works).
*/

#include <Arduino.h>
#include <ESP32Servo.h>

// ── Pin definitions (ESP32-S3-WROOM-1 — see wiring table above) ───────────────

const uint8_t N_STRINGS = 4;

// Stepper coil pins: STEP_PIN[string][IN1..IN4] → ULN2003 IN1..IN4
const uint8_t STEP_PIN[N_STRINGS][4] = {
    {  4,  5,  6,  7 },   // String 0 (G) — ULN2003 #0
    { 15, 16, 17, 18 },   // String 1 (C) — ULN2003 #1
    {  8,  9, 10, 11 },   // String 2 (E) — ULN2003 #2
    { 12, 13, 14, 21 },   // String 3 (A) — ULN2003 #3
};

// Servo signal pins
const uint8_t FRET_SERVO_PIN[N_STRINGS]  = {  1,  2, 41, 42 };
const uint8_t STRUM_SERVO_PIN[N_STRINGS] = { 39, 40, 47, 48 };

// ── Fret step lookup table ────────────────────────────────────────────────────
// Absolute carriage position (in 28BYJ-48 half-steps) for each fret.
// Spacing follows equal-temperament fret geometry — steps[n] = K·(1 − 2^(−n/12))
// with K chosen so fret 12 (the octave) lands at 1024 half-steps. Frets bunch up
// as you climb the neck, exactly like a real fretboard. Re-calibrate with CAL.
const long FRET_STEPS[] = {
    0,    // open
    115,  // fret 1
    223,  // fret 2
    326,  // fret 3
    423,  // fret 4
    514,  // fret 5
    600,  // fret 6
    681,  // fret 7
    758,  // fret 8
    830,  // fret 9
    899,  // fret 10
    963,  // fret 11
    1024  // fret 12
};
const uint8_t MAX_FRET = (sizeof(FRET_STEPS) / sizeof(FRET_STEPS[0])) - 1;

// ── Servo angle constants ─────────────────────────────────────────────────────
const int FRET_REST   = 0;    // fret servo released (0°)
const int FRET_PRESS  = 90;   // fret servo pressing string down (90°)
const int STRUM_A     = 60;   // strum servo — one side of neutral
const int STRUM_B     = 120;  // strum servo — the other side of neutral

// ── Motion parameters ─────────────────────────────────────────────────────────
const uint16_t STEP_DELAY_US = 1200;  // per half-step; 28BYJ-48 is happy ~1–2 ms
const uint16_t FRET_SETTLE_MS = 120;  // let the fret servo seat before strumming
const uint16_t STRUM_SETTLE_MS = 90;  // let the strum servo finish its sweep

// ── Hardware objects ──────────────────────────────────────────────────────────
Servo fretServo[N_STRINGS];
Servo strumServo[N_STRINGS];

// ── State ─────────────────────────────────────────────────────────────────────
long    carriagePos[N_STRINGS] = { 0, 0, 0, 0 };   // current absolute half-steps
uint8_t phaseIdx[N_STRINGS]    = { 0, 0, 0, 0 };    // index into HALF_STEP[]
bool    fretPressed[N_STRINGS] = { false, false, false, false };
bool    strumToggle[N_STRINGS] = { false, false, false, false }; // last strum side

// 28BYJ-48 half-step sequence. Bit3=IN1, Bit2=IN2, Bit1=IN3, Bit0=IN4.
const uint8_t HALF_STEP[8] = {
    0b1000, 0b1100, 0b0100, 0b0110,
    0b0010, 0b0011, 0b0001, 0b1001
};

// ── Low-level stepper helpers ─────────────────────────────────────────────────

// Energise the four coils of string s according to half-step phase `idx`.
void applyPhase(uint8_t s, uint8_t idx) {
    uint8_t bits = HALF_STEP[idx & 7];
    for (uint8_t p = 0; p < 4; p++) {
        digitalWrite(STEP_PIN[s][p], (bits >> (3 - p)) & 0x01);
    }
}

// De-energise all coils of string s (prevents the ULN2003 from cooking while
// idle — the gearbox friction + fret servo hold position, not the coils).
void releaseCoils(uint8_t s) {
    for (uint8_t p = 0; p < 4; p++) digitalWrite(STEP_PIN[s][p], LOW);
}

// Drive string s's carriage to an absolute half-step position.
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

// ── Fret / strum helpers ──────────────────────────────────────────────────────

void releaseFretString(uint8_t s) {
    fretServo[s].write(FRET_REST);
    fretPressed[s] = false;
}

// Move carriage to `fret` and press the string down. fret < 0 = open string.
void fretString(uint8_t s, int fret) {
    if (fret < 0) {
        releaseFretString(s);
        return;
    }
    if (fret > MAX_FRET) fret = MAX_FRET;
    moveCarriage(s, FRET_STEPS[fret]);
    fretServo[s].write(FRET_PRESS);
    fretPressed[s] = true;
}

// Strum one string. The servo flips to the OPPOSITE side every call, so two
// consecutive strums are always in opposite directions — never the same way
// twice in a row.
void strumString(uint8_t s) {
    strumToggle[s] = !strumToggle[s];
    strumServo[s].write(strumToggle[s] ? STRUM_B : STRUM_A);
    delay(STRUM_SETTLE_MS);
}

// Fret an entire chord. fret[] holds 4 values, one per string (-1 = open).
void fretChord(const int fret[N_STRINGS]) {
    for (uint8_t s = 0; s < N_STRINGS; s++) fretString(s, fret[s]);
    delay(FRET_SETTLE_MS);
}

void homeAll() {
    for (uint8_t s = 0; s < N_STRINGS; s++) {
        releaseFretString(s);
        moveCarriage(s, 0);
    }
}

// ── Diagnostics ───────────────────────────────────────────────────────────────

void dumpState() {
    Serial.println(F("── ukulele state ─────────────────────────────"));
    Serial.println(F("str  carriage  fret  pressed  strumSide"));
    for (uint8_t s = 0; s < N_STRINGS; s++) {
        // nearest fret for the current carriage position (display only)
        uint8_t nf = 0;
        for (uint8_t f = 0; f <= MAX_FRET; f++)
            if (labs(FRET_STEPS[f] - carriagePos[s]) <
                labs(FRET_STEPS[nf] - carriagePos[s])) nf = f;

        Serial.printf("  %u  %8ld   %3u    %-5s     %c\n",
                      s, carriagePos[s], nf,
                      fretPressed[s] ? "yes" : "no",
                      strumToggle[s] ? 'B' : 'A');
    }
    Serial.println(F("pins  IN1/IN2/IN3/IN4   fretSrv  strumSrv"));
    for (uint8_t s = 0; s < N_STRINGS; s++) {
        Serial.printf("  %u   %2u/%2u/%2u/%2u       %3u      %3u\n",
                      s, STEP_PIN[s][0], STEP_PIN[s][1], STEP_PIN[s][2],
                      STEP_PIN[s][3], FRET_SERVO_PIN[s], STRUM_SERVO_PIN[s]);
    }
    Serial.println(F("──────────────────────────────────────────────"));
}

// ── Command parser ────────────────────────────────────────────────────────────

void handleCommand(const String& cmd) {
    if (cmd.startsWith("FRET ")) {
        int sp = cmd.indexOf(' ', 5);
        uint8_t s = (uint8_t) cmd.substring(5, sp).toInt();
        int fret  = cmd.substring(sp + 1).toInt();
        if (s < N_STRINGS) fretString(s, fret);

    } else if (cmd.startsWith("RELEASE ")) {
        uint8_t s = (uint8_t) cmd.substring(8).toInt();
        if (s < N_STRINGS) releaseFretString(s);

    } else if (cmd.startsWith("STRUM ")) {
        uint8_t s = (uint8_t) cmd.substring(6).toInt();
        if (s < N_STRINGS) strumString(s);

    } else if (cmd == "STRUMALL") {
        for (uint8_t s = 0; s < N_STRINGS; s++) strumString(s);

    } else if (cmd.startsWith("CHORD ")) {
        // CHORD <f0> <f1> <f2> <f3>
        int fret[N_STRINGS];
        int idx = 6;
        for (uint8_t s = 0; s < N_STRINGS; s++) {
            int sp = cmd.indexOf(' ', idx);
            String tok = (sp < 0) ? cmd.substring(idx) : cmd.substring(idx, sp);
            fret[s] = tok.toInt();
            idx = (sp < 0) ? cmd.length() : sp + 1;
        }
        fretChord(fret);

    } else if (cmd == "HOME") {
        homeAll();
        Serial.println("HOMED");

    } else if (cmd.startsWith("CAL ")) {
        // CAL <str> <steps>   or   CAL <str> ZERO
        int sp = cmd.indexOf(' ', 4);
        uint8_t s = (uint8_t) cmd.substring(4, sp).toInt();
        String arg = cmd.substring(sp + 1);
        arg.trim();
        if (s < N_STRINGS) {
            if (arg == "ZERO") {
                carriagePos[s] = 0;                 // redefine home here
                Serial.printf("CAL str %u zeroed\n", s);
            } else {
                moveCarriage(s, arg.toInt());       // jog to absolute steps
                Serial.printf("CAL str %u @ %ld steps\n", s, carriagePos[s]);
            }
        }

    } else if (cmd == "DUMP") {
        dumpState();
    }
    // Unknown commands silently ignored
}

// ── Arduino setup & loop ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    // ESP32Servo: reserve all four LEDC timers up front so the 8 servos can be
    // distributed across LEDC + MCPWM on the S3.
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
        strumServo[s].write(STRUM_A);   // park strummers on side A
    }

    homeAll();

    Serial.println("READY");   // Python host waits for exactly this string
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) handleCommand(line);
    }
}
