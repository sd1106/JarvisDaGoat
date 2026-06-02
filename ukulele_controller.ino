/*
  ukulele_controller.ino
  Self-playing ukulele – Arduino firmware
  ========================================
  Receives newline-terminated ASCII commands from the host PC and drives:
    • 4 stepper motors (one per string) via A4988 / DRV8825 drivers
    • 4 pluck solenoids (one per string) via transistor/relay drivers

  Pin layout (change to match your wiring)
  -----------------------------------------
  String  | STEP  | DIR   | ENABLE | PLUCK
  --------|-------|-------|--------|------
    0 (G) |   2   |   3   |   4    |  22
    1 (C) |   5   |   6   |   7    |  23
    2 (E) |   8   |   9   |  10    |  24
    3 (A) |  11   |  12   |  13    |  25

  LIMIT switches (active-LOW, internal pull-up) used for homing:
    String 0 → pin 30,  1 → 31,  2 → 32,  3 → 33

  Commands accepted (from Python)
  --------------------------------
  MOVE <str> <steps>\n   – absolute position move
  PLUCK <str>\n           – pulse the pluck solenoid (~40 ms)
  HOME\n                  – run all carriages toward their limit switches
  WAIT <ms>\n             – pause <ms> ms before processing next command
  (anything else is silently ignored)

  On boot the firmware homes all carriages, then sends "READY\n".
*/

#include <Arduino.h>

// ── Pin definitions ──────────────────────────────────────────────────────────

const uint8_t N_STRINGS = 4;

// Stepper driver pins (one per string)
const uint8_t STEP_PIN[N_STRINGS]   = { 2,  5,  8, 11 };
const uint8_t DIR_PIN[N_STRINGS]    = { 3,  6,  9, 12 };
const uint8_t EN_PIN[N_STRINGS]     = { 4,  7, 10, 13 };

// Pluck solenoid pins (HIGH = activate)
const uint8_t PLUCK_PIN[N_STRINGS]  = { 22, 23, 24, 25 };

// Homing limit switch pins (INPUT_PULLUP, LOW = triggered)
const uint8_t LIMIT_PIN[N_STRINGS]  = { 30, 31, 32, 33 };

// ── Motion parameters ────────────────────────────────────────────────────────

// Microseconds between STEP pulses; lower = faster (but may miss steps).
// Start conservative and reduce once mechanics are tuned.
const uint16_t STEP_DELAY_US  = 800;

// HIGH pulse width for the STEP pin (most drivers need ≥ 1 µs)
const uint8_t  STEP_PULSE_US  = 10;

// Homing: move toward limit switch at slower speed
const uint16_t HOME_DELAY_US  = 1200;

// Pluck solenoid on-time in milliseconds
const uint16_t PLUCK_MS       = 40;

// After homing, back off this many steps so the carriage isn't sitting
// on the limit switch under load.
const int16_t  HOME_BACKOFF   = 20;

// ── State ────────────────────────────────────────────────────────────────────

long currentPos[N_STRINGS] = { 0, 0, 0, 0 };  // in steps from home

// ── Low-level stepper helpers ─────────────────────────────────────────────────

/**
 * Move string s by `steps` steps.
 * Positive = away from home (toward higher frets).
 * Negative = toward home.
 */
void stepMotor(uint8_t s, long steps) {
    if (steps == 0) return;

    // Direction
    if (steps > 0) {
        digitalWrite(DIR_PIN[s], HIGH);
    } else {
        digitalWrite(DIR_PIN[s], LOW);
        steps = -steps;
    }

    // Enable driver
    digitalWrite(EN_PIN[s], LOW);

    for (long i = 0; i < steps; i++) {
        digitalWrite(STEP_PIN[s], HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(STEP_PIN[s], LOW);
        delayMicroseconds(STEP_DELAY_US - STEP_PULSE_US);
    }
}

/**
 * Move string s to an absolute position (steps from home).
 */
void moveTo(uint8_t s, long targetSteps) {
    long delta = targetSteps - currentPos[s];
    stepMotor(s, delta);
    currentPos[s] = targetSteps;
}

/**
 * Home a single string by driving toward the limit switch, then backing off.
 */
void homeString(uint8_t s) {
    digitalWrite(EN_PIN[s], LOW);
    digitalWrite(DIR_PIN[s], LOW);   // toward home / nut

    // Drive until limit switch closes (or 50 000 steps safety limit)
    for (long i = 0; i < 50000L; i++) {
        if (digitalRead(LIMIT_PIN[s]) == LOW) break;  // switch triggered
        digitalWrite(STEP_PIN[s], HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(STEP_PIN[s], LOW);
        delayMicroseconds(HOME_DELAY_US - STEP_PULSE_US);
    }

    // Back off a little so we're not pressing the switch
    digitalWrite(DIR_PIN[s], HIGH);
    for (int16_t i = 0; i < HOME_BACKOFF; i++) {
        digitalWrite(STEP_PIN[s], HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(STEP_PIN[s], LOW);
        delayMicroseconds(HOME_DELAY_US - STEP_PULSE_US);
    }

    currentPos[s] = 0;
}

/**
 * Home all 4 strings (done in parallel by interleaving steps).
 * Each string drives toward its limit switch simultaneously.
 */
void homeAll() {
    bool done[N_STRINGS]    = { false, false, false, false };
    long safety[N_STRINGS]  = { 0, 0, 0, 0 };

    // Enable all drivers, set direction toward home
    for (uint8_t s = 0; s < N_STRINGS; s++) {
        digitalWrite(EN_PIN[s],  LOW);
        digitalWrite(DIR_PIN[s], LOW);
    }

    bool allDone = false;
    while (!allDone) {
        allDone = true;
        for (uint8_t s = 0; s < N_STRINGS; s++) {
            if (done[s]) continue;
            if (digitalRead(LIMIT_PIN[s]) == LOW || safety[s] > 50000L) {
                done[s] = true;
                continue;
            }
            digitalWrite(STEP_PIN[s], HIGH);
            delayMicroseconds(STEP_PULSE_US);
            digitalWrite(STEP_PIN[s], LOW);
            safety[s]++;
            allDone = false;
        }
        delayMicroseconds(HOME_DELAY_US - STEP_PULSE_US);
    }

    // Back-off pass
    for (uint8_t s = 0; s < N_STRINGS; s++) {
        digitalWrite(DIR_PIN[s], HIGH);
    }
    for (int16_t i = 0; i < HOME_BACKOFF; i++) {
        for (uint8_t s = 0; s < N_STRINGS; s++) {
            digitalWrite(STEP_PIN[s], HIGH);
            delayMicroseconds(STEP_PULSE_US);
            digitalWrite(STEP_PIN[s], LOW);
        }
        delayMicroseconds(HOME_DELAY_US - STEP_PULSE_US);
    }

    for (uint8_t s = 0; s < N_STRINGS; s++) currentPos[s] = 0;
}

// ── Command parser ────────────────────────────────────────────────────────────

void handleCommand(const String& cmd) {
    if (cmd.startsWith("MOVE ")) {
        // MOVE <string_index> <absolute_steps>
        int  spaceIdx = cmd.indexOf(' ', 5);
        uint8_t s     = (uint8_t) cmd.substring(5, spaceIdx).toInt();
        long    steps = cmd.substring(spaceIdx + 1).toInt();

        if (s < N_STRINGS) {
            moveTo(s, steps);
        }

    } else if (cmd.startsWith("PLUCK ")) {
        // PLUCK <string_index>
        uint8_t s = (uint8_t) cmd.substring(6).toInt();
        if (s < N_STRINGS) {
            digitalWrite(PLUCK_PIN[s], HIGH);
            delay(PLUCK_MS);
            digitalWrite(PLUCK_PIN[s], LOW);
        }

    } else if (cmd == "HOME") {
        homeAll();
        Serial.println("HOMED");

    } else if (cmd.startsWith("WAIT ")) {
        // WAIT <milliseconds>
        uint32_t ms = (uint32_t) cmd.substring(5).toInt();
        delay(ms);
    }
    // Unknown commands silently ignored
}

// ── Arduino setup & loop ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    for (uint8_t s = 0; s < N_STRINGS; s++) {
        pinMode(STEP_PIN[s],  OUTPUT);
        pinMode(DIR_PIN[s],   OUTPUT);
        pinMode(EN_PIN[s],    OUTPUT);
        pinMode(PLUCK_PIN[s], OUTPUT);
        pinMode(LIMIT_PIN[s], INPUT_PULLUP);

        // Disable drivers until homing is done
        digitalWrite(EN_PIN[s],    HIGH);
        digitalWrite(PLUCK_PIN[s], LOW);
    }

    homeAll();

    Serial.println("READY");   // Python waits for exactly this string
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            handleCommand(line);
        }
    }
}
