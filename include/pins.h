#ifndef PINS_H
#define PINS_H

// ============================================================================
// MKS BASE V1.6 / RAMPS 1.4 Pin Definitions for ATmega2560
// ============================================================================

// --- X Axis Stepper ---
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

// --- Y Axis Stepper ---
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

// --- Z Axis Stepper ---
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62

// --- Extruder E0 (Optional/Reserve) ---
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

// --- Extruder E1 (Optional/Reserve) ---
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30

// --- Endstops / Home Sensors ---
// Standard RAMPS layout maps - (Min) and + (Max)
#define X_MIN_PIN           3
#define X_MAX_PIN           2   // "X+" on the board

#define Y_MIN_PIN          14
#define Y_MAX_PIN          15   // "Y+" on the board

#define Z_MIN_PIN          18
#define Z_MAX_PIN          19   // "Z+" on the board

// Alias definitions for home sensors
#define X_HOME_PIN         X_MAX_PIN
#define Y_HOME_PIN         Y_MAX_PIN
#define Z_HOME_PIN         Z_MAX_PIN

// Using the correct Servo headers (D11, D12, A11, A12) available on board
#define BTN_TARGET_3_PIN   11   // D11
#define BTN_TARGET_4_PIN   12   // D12
#define BTN_TARGET_1_PIN   65   // A11 (Digital Pin 65 on ATmega2560)
#define BTN_TARGET_2_PIN   66   // A12 (Digital Pin 66 on ATmega2560)

// --- Other Helpers ---
#define LED_PIN            13   // Default Mega board LED

#endif // PINS_H
