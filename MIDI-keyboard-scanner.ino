/*
MIDI Keyboard Scanner for Roland/Edirol PCR keybed
Copyright (C) 2023 Lorin Tackett <lorin@lorintackett.com>

This MIDI keyboard scanner works with Roland and Edirol keybeds from the PCR
series. The pin names refer directly to the pins on the PCR W30 keybed.

It is designed to run on a Teensy microcontroller, but it is generic enough
that it should work on any arduino-compatible microcontroller.

IMPORTANT: The Arduino MIDI library is included with the Teensy Arduino library,
you may need to install this library yourself.
https://github.com/FortySevenEffects/arduino_midi_library

Inspired by Daniel Moura's Keyboard Scanner:
https://github.com/oxesoft/keyboardscanner

This code is originally hosted at:
https://github.com/ltackett/MIDI-keyboar-scanner

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Uncomment as necessary
// #include <USB-MIDI>

// Uncomment to see MIDI data being sent out
// #define DEBUG_MIDI_MESSAGE

// Uncomment to see serial messages from the state machine
// #define DEBUG_STATE_MACHINE

#define PIN_T0 0
#define PIN_T1 1
#define PIN_T2 2
#define PIN_T3 3
#define PIN_T4 4
#define PIN_T5 5
#define PIN_T6 6
#define PIN_T7 7

#define PIN_PM2 19
#define PIN_PM3 20
#define PIN_PM4 21
#define PIN_PM5 22
#define PIN_PM6 23

#define PIN_SM2 12
#define PIN_SM3 11
#define PIN_SM4 10
#define PIN_SM5 9
#define PIN_SM6 8

#define LED_PIN 13

#define STATE_KEY_OFF 0
#define STATE_KEY_START 1
#define STATE_KEY_ON 2
#define STATE_KEY_RELEASED 3

#define MIN_TIME_MS 3
#define MAX_TIME_MS 50
#define MAX_TIME_MS_N (MAX_TIME_MS - MIN_TIME_MS)

#define MIDI_CHANNEL 1
#define MIDI_KEY_OFFSET 41
#define MIDI_NOTE_ON 0x90
#define MIDI_NOTE_OFF 0x80
#define TOTAL_KEYS 32

// Trigger pins (OUTPUT)
byte t_pins[]{
  PIN_T0,
  PIN_T1,
  PIN_T2,
  PIN_T3,
  PIN_T4,
  PIN_T5,
  PIN_T6,
  PIN_T7
};

// Primary measurement (INPUT_PULLUP)
byte pm_pins[]{
  PIN_PM2,
  PIN_PM3,
  PIN_PM4,
  PIN_PM5,
  PIN_PM6,
};

// Secondary measurement (INPUT_PULLUP)
byte sm_pins[]{
  PIN_SM2,
  PIN_SM3,
  PIN_SM4,
  PIN_SM5,
  PIN_SM6
};

// Note names from lowest to highest, grouped by octave, for debugging purposes
#ifdef DEBUG_STATE_MACHINE
  const char* note_names[TOTAL_KEYS] = { 
                                  "F2", "F#2", "G2", "G#2", "A2", "A#2", "B2",
  "C3", "C#3", "D3", "D#3", "E3", "F3", "F#3", "G3", "G#3", "A3", "A#3", "B3",
  "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4",
  "C5",
  };
#endif

// Map of pin combinations per key, grouped by octave
byte keybed[TOTAL_KEYS][3]{
  { PIN_T1, PIN_PM2, PIN_SM2 }, // F2
  { PIN_T2, PIN_PM2, PIN_SM2 }, // F#2
  { PIN_T3, PIN_PM2, PIN_SM2 }, // G2
  { PIN_T4, PIN_PM2, PIN_SM2 }, // G#2
  { PIN_T5, PIN_PM2, PIN_SM2 }, // A2
  { PIN_T6, PIN_PM2, PIN_SM2 }, // A#2
  { PIN_T7, PIN_PM2, PIN_SM2 }, // B2

  { PIN_T0, PIN_PM3, PIN_SM3 }, // C3
  { PIN_T1, PIN_PM3, PIN_SM3 }, // C#3
  { PIN_T2, PIN_PM3, PIN_SM3 }, // D3
  { PIN_T3, PIN_PM3, PIN_SM3 }, // D#3
  { PIN_T4, PIN_PM3, PIN_SM3 }, // E3
  { PIN_T5, PIN_PM3, PIN_SM3 }, // F3
  { PIN_T6, PIN_PM3, PIN_SM3 }, // F#3
  { PIN_T7, PIN_PM3, PIN_SM3 }, // G3
  { PIN_T0, PIN_PM4, PIN_SM4 }, // G#3
  { PIN_T1, PIN_PM4, PIN_SM4 }, // A3
  { PIN_T2, PIN_PM4, PIN_SM4 }, // A#3
  { PIN_T3, PIN_PM4, PIN_SM4 }, // B3

  { PIN_T4, PIN_PM4, PIN_SM4 }, // C4
  { PIN_T5, PIN_PM4, PIN_SM4 }, // C#4
  { PIN_T6, PIN_PM4, PIN_SM4 }, // D4
  { PIN_T7, PIN_PM4, PIN_SM4 }, // D#4
  { PIN_T0, PIN_PM5, PIN_SM5 }, // E4
  { PIN_T1, PIN_PM5, PIN_SM5 }, // F4
  { PIN_T2, PIN_PM5, PIN_SM5 }, // F#4
  { PIN_T3, PIN_PM5, PIN_SM5 }, // G4
  { PIN_T4, PIN_PM5, PIN_SM5 }, // G#4
  { PIN_T5, PIN_PM5, PIN_SM5 }, // A4
  { PIN_T6, PIN_PM5, PIN_SM5 }, // A#4
  { PIN_T7, PIN_PM5, PIN_SM5 }, // B4

  { PIN_T0, PIN_PM6, PIN_SM6 }  // C5
};

// Store for state per key
byte states[TOTAL_KEYS] = {};

// Store for PM-to-SM delta time (ktime) per key
long long ktimes[TOTAL_KEYS] = {};

// Boolean if any notes are on
bool is_note_on = false;

// Handle calculating velicty and sending MIDI data
void send_midi_note(byte status_byte, byte key_index, unsigned int time) {
  byte key = MIDI_KEY_OFFSET + key_index;

  // Velocity from time measurement
  unsigned int t = time;
  if (t > MAX_TIME_MS)
    t = MAX_TIME_MS;
  if (t < MIN_TIME_MS)
    t = MIN_TIME_MS;
  t -= MIN_TIME_MS;
  unsigned long velocity = 127 - (t * 127 / MAX_TIME_MS_N);
  byte vel = (((velocity * velocity) >> 7) * velocity) >> 7;

  #ifdef DEBUG_MIDI_MESSAGE
    char out[32];
    sprintf(out, "%02X %02X %03d %d", status_byte, key, vel, time);
    Serial.println(out);
  #endif


  if (status_byte == MIDI_NOTE_ON) {
    usbMIDI.sendNoteOn(key, vel == 0 ? 1 : vel, MIDI_CHANNEL);
  }

  else if (status_byte == MIDI_NOTE_OFF) {
    usbMIDI.sendNoteOff(key, vel, MIDI_CHANNEL);
  }
}

// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);

  // Initialize the note indicator LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Set pinMode for T pins
  for (byte pin = 0; pin < sizeof(t_pins); pin++) {
    pinMode(t_pins[pin], OUTPUT);
  }

  // Set pinMode for PM pins
  for (byte pin = 0; pin < sizeof(pm_pins); pin++) {
    pinMode(pm_pins[pin], INPUT_PULLUP);
  }

  // Set pinMode for SM pins
  for (byte pin = 0; pin < sizeof(sm_pins); pin++) {
    pinMode(sm_pins[pin], INPUT_PULLUP);
  }

  Serial.println("Started");
  Serial.println();
}

// LOOP
// ============================================================================

void loop() {
  // Detect if any keys are on
  for (byte key = 0; key < TOTAL_KEYS; key++) {
    if (states[key] == STATE_KEY_ON) {
      is_note_on = true;
    }
  }

  // Illuminate the LED if any keys are on
  digitalWrite(LED_PIN, is_note_on);

  // Scan through each T pin
  for (byte t_pin_index = 0; t_pin_index < sizeof(t_pins); t_pin_index++) {
    byte t_pin = t_pins[t_pin_index];

    // Enable T pin
    digitalWrite(t_pin, LOW);

    // Delay scanning by a tiny amount to prevent ghost readings
    delayMicroseconds(1);

    // Scan through each PM pin
    for (byte pm_pin_index = 0; pm_pin_index < sizeof(pm_pins); pm_pin_index++) {
      byte pm_pin = pm_pins[pm_pin_index];
      bool pm_switch_is_on = digitalRead(pm_pin) == LOW ? true : false;

      // Scanning each SM pin is unnecessary, as we can derive the related SM
      // pin from the PM pin index.
      byte sm_pin = sm_pins[pm_pin_index];
      bool sm_switch_is_on = digitalRead(sm_pin) == LOW ? true : false;

      // Loop through each key and update the state of the key based on the
      // PM/SM values being pulled low
      for (byte key = 0; key < TOTAL_KEYS; key++) {

        // Match the key in the loop with the current PM gate
        if (keybed[key][0] == t_pin && keybed[key][1] == pm_pin) {
          // Get references to this key's state/ktime so that when we write to
          // it, the new value is stored in the corresponding index of their
          // respective arrays
          byte& state = states[key];
          long long& ktime = ktimes[key];

          #ifdef DEBUG_STATE_MACHINE
            const char* note_name = note_names[key];
          #endif

          // State machine
          // TODO: Add support for sustain pedal
          switch (state) {
            case STATE_KEY_OFF:
              if (pm_switch_is_on) {
                state = STATE_KEY_START;
                ktime = millis(); // get current 

                #ifdef DEBUG_STATE_MACHINE
                  Serial.print(note_name);
                  Serial.println(" started");
                #endif
              } break;
            
            case STATE_KEY_START:
              if (sm_switch_is_on) {
                state = STATE_KEY_ON;
                send_midi_note(MIDI_NOTE_ON, key, millis() - ktime); // MIDI note_on

                #ifdef DEBUG_STATE_MACHINE
                  Serial.print(note_name);
                  Serial.print(" on, ktime: ");
                  Serial.println(millis() - ktime);
                #endif
                
              } else if (!pm_switch_is_on && !sm_switch_is_on) {
                state = STATE_KEY_OFF;

                #ifdef DEBUG_STATE_MACHINE
                  Serial.print(note_name);
                  Serial.println(" off condition met before measurement could be taken, reset to off");
                  Serial.println();
                #endif
              } break;
            
            case STATE_KEY_ON:
              if (!sm_switch_is_on) {
                state = STATE_KEY_RELEASED;

                #ifdef DEBUG_STATE_MACHINE
                  Serial.print(note_name);
                  Serial.println(" released");
                #endif

              } else if (!pm_switch_is_on && !sm_switch_is_on) {
                state = STATE_KEY_OFF;

                #ifdef DEBUG_STATE_MACHINE
                  Serial.print(note_name);
                  Serial.println(" off condition met before release was detected, reset to off");
                  Serial.println();
                #endif
              } break;

            case STATE_KEY_RELEASED:
              if (!pm_switch_is_on && !sm_switch_is_on) {
                state = STATE_KEY_OFF;
                send_midi_note(MIDI_NOTE_OFF, key, millis() - ktime); // MIDI note_off

                #ifdef DEBUG_STATE_MACHINE
                  Serial.print(note_name);
                  Serial.println(" off");
                  Serial.println();
                #endif
              } break;
            
            default: break;
          }
        }
      }
    }

    // Reset T pin for next measurement
    digitalWrite(t_pin, HIGH);
  }
}
