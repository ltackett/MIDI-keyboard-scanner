# MIDI Keyboard Scanner

MIDI Keyboard Scanner for Roland/Edirol PCR keybed  
Copyright (C) 2023 Lorin Tackett <lorin@lorintackett.com>

This MIDI keyboard scanner works with Roland and Edirol keybeds from the PCR series, and may be expanded to work with larger keybeds. The pin names refer directly to the pins on the PCR W30 keybed. Plug each pin from the ribbon cable directly into the pins you've configured on your microcontroller.

This software was designed on a [Teensy microcontroller](https://www.pjrc.com/store/teensy40.html), but is generic enough that it should work on any arduino-compatible microcontroller.

**IMPORTANT:** The Arduino MIDI library is [included with the Arduino library for Teensy](https://www.pjrc.com/teensy/td_midi.html), you may need to install this library yourself:  
https://github.com/FortySevenEffects/arduino_midi_library

Inspired by Daniel Moura's Keyboard Scanner:  
https://github.com/oxesoft/keyboardscanner

---

This code is originally hosted at:  
https://github.com/ltackett/MIDI-keyboard-scanner

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.
