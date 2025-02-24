#include <Arduino.h>
#include <FastLED.h>
#include <USB-MIDI.h>
#include <FlashStorage_SAMD.h>

// BSD 2-Clause License
//
// Copyright (c) 2025, Matt Keveney
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


// Firmware for MIDI foot pedal.
//
// This firwmare was developed in conjunction with a 3D printable
// project.  See the following link for details:
//
// http://keveney.com/posts/midi-foot-controller/
//
// This firmware uses the Arduino framework, the MidiUSB library, and the FastLED library.
// By using this code you agree to abide by the license agreements for each of
// these components as well as the above.
//
// This project was developed with the Seeed Studio XIAO SAMD21 board.  It may work on other
// hardware, as long as the above libraries are supported.
//
// The remaining hardware consists of a chain of WS2812 addressable LEDs wired to a single
// digitial pin; and a series of momentary contact switches, each wired between ground
// and a digital input pin.  See the configuration notes below for more.


// Configuration notes:
//
// Button config structures live in an array.
// Corresponding LEDs are expected to be at same index on the WS2812 chain
// There are three types of button may behave in a few different ways:
//
// momentary CC (Control change)
//    config CC message, LED color
//    on press, send midi CC 127, LED on
//    on release, send midi CC 0, LED off
//
// toggle CC (Control change)
//    config CC message, LED color
//    on press toggle between on/off states
//        if off, send midi CC message value 127 and turn LED on
//        if on, send midi CC message value 0 and turn LED off
//
// switch_PC (Program change)
//    config bank number, n_progs, first_prog programs
//    on press,
//      cycles to next program (from 0 to n_progs - 1)
//      sends midi PC message (offsetting by first_prog)
//      cycles to next LED color
//
//    If multiple switch_PC buttons are configured, only one may be active at a time.
//    Inactive ones have their LED turned off.
//    When reactivating, current_prog is reselected without cycling.
//
// Global config:
//    digital pin for LED chain
//    channel for all messages
//    Saturation for LEDs
//    V for LEDs
//
//
// For further details on MIDI see:
//  See https://midi.org/midi-1-0-detailed-specification
//

// data types & constants

#define SYSEX_MSG   0xF0
#define SYSEX_MFG_ID 0x7D
#define SYSEX_EOX   0xF7

#define DEBOUNCE_DELAY_MS 50
#define MSG_CTRLCHANGE  0x0B        // control change
#define MSG_PROGCHANGE  0x0C        // program change

// Bank select is a special control change message.
// it supports 14 bit values via MSB and LSB variants.
//
#define BANK_SEL_MSB (MIDI_NAMESPACE::BankSelect)
#define BANK_SEL_LSB (MIDI_NAMESPACE::BankSelect + 32)


#define NVRAM_SIGNATURE 0xDEEC0DED
#define NVRAM_SIGNATURE_OFFSET 0
#define NVRAM_SIGNATURE_SIZE (sizeof(uint32_t))
#define NVRAM_CFG_OFFSET (NVRAM_SIGNATURE_OFFSET + NVRAM_SIGNATURE_SIZE)

#define NUM_BUTTONS 3

typedef enum {
    MOMENTARY_CC = 0,
    TOGGLE_CC = 1,
    SWITCH_PC = 2
} button_type;

typedef struct {
    button_type btype;
    union {
        struct {        // used by MOMENTARY_CC and TOGGLE_CC
            uint8_t     code;
            uint8_t     hue;
        } cc;
        struct {        // used by SWITCH_PC
            uint8_t     n_progs;
            uint16_t    bank;
            uint8_t     first_prog;
        } pc;
    };
} button_cfg;

// overall config.  Saved in NVRAM.
typedef struct {
    uint8_t midi_channel;
    uint8_t vee;
    button_cfg buttons[NUM_BUTTONS];
} config;

typedef struct {
    uint8_t         press_state : 1;
    uint8_t         last_press_state : 1;
    unsigned long   last_bounce;
    union {
        bool        cc_tog_on;      // used by TOGGLE_CC
        struct {                    // used by SWITCH_PC
            uint8_t current_prog;
            bool    active;
        } spc;
    };
} button_state;


/// Pre-defined values for our config hues, which range from 0-127
typedef enum {
    FP_HUE_RED = 0,       ///< Red (0°)
    FP_HUE_ORANGE = 16,   ///< Orange (45°)
    FP_HUE_YELLOW = 32,   ///< Yellow (90°)
    FP_HUE_GREEN = 48,    ///< Green (135°)
    FP_HUE_AQUA = 64,    ///< Aqua (180°)
    FP_HUE_BLUE = 80,    ///< Blue (225°)
    FP_HUE_PURPLE = 96,  ///< Purple (270°)
    FP_HUE_PINK = 112    ///< Pink (315°)
} FPHSVHue;

#define LED_OFF (CRGB(0, 0, 0))

// This creates a transport object and then a MidiInterface object from it
// transport is called __usbMyMidi, inteface: MyMidi
// first param is 'cable number' (?) that the transport object uses

USBMIDI_CREATE_INSTANCE(0, MyMidi)

#define SAT 255     // Saturated colors look best

#define LED_PIN D6
const uint8_t button_pins[NUM_BUTTONS] = {D5, D4, D3};
config cfg = {
    midi_channel: 2,
    vee: 64,
    buttons: {
      {btype: SWITCH_PC, {pc:{n_progs: 6, bank: 2, first_prog: 0}}},
      {btype: TOGGLE_CC,  {cc:{code: 80, hue: FP_HUE_AQUA}}},
      {btype: MOMENTARY_CC, {cc:{code: 81, hue: FP_HUE_ORANGE}}}
    }
};

// ==========================================================

// current state
//
button_state button_states[NUM_BUTTONS];
CRGB leds[NUM_BUTTONS];

void bankSelectMessage(uint16_t bank) {
    MyMidi.sendControlChange(BANK_SEL_MSB, bank >> 7, cfg.midi_channel);
    MyMidi.sendControlChange(BANK_SEL_LSB, bank & 0x7F, cfg.midi_channel);
}

// Read config from NVRam
//
void cfg_from_NVRAM() {
    uint32_t signature;
    EEPROM.get(NVRAM_SIGNATURE_OFFSET, signature);

    // if signature mismatch, this is our first run after initialization
    // so we _write_ the default config to nvram
    if (signature != NVRAM_SIGNATURE) {
        signature = NVRAM_SIGNATURE;
        EEPROM.put(NVRAM_SIGNATURE_OFFSET, signature);
        EEPROM.put(NVRAM_CFG_OFFSET, cfg);
    } else {
        EEPROM.get(NVRAM_CFG_OFFSET, cfg);
    }
}

// convert 7 bit hue CHSV for use by fastLED
//
CHSV toColor(uint8_t hue) {
    return CHSV(hue << 1, SAT, cfg.vee << 1);
}

// Control change momentary action
//
void momentary(uint8_t bix) {
    button_cfg *cf = &cfg.buttons[bix];

    // note: buttons are pulled high when open and grounded when closed.
    // thus pressed state is 0; unpressed is 1
    //
    if (!button_states[bix].press_state) {
        hsv2rgb_rainbow(toColor(cf->cc.hue), leds[bix]);
        MyMidi.sendControlChange(cf->cc.code, 127, cfg.midi_channel);
    } else {
        leds[bix] = LED_OFF;
        MyMidi.sendControlChange(cf->cc.code, 0, cfg.midi_channel);
    }
}

// control change toggle action
//
void toggle_cc(uint8_t bix) {
    button_cfg *cf = &cfg.buttons[bix];
    button_state *bs = &button_states[bix];

    if (bs->press_state) {  // change state on press; ignore release;
        return;
    }

    bs->cc_tog_on = !bs->cc_tog_on;

    if (bs->cc_tog_on) {
        hsv2rgb_rainbow(toColor(cf->cc.hue), leds[bix]);
        MyMidi.sendControlChange(cf->cc.code, 127, cfg.midi_channel);
    } else {
        leds[bix] = LED_OFF;
        MyMidi.sendControlChange(cf->cc.code, 0, cfg.midi_channel);
    }
}

// program-change action
//
void switch_pc(uint8_t bix) {
    button_cfg *cf = &cfg.buttons[bix];
    button_state *bs = &button_states[bix];
    int hue;

    if (bs->press_state) {          // change program on press; ignore release;
        return;
    }

    if (bs->spc.active) {
        // if we're the active switch, just advance the program
        bs->spc.current_prog = (bs->spc.current_prog + 1) % cf->pc.n_progs;
    } else {
        // otherwise, deactivate any other SWITCH_PC buttons (turn LED off)
        for (int i = 0; i < NUM_BUTTONS; i++) {
            if (cfg.buttons[i].btype == SWITCH_PC) {
                button_states[i].spc.active = false;
                leds[i] = LED_OFF;
            }
        }
        // ...and turn this one back on
        bs->spc.active = true;
    }

    // we divide the hue space by n_progs;
    // (This means that pedals with different n_progs will have different color sequences; confusing?)

    hue = (bs->spc.current_prog * 127) / cf->pc.n_progs;
    hsv2rgb_rainbow(toColor(hue), leds[bix]);

    // reselect bank in case we've just reactivated this button.
    bankSelectMessage(cf->pc.bank);
    MyMidi.sendProgramChange(bs->spc.current_prog + cf->pc.first_prog, cfg.midi_channel);
}

// perform action associated with button
//
void button_action(uint8_t bix) {
    switch (cfg.buttons[bix].btype) {
        case MOMENTARY_CC:
            return momentary(bix);
        case TOGGLE_CC:
            return toggle_cc(bix);
        case SWITCH_PC:
            return switch_pc(bix);
    }
}

// Initialize software state, hardware, and midi for specified button
//
void button_init(uint8_t bix) {

    button_state *bs = &button_states[bix];
    button_cfg *cf = &cfg.buttons[bix];
    static bool first_pc;
    if (bix == 0) {
        first_pc = true;
    }

    bs->press_state = 1;
    bs->last_press_state = 1;
    bs->last_bounce = 0;

    pinMode(button_pins[bix], INPUT_PULLUP);

    switch (cfg.buttons[bix].btype) {

        case MOMENTARY_CC:
            leds[bix] = LED_OFF;
            MyMidi.sendControlChange(cf->cc.code, 0, cfg.midi_channel);
            break;

        case TOGGLE_CC:
            // toggles are ON by default
            hsv2rgb_rainbow(toColor(cf->cc.hue), leds[bix]);
            bs->cc_tog_on = true;
            MyMidi.sendControlChange(cf->cc.code, 127, cfg.midi_channel);
            break;

        case SWITCH_PC:
            button_states[bix].spc.current_prog = 0;
            // first switch_pc should be active; any others in the list initialized to inactive
            button_states[bix].spc.active = first_pc;
            if (first_pc) {
                hsv2rgb_rainbow(toColor(0), leds[bix]);
                bankSelectMessage(cf->pc.bank);
                MyMidi.sendProgramChange(cf->pc.first_prog, cfg.midi_channel);
            } else {
                leds[bix] = LED_OFF;
            }
            first_pc = false;
            break;
    }
}


// Read single button, debounce, and perform action when button state changes
//
void readButton(uint8_t ix) {

    int p_state  = digitalRead(button_pins[ix]);
    button_state *bs = &button_states[ix];

    if (p_state != bs->last_press_state) {
        bs->last_bounce = millis();
    }

    if (millis() - bs->last_bounce > DEBOUNCE_DELAY_MS) {

        if (p_state != bs->press_state) {
            bs->press_state = p_state;

            // This action must return quickly or we might miss a button press;
            // no blocking operations please.
            //
            button_action(ix);
        }
    }

    bs->last_press_state = p_state;
}


// these functions convert config values (max 127) to/from full byte range values (max 255)
//    as used in FastLED's H S or V parameters.
//    (We sacrifice a bit of resolution to simplify transferring the config to/from WebMIDI)
//
uint8_t scaleTo255(uint8_t n) {
    // yes, this is equivalent to proper scaling with roundoff
    return (n < 64) ? n << 1 : (n << 1) + 1;
}
uint8_t scaleTo127(uint8_t n) {
    return n >> 1;
}


// sends message back to config tool
//
void handleGetCfg(byte* array, unsigned size) {

    // we send back 'setcfg'

    byte msg[40] = {SYSEX_MSG, SYSEX_MFG_ID, 0x22, 0x01, 0x35, 0x37, 0x39, 0x32};
    int ix = 8;

    // global config items
    msg[ix++] = cfg.vee;
    msg[ix++] = cfg.midi_channel;
    msg[ix++] = NUM_BUTTONS;

    // button configs
    for (int i = 0; i < NUM_BUTTONS; i++) {
        msg[ix++] = cfg.buttons[i].btype;
        switch (cfg.buttons[i].btype) {
            case MOMENTARY_CC:
            case TOGGLE_CC:
                msg[ix++] = cfg.buttons[i].cc.code;
                msg[ix++] = cfg.buttons[i].cc.hue;
                break;
            case SWITCH_PC:
                msg[ix++] = cfg.buttons[i].pc.n_progs;
                msg[ix++] = (cfg.buttons[i].pc.bank >> 7);
                msg[ix++] = (cfg.buttons[i].pc.bank & 0x7F);
                msg[ix++] = cfg.buttons[i].pc.first_prog;
        }
    }
    msg[ix++] = SYSEX_EOX;

    MyMidi.sendSysEx((&msg[ix] - msg), msg, true);
}

// setCfg has extra data:
//  note: all elements are 8 bit, with top bit cleared; values 0-127 only.
//
//  <vee>           // 'v' component of hsv for all led colors; we scale to 0-255 later
//  <channel>       // channel to transmit CC and PC messages.
//  <n_buttons>     // # of buttons in config.
//
//      note: config tool must leave this unchanged; assumption is that the number of buttons is
//      a 'hardware' config, and not alterable after building.
//
//      otoh: making this alterable (along with pin assignments), would obviate
//      the need to compile the firmware _at all_, so might be something to think about.
//
//  One or more button_config elements, each beginning with a type, followed
//  by a type-specific body.  ...like the button_cfg structure:
//
//  <MOMENTARY_CC>
//  <code>
//  <hue>           // hue is 0-127; we scale to 0-255 later
//
//  <TOGGLE_CC>
//  <code>
//  <hue>
//
//  <SWITCH_PC>
//  <n_progs>
//  <bank MSB>
//  <bank LSB>
//  <first_prog>

void handleSetCfg(byte* msg, unsigned size) {
    if (msg[10] != NUM_BUTTONS) {
        return;
    }

    int ix = 8;

    cfg.vee = msg[ix++];
    cfg.midi_channel = msg[ix++];

    ix++;
    uint16_t b;

    for (int i = 0; i < NUM_BUTTONS; i++) {
        cfg.buttons[i].btype = (button_type)msg[ix++];

        if (cfg.buttons[i].btype == SWITCH_PC) {
            cfg.buttons[i].pc.n_progs = msg[ix++];

            b = msg[ix++] << 7;
            b |= msg[ix++];
            cfg.buttons[i].pc.bank = b;
            cfg.buttons[i].pc.first_prog = msg[ix++];
        } else {
            cfg.buttons[i].cc.code = msg[ix++];
            cfg.buttons[i].cc.hue = msg[ix++];
        }
        button_init(i);
    }
    EEPROM.put(NVRAM_CFG_OFFSET, cfg);

}


// our sysex messages are in the form:
//
//      0xf0 <mfid> 0x11 <device ID> <signature> 0xF7
//      0xf0 <mfid> 0x22 <device ID> <signature> <data> 0xF7
//      0xf0 <mfid> 0x33 <device ID> <signature> <error> 0xF7
//
// our signature is the 4 byte sequenc: 0x35 0x37 0x39 0x32
// we ignore messages that don't match our signature.
// For now, we disregard device ID, but that may come
// into play later.

//      cmd 11  - requestCfg - replies with setCfg message back (to device ID 127, which is the config tool)
//      cmd 22  - setCfg - receives new config and either:
//              re-initializes if recipient is the pedal device - replies with ackCfg
//              updates config U/I if recipient is the config tool
//
//      cmd 33  - ackCfg <error> - reply from pedal to setCfg msg
//              error = 0     okay
//              error = nz    error code
//              (still need to devise error codes)
//
//      note: config tool need not reply to setCfg.
//
void handleSystemExclusive(byte* array, unsigned size) {
    if (size < 8) {
        //Serial.println("Sysex msg too short; ignoring");
        return;
    }

    if (array[0] != SYSEX_MSG ||        // need this?
        array[1] != SYSEX_MFG_ID ||
                                // ignore cmd here; check below
                                // ignore device ID for now
        array[4] != 0x35 ||             // check signature
        array[5] != 0x37 ||
        array[6] != 0x39 ||
        array[7] != 0x32 ) {
        //Serial.println("sysex mismatch; ignoring");
        return;
    }

    // dispatch messages
    switch(array[2]) {
        case 0x11:
            return handleGetCfg(array, size);
        case 0x22:
            return handleSetCfg(array, size);
        //default:
            //Serial.printf("unrecognized message: %02x\n", array[2]);
    }
}


void setup() {
    //Serial.begin(115200);

    // read config from NVRAM
    cfg_from_NVRAM();

    MyMidi.setHandleSystemExclusive(handleSystemExclusive);

    // start listening.
    // Note we listen on all channels, but _transmit_ only on our configured channel.
    //
    // this also enables "soft thru" ? which evidently echoes messages through?
    //      ...need to better understand this.
    MyMidi.begin(MIDI_CHANNEL_OMNI);

    // The "Typical8mmPixel" color correction works best for the off-brand 5mm
    // T-package standalone LEDs I tested. YMMV.
    //
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_BUTTONS).setCorrection(Typical8mmPixel);
    for (int i = 0; i < NUM_BUTTONS; i++) {
        button_init(i);
    }
    FastLED.show();
}

void loop() {
    for (int i = 0; i < NUM_BUTTONS; i++) {
        readButton(i);
    }
    // These seem to be fast enough to not interfere with our debounce logic.
    // If this becomes an issue, refactor to handle debounce via hardware interrupt.
    //
    FastLED.show();

    // read any pending Midi messages
    // returns true if anything was read
    // fires any registered callback if corresponding message was received
    MyMidi.read();
}
