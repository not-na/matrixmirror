#pragma once

#include <stdio.h>
#include "math.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

#include "hub75.pio.h"

// Size of the display, width is x axis and height is y axis
#define DISPLAY_WIDTH 128  // Width is the length of the shift register chain (usually the longer axis)
#define DISPLAY_HEIGHT 64  // Height is twice the display scan factor
static_assert(DISPLAY_WIDTH >= DISPLAY_HEIGHT);

// Scan factor of the display
// Code will currently not work correctly if this is not exactly half of DISPLAY_SIZE
#define DISPLAY_SCAN 32
static_assert(DISPLAY_SCAN*2 == DISPLAY_HEIGHT);

// Integer between 1 and 8
// Lower numbers cause LSBs to be skipped
#define DISPLAY_BITDEPTH 8

// R0, G0, B0, R1, G1, B1 pins, consecutive
#define DISPLAY_DATAPINS_BASE 6
#define DISPLAY_DATAPINS_COUNT 6

// A, B, C, D, E pins for row selection, consecutive
#define DISPLAY_ROWSEL_BASE 12
#define DISPLAY_ROWSEL_COUNT 5
static_assert(pow(2, DISPLAY_ROWSEL_COUNT) == DISPLAY_SCAN);

// CLK pin
#define DISPLAY_CLKPIN 17
// LAT, OE pins. Must be consecutive
#define DISPLAY_STROBEPIN 18
#define DISPLAY_OENPIN DISPLAY_STROBEPIN+1

// Amount of pixels per framebuffer
#define DISPLAY_FRAMEBUFFER_SIZE (DISPLAY_WIDTH*DISPLAY_HEIGHT)

// Amount of time to blank the display after each full frame draw
// 0 for full brightness, increase to dim
// ~3000 for half-brightness
// Note that very high values will cause FPS drops and may cause visible flicker
#define DISPLAY_WAIT_US 3000

// An arbitrary 32-bit number to use for triggering redraws / simulations
// We could just use the same number or even 0, but this can catch bugs if
// other code also sends something over the FIFO
#define DISPLAY_TRIGGER_REDRAW_MAGIC_NUMBER 0xABCDEF01
#define DISPLAY_REDRAW_DONE_MAGIC_NUMBER 0x00ABCDEF

enum DISPLAY_REDRAWSTATE {
    DISPLAY_REDRAWSTATE_IDLE,
    DISPLAY_REDRAWSTATE_CLEAR,
};

extern float hub75_hz;

// TODO: write docs for hub75_* functions
void hub75_init();

[[noreturn]] void hub75_main();

bool hub75_pio_sm_stalled();
void hub75_pio_sm_clearstall();

DISPLAY_REDRAWSTATE hub75_update(DISPLAY_REDRAWSTATE state);

void hub75_blit_from_buffer(const uint32_t* buf, uint32_t w, uint32_t h);

static inline void hub75_draw_pixel(uint32_t* buf, uint32_t x, uint32_t y, uint32_t color);