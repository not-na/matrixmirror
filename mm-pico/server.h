#pragma once

#include "pico/stdlib.h"

#include <hub75.h>

constexpr int bufsize = DISPLAY_WIDTH*DISPLAY_HEIGHT*3;

void server_init();

uint8_t* server_get_cur_buf();