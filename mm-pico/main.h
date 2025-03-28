#pragma once

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/bootrom.h"


#include <cstdio>

#include "config.h"
#include "hub75.h"

constexpr uint8_t magic_preamble[4] = {
    0x00,
    0xC0,
    0xFE,
    0xAA,
};

constexpr int tcp_port = 1234;

enum framebuf_src {
    FRAMEBUF_SRC_SERIAL,
    FRAMEBUF_SRC_SPI,
    FRAMEBUF_SRC_TCP,
};

extern char wifi_ssid[];
extern char wifi_password[];
