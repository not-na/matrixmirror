#include "main.h"

#include <hardware/watchdog.h>
#include <pico/cyw43_arch.h>
#include <server.h>

constexpr framebuf_src use_framebuf_src = FRAMEBUF_SRC_TCP;

uint32_t framebuf[DISPLAY_SIZE*DISPLAY_SIZE];


int main() {
    stdio_init_all();

    puts("mm-pico booting up...\n");

    // Initialize hub75
    hub75_init();

    multicore_launch_core1(hub75_main);
    sleep_ms(100);

    if (watchdog_caused_reboot()) {
        puts("Reboot caused by watchdog!");
    }

    if (use_framebuf_src == FRAMEBUF_SRC_TCP) {
        if (cyw43_arch_init_with_country(CYW43_COUNTRY_GERMANY)) {
            puts("Failed to initialize WiFi");
            watchdog_reboot(0,0,0);
            busy_wait_ms(1000);
        }

        cyw43_arch_enable_sta_mode();

        if (cyw43_arch_wifi_connect_timeout_ms(wifi_ssid, wifi_password, CYW43_AUTH_WPA3_WPA2_AES_PSK, 30000)) {
            puts("Failed to connect to network");
            watchdog_reboot(0,0,0);
            busy_wait_ms(1000);
        }

        puts("WiFi connected");

        server_init();
    }

    uint32_t frame = 0;
    absolute_time_t frame_time = get_absolute_time();

    bool last_loop_rendered = false;
    absolute_time_t last_warn = get_absolute_time();
    uint32_t warn_count = 0;

    watchdog_enable(1000, true);

    while (true) {
        watchdog_update();
        frame_time = get_absolute_time();

        // if (last_loop_rendered) {
        //     // Close to the limit, warn
        //     warn_count++;
        //     if (time_reached(delayed_by_ms(last_warn, 1000))) {
        //         printf("WARNING: CPU may be overloaded, warning counter: %lu\n", warn_count);
        //         last_warn = get_absolute_time();
        //     }
        // }

        if (use_framebuf_src == FRAMEBUF_SRC_SERIAL) {
            // Framebuffer received over serial, either USB or UART
            if (!stdio_usb_connected()) {
                // Attempting to do getchar() if USB is not connected seems to prevent the device
                // from enumerating
                continue;
            }

            printf("Waiting for frame... (USB=%d)\n", stdio_usb_connected());
            // Wait for preamble first: 00 C0 FE AA
            int s = 0;
            while (true) {
                uint8_t c = getchar();
                if (s == 0 && c == 0x00) {
                    s = 1;
                } else if (s == 1 && c == 0xC0) {
                    s = 2;
                } else if (s == 2 && c == 0xFE) {
                    s = 3;
                } else if (s == 3 && c == 0xAA) {
                    break;  // Done!
                } else {
                    s = 0;
                }
            }

            for (int i = 0; i < sizeof(framebuf)/sizeof(framebuf[0]); ++i) {
                // Fetch packed RGB and store into framebuf with padding
                framebuf[i] = getchar()  | getchar() << 8 | getchar() << 16;
            }

        } else if (use_framebuf_src == FRAMEBUF_SRC_SPI) {
            panic("SPI not yet implemented");
        } else if (use_framebuf_src == FRAMEBUF_SRC_TCP) {
            uint8_t* buf = server_get_cur_buf();
            if (buf == nullptr) {
                //printf("No buffer ready\n");
                sleep_ms(5);
                continue;
            }

            for (int i = 0; i < sizeof(framebuf)/sizeof(framebuf[0]); ++i) {
                // Unpack RGB into framebuf
                framebuf[i] = buf[i*3] | buf[i*3+1] << 8 | buf[i*3+2] << 16;
            }
        }

        // Wait until FIFO gives us back a token so we can refresh
        uint32_t fifo_out = 0;
        if (!multicore_fifo_pop_timeout_us(1000*100, &fifo_out)) {
            panic("HUB75 is unresponsive");
        }
        assert(fifo_out == DISPLAY_REDRAW_DONE_MAGIC_NUMBER);

        //printf("Got frame, blitting\n");
        hub75_blit_from_buffer(framebuf, 64, 64);

        // Done here, trigger redraw from second core
        multicore_fifo_push_blocking(DISPLAY_TRIGGER_REDRAW_MAGIC_NUMBER);

        frame++;
        last_loop_rendered = true;

        // Timing stats printed every second
        absolute_time_t et = get_absolute_time();
        if (frame % (display_fps/1) == 0) {
            int64_t ft = absolute_time_diff_us(frame_time, et);
            printf("Frametime=%lldus (max=%dus) cpu=%.3f%%\n", ft, 1000000/display_fps,  ((int32_t)ft)/(1000000.0/display_fps)*100);
        }
    }
}
