#include "main.h"

#include <hardware/spi.h>
#include <hardware/watchdog.h>
#include <pico/cyw43_arch.h>
#include <server.h>

#include "hardware/clocks.h"

constexpr framebuf_src use_framebuf_src = FRAMEBUF_SRC_SPI;

uint32_t framebuf[DISPLAY_WIDTH*DISPLAY_HEIGHT];

constexpr int pin_spi_clk = 2;
constexpr int pin_spi_miso = 3;
constexpr int pin_spi_mosi = 4;
constexpr int pin_spi_cs = 5;

auto host_spi = spi0;

uint8_t buf_a[bufsize+sizeof(magic_preamble)];
uint8_t buf_b[bufsize+sizeof(magic_preamble)];

uint8_t* buf_dma = buf_a;
uint8_t* buf_proc = buf_b;

int main() {
    uint dma_rx;

    stdio_init_all();
    //sleep_ms(2000);

    puts("mm-pico booting up...\n");

    // Initialize hub75
    hub75_init();

    multicore_launch_core1(hub75_main);

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
    } else if (use_framebuf_src == FRAMEBUF_SRC_SPI) {
        // Setup GPIO and SPI
        spi_init(host_spi, 1*1000*1000);
        gpio_set_function(pin_spi_clk, GPIO_FUNC_SPI);
        gpio_set_function(pin_spi_mosi, GPIO_FUNC_SPI);
        gpio_set_function(pin_spi_miso, GPIO_FUNC_SPI);
        gpio_set_function(pin_spi_cs, GPIO_FUNC_SPI);

        spi_set_format(host_spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
        spi_set_slave(host_spi, true);

        // Init RX DMA
        dma_rx = dma_claim_unused_channel(true);

        dma_channel_config c = dma_channel_get_default_config(dma_rx);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
        channel_config_set_dreq(&c, spi_get_dreq(host_spi, false));
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        dma_channel_configure(dma_rx, &c,
            buf_dma,
            &spi_get_hw(host_spi)->dr,
            bufsize+sizeof(magic_preamble),
            true
        );
    }

    uint32_t frame = 0;
    absolute_time_t frame_time = get_absolute_time();
    absolute_time_t stat_time = get_absolute_time();

    bool last_loop_rendered = false;
    absolute_time_t last_warn = get_absolute_time();
    uint32_t warn_count = 0;

    uint desync_count = 0;
    uint sync_count = 0;

    watchdog_enable(1000, true);

    //printf("Running with a system clock of %dkHz\n", frequency_count_khz(clk_sys));

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
            // Wait for DMA to be ready
            //puts("Waiting for frame...");
            dma_channel_wait_for_finish_blocking(dma_rx);
            //printf("DMA finished!\n");

            // Check if preamble is correct
            if (memcmp(buf_dma, magic_preamble, sizeof(magic_preamble)) != 0) {
                // Preamble does not match!
                // We have likely been desynced (or booted in the middle of a frame)
                puts("Desynced, resyncing");
                desync_count++;
                if (desync_count == 5) {
                    // Something is wrong, we have too many desyncs recently
                    // Reboot to try and fix the issue
                    printf("Too many desyncs, rebooting\n");
                    gpio_init(DISPLAY_OENPIN);
                    gpio_set_dir(DISPLAY_OENPIN, GPIO_OUT);
                    gpio_put(DISPLAY_OENPIN, true);
                    watchdog_reboot(0, 0, 0);
                }

                // Poll SPI data until we get a preamble
                int s = 0;
                while (true) {
                    uint8_t c;
                    spi_read_blocking(host_spi, 0, &c, 1);
                    //printf("c=%d\n", c);

                    if (c == magic_preamble[s]) {
                        s++;
                        if (s >= sizeof(magic_preamble)) {
                            break;  // Synchronized again
                        }
                    } else {
                        s = 0;
                    }
                }

                dma_channel_set_write_addr(dma_rx, &buf_dma[sizeof(magic_preamble)], true);
                memcpy(buf_dma, magic_preamble, sizeof(magic_preamble));  // Manually copy preamble since we already consumed it
                puts("Synced again");
                continue;
            }

            sync_count++;
            if (sync_count > 10) {
                desync_count = 0;
            }

            // Swap buffers and restart DMA immediately
            uint8_t* tmp = buf_dma;
            buf_dma = buf_proc;
            buf_proc = tmp;

            dma_channel_set_write_addr(dma_rx, buf_dma, true);
            //puts("Buffers swapped");

            // Unpack RGB into framebuf
            for (int i = 0; i < sizeof(framebuf)/sizeof(framebuf[0]); ++i) {
                framebuf[i] = buf_proc[i*3+sizeof(magic_preamble)] | buf_proc[i*3+sizeof(magic_preamble)+1] << 8 | buf_proc[i*3+sizeof(magic_preamble)+2] << 16;
            }

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
        hub75_blit_from_buffer(framebuf, DISPLAY_WIDTH, DISPLAY_HEIGHT);

        // Done here, trigger redraw from second core
        multicore_fifo_push_blocking(DISPLAY_TRIGGER_REDRAW_MAGIC_NUMBER);

        frame++;
        last_loop_rendered = true;

        // Timing stats printed every second
        absolute_time_t et = get_absolute_time();
        if (frame % (display_fps/1) == 0) {
            int64_t ft = absolute_time_diff_us(frame_time, et);
            float fps = 1000000.0f / absolute_time_diff_us(stat_time, et) * display_fps;
            printf("Frametime=%lldus (max=%dus) cpu=%.3f%% FPS: %f (%fHz)\n", ft, 1000000/display_fps,  ((int32_t)ft)/(1000000.0/display_fps)*100, fps, hub75_hz);
            stat_time = et;
        }
    }
}
