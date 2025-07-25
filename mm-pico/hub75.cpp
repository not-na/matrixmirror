#include "hub75.h"

/**
 * PIO programs and rough architecture are based on hub75 example from pico-examples
 * repo. To improve performance, DMA is used and redrawing is done while waiting
 * for DMA or bit timings.
 *
 * Architecture / Flow of the display driver:
 *
 * Nested loops:
 * outer is per frame and infinite
 *      inner per row and DISPLAY_SCAN iterations
 *          innermost per bit for color
 *
 *
 * Per row loop:
 * Per bit loop:
 *  Setup and trigger DMA to send pixel data for row to hub75_data PIO SM
 *
 *  Check if simulation step is done by checking SIO FIFO for a magic number
 *  If so, set flag display_redraw
 *
 *  Wait until DMA / PIO shifting is done
 *  Push out alignment pixels
 *
 *  Pulse LAT and OE using hub75_row PIO SM
 *
 *  Until hub75_row PIO SM is done, call display update routine
 *
 * Next bit, until full depth is done
 * Next row, until frame is done
 *
 * If frame is done and display_flip is set:
 *      Swap addresses of front and back buffers
 *      Write magic number to SIO FIFO to signal next simulation step
 *
 * Next frame, forever
 *
 * Display update routine:
 *      Restartable, e.g. state is saved between runs
 *      Clear backbuf by copying over background image
 *      Draw particles
 *      Set display_flip flag
 *
 * Framebuffers are interleaved to maximize performance
 * Given this Display:
 *   x->
 * y 1234
 * | 4567
 * v 89AB
 *   CDEF
 *
 * The framebuffer would look something like this:
 *  18293A4B
 *  4C5D6E7F
 *
 * The DMA would first push out the 182... row and then the 4C5... row
 * Even numbered Framebuffer indices are rows 0-15 and odd numbered indices are rows 16-31
 *
 * y-Coordinates are rows, x are columns
 *
 * Panel Colors:
 *
 * BGR -> 0x00BBGGRR
 */

static inline uint32_t gamma_correct_565_888(uint16_t pix) {
    uint32_t r_gamma = pix & 0xf800u;
    r_gamma *= r_gamma;
    uint32_t g_gamma = pix & 0x07e0u;
    g_gamma *= g_gamma;
    uint32_t b_gamma = pix & 0x001fu;
    b_gamma *= b_gamma;
    return (b_gamma >> 2 << 16) | (g_gamma >> 14 << 8) | (r_gamma >> 24 << 0);
}

// Double-buffering
uint32_t display_buffers[2][DISPLAY_FRAMEBUFFER_SIZE];

uint32_t* display_front_buf = &display_buffers[0][0];
uint32_t* display_back_buf = &display_buffers[1][0];

PIO display_pio = pio0;
uint display_sm_data, display_sm_row;
uint display_offset_data, display_offset_row;

int display_dma_chan;

bool display_flip = false;

uint8_t display_framenum = 0;

float hub75_hz = NAN;

void hub75_init() {
    // Information for picotool
    bi_decl(bi_3pins_with_names(
            DISPLAY_DATAPINS_BASE, "HUB75 R0 Pin",
            DISPLAY_DATAPINS_BASE+1, "HUB75 G0 Pin",
            DISPLAY_DATAPINS_BASE+2, "HUB75 B0 Pin"));
    bi_decl(bi_3pins_with_names(
            DISPLAY_DATAPINS_BASE+3, "HUB75 R1 Pin",
            DISPLAY_DATAPINS_BASE+4, "HUB75 G1 Pin",
            DISPLAY_DATAPINS_BASE+5, "HUB75 B1 Pin"));
    bi_decl(bi_4pins_with_names(
            DISPLAY_ROWSEL_BASE, "HUB75 A Pin",
            DISPLAY_ROWSEL_BASE+1, "HUB75 B Pin",
            DISPLAY_ROWSEL_BASE+2, "HUB75 C Pin",
            DISPLAY_ROWSEL_BASE+3, "HUB75 D Pin"
            ));
    bi_decl(bi_3pins_with_names(
            DISPLAY_CLKPIN, "HUB75 CLK Pin",
            DISPLAY_STROBEPIN, "HUB75 LAT / STB Pin",
            DISPLAY_OENPIN, "HUB75 OEn Pin"
            ));

    // Immediately disable display to reduce flashes during reset
    gpio_init(DISPLAY_OENPIN);
    gpio_set_dir(DISPLAY_OENPIN, GPIO_OUT);
    gpio_put(DISPLAY_OENPIN, true);

    // Initialize PIO
    display_sm_data = pio_claim_unused_sm(display_pio, true);
    display_sm_row = pio_claim_unused_sm(display_pio, true);

    display_offset_data = pio_add_program(display_pio, &hub75_data_program);
    display_offset_row = pio_add_program(display_pio, &hub75_row_program);

    hub75_data_program_init(
            display_pio,
            display_sm_data,
            display_offset_data,
            DISPLAY_DATAPINS_BASE,
            DISPLAY_CLKPIN
            );
    hub75_row_program_init(
            display_pio,
            display_sm_row,
            display_offset_row,
            DISPLAY_ROWSEL_BASE, DISPLAY_ROWSEL_COUNT,
            DISPLAY_STROBEPIN
            );

    // Initialize DMA
    display_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(display_dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, DREQ_PIO0_TX0+display_sm_data);

    dma_channel_configure(
            display_dma_chan,
            &c,
            &pio0_hw->txf[display_sm_data],
            NULL,  // Will be set later for each transfer
            DISPLAY_WIDTH*2,  // Because we shift out two rows at once
            false
            );
}

[[noreturn]] void __not_in_flash_func(hub75_main)() {
    puts("Hello from HUB75!");

    DISPLAY_REDRAWSTATE redrawstate = DISPLAY_REDRAWSTATE_IDLE;

    // Fill both framebuffers with a default pattern
    for (int x = 0; x < DISPLAY_WIDTH; ++x) {
        for (int y = 0; y < DISPLAY_HEIGHT; ++y) {
            uint32_t c = 2 << 16 | 0 << 8 | 0 << 0; //(x*4) << 16 | (y*4) << 8 | 16 << 0;
            if (y == 0) {
                c = x*(256/DISPLAY_WIDTH);
            }
            hub75_draw_pixel(display_front_buf, x, y, c);
            hub75_draw_pixel(display_back_buf, x, y, c);
        }
    }

    puts("HUB75 is done initializing framebuffers");

    // Main waits until we are done loading
    multicore_fifo_push_blocking(DISPLAY_REDRAW_DONE_MAGIC_NUMBER);

    absolute_time_t last_refresh = 0;
    uint32_t frame = 0;

    while (true) {
        // per-Frame loop
        frame++;
        last_refresh = get_absolute_time();

        for (int row = 0; row < DISPLAY_SCAN; ++row) {
            // per-Row loop

            for (int bit = 8-DISPLAY_BITDEPTH; bit < 8; ++bit) {
                // per-Bit level loop

                // Set correct bit offset in data SM
                hub75_data_set_shift(display_pio, display_sm_data, display_offset_data, bit);

                // Start DMA to push out pixels
                dma_channel_set_read_addr(display_dma_chan,
                                          &display_front_buf[row*DISPLAY_WIDTH*2],
                                          true);

                // Check SIO FIFO if new buffer is available
                if (   !display_flip
                    && multicore_fifo_rvalid()
                    && multicore_fifo_pop_blocking() == DISPLAY_TRIGGER_REDRAW_MAGIC_NUMBER) {
                    // New data available, set flag
                    display_flip = true;
                }

                // Wait for DMA completion to prevent dummy pixels being inserted at the wrong position
                // We could try and redraw here as well, but the dma (and shifting) itself is quite fast
                dma_channel_wait_for_finish_blocking(display_dma_chan);

                // Push two dummy pixels to properly synchronize
                // Also acts as a synchronization between row and data
                pio_sm_put_blocking(display_pio, display_sm_data, 0);
                pio_sm_put_blocking(display_pio, display_sm_data, 0);

                // Just in case the flags were still set
                hub75_pio_sm_clearstall();

                // Finish waiting if redraw was quick or not necessary
                // Also clears FIFO stall flags
                hub75_wait_tx_stall(display_pio, display_sm_data);
                hub75_wait_tx_stall(display_pio, display_sm_row);

                // Pulse LAT and OE using PIO
                pio_sm_put_blocking(display_pio,
                                    display_sm_row,
                                    row | ((16u * (1u << bit)) << DISPLAY_ROWSEL_COUNT)
                );
            }
        }

        // Ready to flip
        if (display_flip) {
            __compiler_memory_barrier();
            //printf("Flip\n");
            display_flip = false;

            // Swap buffers
            uint32_t* tmp = display_back_buf;
            display_back_buf = display_front_buf;
            display_front_buf = tmp;

            display_framenum++;

            __compiler_memory_barrier();

            // Send magic number back over FIFO to signal that the simulation buffer
            // can be reused
            // Keeps both cores synced
            if (!multicore_fifo_wready()) {
                // Should never happen, panic
                panic("Tried to signal finished redraw, but FIFO was full!\n");
            }
            multicore_fifo_push_blocking(DISPLAY_REDRAW_DONE_MAGIC_NUMBER);
        }

        if (frame % 1000 == 0) {
            absolute_time_t ct = get_absolute_time();
            int64_t td = absolute_time_diff_us(last_refresh, ct);
            float hz = 1/((float)td/1000/1000);
            hub75_hz = hz;
            // printf("Refresh: took %lldus -> %.3fHz\n", td, hz);
        }
    }
}

void __not_in_flash_func(hub75_blit_from_buffer)(const uint32_t* buf, uint32_t w, uint32_t h) {
    assert(w == DISPLAY_WIDTH);
    assert(h == DISPLAY_HEIGHT);

    for (int y = 0; y < DISPLAY_HEIGHT; ++y) {
        // Precalculate row address start
        int startaddr = y*DISPLAY_WIDTH*2;
        if (y >= DISPLAY_SCAN) {
            startaddr = (y-DISPLAY_SCAN) * DISPLAY_WIDTH*2 + 1;
        }

        for (int x = 0; x < DISPLAY_WIDTH; ++x) {
            display_back_buf[startaddr+2*x] = buf[y*DISPLAY_WIDTH+x];
        }
    }
}

bool hub75_pio_sm_stalled() {
    // Checks whether the state machines are stalled
    // We currently only check the row SM, since it will take longer for higher
    // bit levels
    uint32_t txstall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + display_sm_row);
    //txstall_mask |= 1u << (PIO_FDEBUG_TXSTALL_LSB + display_sm_data);
    return !(display_pio->fdebug & txstall_mask);
}

void hub75_pio_sm_clearstall() {
    uint32_t txstall_mask = 1u << (PIO_FDEBUG_TXSTALL_LSB + display_sm_row);
    txstall_mask |= 1u << (PIO_FDEBUG_TXSTALL_LSB + display_sm_data);
    display_pio->fdebug = txstall_mask;
}

static inline void __not_in_flash_func(hub75_draw_pixel)(uint32_t* buf, uint32_t x, uint32_t y, uint32_t color) {
    if (y<DISPLAY_SCAN) {
        // Normal, store pixel
        buf[y*DISPLAY_WIDTH*2+2*x] = color;
    } else {
        // Above display scan, store as interleaved
        buf[(y-DISPLAY_SCAN)*DISPLAY_WIDTH*2+2*x+1] = color;
    }
}