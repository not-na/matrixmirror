#include "main.h"

#include <ctime>


int main() {
    stdio_init_all();
    puts("mm-pico booting up...\n");

    // Initialize hub75
    hub75_init();

    multicore_launch_core1(hub75_main);
    sleep_ms(100);

    uint32_t frame = 0;
    absolute_time_t frame_time = get_absolute_time();

    bool last_loop_rendered = false;
    absolute_time_t last_warn = get_absolute_time();
    uint32_t warn_count = 0;

    while (true) {
        if (time_reached(delayed_by_ms(frame_time, 1000/display_fps))) {
            frame_time = get_absolute_time();

            if (last_loop_rendered) {
                // Close to the limit, warn
                warn_count++;
                if (time_reached(delayed_by_ms(last_warn, 1000))) {
                    printf("WARNING: CPU may be overloaded, warning counter: %lu\n", warn_count);
                    last_warn = get_absolute_time();
                }
            }

            // TODO: fetch frame data

            // TODO: update display buffer from frame data
            // display_background = ...

            // Done here, trigger redraw from second core
            //multicore_fifo_push_blocking(DISPLAY_TRIGGER_REDRAW_MAGIC_NUMBER);

            frame++;
            last_loop_rendered = true;

            // Timing stats printed every second
            absolute_time_t et = get_absolute_time();
            if (frame % (display_fps/1) == 0) {
                int64_t ft = absolute_time_diff_us(frame_time, et);
                printf("Frametime=%lldus (max=%dus) cpu=%.3f%%\n", ft, 1000000/display_fps,  ((int32_t)ft)/(1000000.0/display_fps)*100);
            }
        } else {
            last_loop_rendered = false;
            busy_wait_until(delayed_by_ms(frame_time, 1000/display_fps));
        }
    }
}
