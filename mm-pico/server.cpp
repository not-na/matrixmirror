#include "server.h"

#include <cyw43_configport.h>
#include <hardware/watchdog.h>
#include <hub75.h>
#include <lwip/tcp.h>
#include <main.h>

// Buffers are stored without preamble, the extra bytes are for the SPI mode
extern uint8_t buf_a[bufsize+sizeof(magic_preamble)];
extern uint8_t buf_b[bufsize+sizeof(magic_preamble)];

uint8_t* buf_recv = buf_a;
uint8_t* buf_present = buf_b;

tcp_pcb* server_pcb;
tcp_pcb* client_pcb;

bool is_receiving_preamble = true;
int preamble_state = 0;
int buf_remaining = 0;
int buf_offset = 0;

bool buf_ready = false;

auto_init_mutex(buf_mutex);

err_t server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        puts("Invalid data on recv");
        watchdog_reboot(0,0,0);
        busy_wait_ms(1000);
    }

    cyw43_arch_lwip_check();

    if (p->tot_len > 0) {
        //printf("Got packet: %d len\n", p->tot_len);
        // Got some data

        int recv_left = p->tot_len;
        int offset = 0;

        while (recv_left > 0) {
            if (is_receiving_preamble) {
                // Still in preamble, only consume single bytes at a time
                uint8_t p_tmp;
                int recvd = pbuf_copy_partial(p, &p_tmp, 1, offset);
                recv_left -= recvd;
                offset += recvd;

                if (recvd == 0) {
                    puts("Could not copy data from pbuf");
                    watchdog_reboot(0,0,0);
                    busy_wait_ms(1000);
                }

                // Do preamble SM
                if (p_tmp == magic_preamble[preamble_state]) {
                    preamble_state++;
                    if (preamble_state == 4) {
                        is_receiving_preamble = false;
                        buf_remaining = bufsize;
                        buf_offset = 0;
                    }
                } else {
                    // Fall back
                    preamble_state = 0;
                }
            } else {
                //printf("Buf rx rem=%d target=%d poff=%d\n", buf_remaining, MIN(buf_remaining, recv_left), offset);
                if (mutex_enter_timeout_ms(&buf_mutex, 1)) {
                    __compiler_memory_barrier();
                    if (buf_ready) {
                        printf("Not ready\n");
                        // Buffer not yet swapped, don't do anything
                        mutex_exit(&buf_mutex);
                        break;
                    }
                    mutex_exit(&buf_mutex);
                }

                // Framebuffer, consume as much as possible
                int recvd = pbuf_copy_partial(p, &buf_recv[buf_offset], MIN(buf_remaining, recv_left), offset);
                buf_offset += recvd;
                buf_remaining -= recvd;
                offset += recvd;
                recv_left -= recvd;

                if (buf_remaining == 0) {
                    // Buffer fully received
                    if (mutex_enter_timeout_ms(&buf_mutex, 1)) {
                        //printf("Mark as ready\n");
                        __compiler_memory_barrier();

                        buf_ready = true;
                        is_receiving_preamble = true;
                        preamble_state = 0;

                        __compiler_memory_barrier();
                        mutex_exit(&buf_mutex);
                    } else {
                        printf("No mutex\n");
                        // Could not get mutex in time, do nothing
                        // Next time this function is called we will try again
                        break;
                    }
                }
            }
        }

        //tcp_recved(tpcb, offset);
        if (offset != p->tot_len) {
            is_receiving_preamble = true;
            preamble_state = 0;
        }
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    return ERR_OK;
}

static void server_err(void *arg, err_t err) {
    printf("server error: %d\n", err);
    watchdog_reboot(0,0,0);
    busy_wait_ms(1000);
}

static err_t server_poll(void *arg, struct tcp_pcb *tpcb) {
    //printf("server poll\n");
    return ERR_OK;
}

static err_t server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    printf("server sent %d\n", len);
    return ERR_OK;
}

static err_t server_accept(void *arg, struct tcp_pcb *n_client_pcb, err_t err) {
    if (err != ERR_OK || n_client_pcb == nullptr) {
        panic("Failure in accept");
    }

    puts("Client connected");

    client_pcb = n_client_pcb;
    tcp_arg(client_pcb, nullptr);
    tcp_sent(client_pcb, server_sent);
    tcp_recv(client_pcb, server_recv);
    tcp_err(client_pcb, server_err);
    tcp_poll(client_pcb, server_poll, 10);

    return ERR_OK;
}

void server_init() {
    printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), tcp_port);

    tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        panic("Failed to create PCB");
    }

    err_t err = tcp_bind(pcb, nullptr, tcp_port);
    if (err) {
        panic("Failed to bind to port");
    }

    server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!server_pcb) {
        if (pcb) {
            tcp_close(pcb);
        }

        panic("Failed to listen");
    }

    tcp_arg(server_pcb, nullptr);
    tcp_accept(server_pcb, server_accept);
}

uint8_t* server_get_cur_buf() {
    if (!mutex_enter_timeout_ms(&buf_mutex, 1000)) {
        puts("Mutex timed out!");
        watchdog_reboot(0,0,0);
        busy_wait_ms(1000);
    }
    __compiler_memory_barrier();

    uint8_t* out = nullptr;
    if (buf_ready) {
        // Got a buffer ready to process

        // Swap around buffers
        out = buf_recv;
        buf_recv = buf_present;
        buf_present = out;

        // Mark recv as not ready (as in not filled) so it can be filled again
        buf_ready = false;
    }

    __compiler_memory_barrier();
    mutex_exit(&buf_mutex);

    return out;
}
