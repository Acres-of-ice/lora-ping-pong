#include "console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

static const char *TAG = "console";

#define CMD_LINE_MAX     300
#define CHUNK        256
// Most routes answer in milliseconds; the slow ones (a CSV of a long run) keep
// sending, and the timeout only counts idle time between reads.
#define RECV_IDLE_MS 3000

// Has the whole response arrived? Keep-alive means the server will not close the
// socket for us, so end of message is found the way a client would: Content-Length
// bytes after the headers, or a chunked body's terminating zero-length chunk.
typedef struct {
    size_t total;
    long   header_end;      // offset just past "\r\n\r\n", -1 until seen
    long   content_length;  // -1 when not given
    bool   chunked;
    char   head[1024];      // enough of the response to find the headers in
    char   tail[5];         // last bytes seen, for the chunked terminator
} resp_t;

static void resp_feed(resp_t *r, const char *buf, int n)
{
    if (r->header_end < 0) {
        const size_t room = sizeof(r->head) - 1 - r->total;
        const size_t copy = (size_t)n < room ? (size_t)n : room;
        memcpy(r->head + r->total, buf, copy);
        r->head[r->total + copy] = '\0';
        const char *end = strstr(r->head, "\r\n\r\n");
        if (end != NULL) {
            r->header_end = (end - r->head) + 4;
            // esp_http_server writes these exact spellings.
            const char *cl = strstr(r->head, "Content-Length: ");
            if (cl != NULL && cl < end) {
                r->content_length = strtol(cl + 16, NULL, 10);
            }
            const char *te = strstr(r->head, "Transfer-Encoding: chunked");
            r->chunked = (te != NULL && te < end);
        }
    }
    r->total += (size_t)n;
    for (int i = 0; i < n; i++) {
        memmove(r->tail, r->tail + 1, sizeof(r->tail) - 1);
        r->tail[sizeof(r->tail) - 1] = buf[i];
    }
}

static bool resp_done(const resp_t *r)
{
    if (r->header_end < 0) {
        return false;
    }
    if (r->chunked) {
        return memcmp(r->tail, "0\r\n\r\n", 5) == 0;
    }
    if (r->content_length >= 0) {
        return r->total >= (size_t)(r->header_end + r->content_length);
    }
    return false;  // neither: read until the server closes
}

// One framed line, written in a single call so a log line from another task can
// land between two of them but never inside one.
static void emit_data(int id, const char *buf, int n)
{
    static const char HEX[] = "0123456789abcdef";
    static char line[16 + 2 * CHUNK + 2];
    int k = snprintf(line, sizeof(line), "@@D %d ", id);
    for (int i = 0; i < n; i++) {
        line[k++] = HEX[(uint8_t)buf[i] >> 4];
        line[k++] = HEX[(uint8_t)buf[i] & 0x0F];
    }
    line[k++] = '\n';
    fwrite(line, 1, k, stdout);
    fflush(stdout);
}

static void fetch(int id, const char *path)
{
    printf("@@B %d %s\n", id, path);
    fflush(stdout);
    const int64_t t0 = esp_timer_get_time();

    const char *err = NULL;
    resp_t r = { .header_end = -1, .content_length = -1 };

    const int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0) {
        printf("@@E %d 0 socket failed\n", id);
        return;
    }
    const struct timeval tv = { .tv_sec = RECV_IDLE_MS / 1000, .tv_usec = 0 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in to = {
        .sin_family      = AF_INET,
        .sin_port        = htons(80),
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
    };
    if (connect(s, (struct sockaddr *)&to, sizeof(to)) != 0) {
        err = "connect failed (is the web server running?)";
    } else {
        char req[CMD_LINE_MAX + 96];
        const int n = snprintf(req, sizeof(req),
                               "GET %s HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n",
                               path);
        if (send(s, req, n, 0) != n) {
            err = "send failed";
        }
    }

    char buf[CHUNK];
    while (err == NULL && !resp_done(&r)) {
        const int n = recv(s, buf, sizeof(buf), 0);
        if (n == 0) {
            break;  // server closed: that is the end of it
        }
        if (n < 0) {
            err = r.total ? "timed out mid-response" : "no response";
            break;
        }
        resp_feed(&r, buf, n);
        emit_data(id, buf, n);
    }
    close(s);
    // The time goes after the error text's slot as "+<ms>", so a script can see
    // how long the board itself took.
    printf("@@E %d %u +%lld%s%s\n", id, (unsigned)r.total,
           (long long)((esp_timer_get_time() - t0) / 1000), err ? " " : "", err ? err : "");
    fflush(stdout);
}

static void console_task(void *arg)
{
    char line[CMD_LINE_MAX + 1];
    int  len = 0;
    int  next_id = 1;

    for (;;) {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY) != 1) {
            continue;
        }
        if (c != '\n' && c != '\r') {
            if (len < CMD_LINE_MAX) {
                line[len++] = (char)c;
            }
            continue;
        }
        line[len] = '\0';
        len = 0;

        char *p = line;
        while (*p == ' ') {
            p++;
        }
        int id = 0;
        if (*p == '@') {  // "@<id> <path>": the script picks the id
            id = (int)strtol(p + 1, &p, 10);
            while (*p == ' ') {
                p++;
            }
        }
        if (strncmp(p, "GET ", 4) == 0) {
            p += 4;
        }
        if (*p != '/') {
            if (*p != '\0') {
                printf("console: routes start with '/', e.g. /status\n");
            }
            continue;
        }
        if (id == 0) {
            id = next_id++;
        }
        fetch(id, p);
    }
}

esp_err_t console_start(void)
{
#if !CONFIG_USB_ROUTES
    if (true) {
        return ESP_OK;  // compiled either way, so it cannot rot unseen
    }
#endif
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    // A response is streamed out a line at a time; a larger ring lets the host
    // drain it in bigger reads. With no host attached the driver waits once for
    // 50 ms and then drops output rather than blocking, so a board in the field
    // with nothing on its USB port never stalls on a log line.
    cfg.tx_buffer_size = 2048;
    cfg.rx_buffer_size = 512;
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB serial driver failed (%s); no console", esp_err_to_name(err));
        return err;
    }
    usb_serial_jtag_vfs_use_driver();
    if (xTaskCreate(console_task, "console", 4096, NULL, 2, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Serial console ready: type a route, e.g. /status");
    return ESP_OK;
}
