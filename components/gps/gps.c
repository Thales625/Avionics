#include "gps.h"

#include "driver/uart.h"
#include <esp_idf_lib_helpers.h>

#include <string.h>

// #define ENABLE_LOG

#ifdef ENABLE_LOG
#include "esp_log.h"
static const char *TAG = "gps";
#endif

static char gps_line[128];
static int gps_idx = 0;

static const char* nmea_next_field(const char *p) {
    while (*p && *p != ',' && *p != '*') p++;
    if (*p == ',' || *p == '*') p++;
    return p;
}

static bool gps_check_checksum(const char *sentence) {
    if (sentence[0] != '$') return false;

    uint8_t checksum = 0;
    const char *p = sentence + 1;

    while (*p && *p != '*') {
        checksum ^= *p;
        p++;
    }

    if (*p != '*' || strlen(p) < 3) return false;
    p++; // skip '*'

    char hex[3] = {p[0], p[1], '\0'};
    uint8_t expected = (uint8_t)strtol(hex, NULL, 16);

    return checksum == expected;
}

static int32_t parse_nmea_coord(const char *s) {
    int32_t value = 0;

    while (*s && *s != ',' && *s != '*') {
        if (*s != '.') {
            value = value * 10 + (*s - '0');
        }
        s++;
    }

    return value;
}

static void parse_gpgga(int32_t *lat, int32_t *lon, uint8_t *satellites, const char *line) {
    const char *p = line;

    p = nmea_next_field(p); // skip [0]: msg ID
    p = nmea_next_field(p); // skip [1]: UTC

    const char *lat_str = p; // [2]: latitude
    p = nmea_next_field(p);
    const char *ns_str = p; // [3]: (n/s)
    p = nmea_next_field(p);
    const char *lon_str = p; // [4]: longitude
    p = nmea_next_field(p);
    const char *ew_str = p; //[5]: (e/w)
    p = nmea_next_field(p);
    const char *fix_str = p; // [6]: quality (0 = invalid, 1 = fixed)
    p = nmea_next_field(p);
    const char *sat_str = p; // [7]: satellites

    if (*fix_str >= '1' && *fix_str <= '9') {
        if (*sat_str != ',' && *sat_str != '*') {
            *satellites = (uint8_t)strtoul(sat_str, NULL, 10);
        }

        if (*lat_str != ',' && *lat_str != '*') {
            *lat = parse_nmea_coord(lat_str);
            if (*ns_str == 'S') *lat = -*lat;
        }

        if (*lon_str != ',' && *lon_str != '*') {
            *lon = parse_nmea_coord(lon_str);
            if (*ew_str == 'W') *lon = -*lon;
        }
    } else {
        *satellites = 0;
    }
}

esp_err_t gps_init_desc(gps_dev_t *dev, gpio_num_t tx, gpio_num_t rx, uart_port_t uart_num) {
    uart_config_t uart_config = {
        .baud_rate  = GPS_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    dev->uart_num = uart_num;

    if (uart_param_config(dev->uart_num, &uart_config) != ESP_OK) {
        return ESP_FAIL;
    }
    if (uart_set_pin(dev->uart_num, rx, tx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        return ESP_FAIL;
    }
    if (uart_driver_install(dev->uart_num, GPS_BUF_SIZE, 0, 0, NULL, 0) != ESP_OK) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t gps_update(gps_dev_t *dev) {
    uint8_t data[128];
    int len;
    // int len = uart_read_bytes(dev->uart_num, data, sizeof(data) - 1, pdMS_TO_TICKS(10));

    while (1) {
        len = uart_read_bytes(dev->uart_num, data, sizeof(data), 0);

        if (len <= 0) break;

        for (int i = 0; i < len; i++) {
            uint8_t byte = data[i];

            if (byte == '\r') {
                continue; // ignore
            } else if (byte == '\n') {
                // EOL
                gps_line[gps_idx] = '\0';

                #ifdef ENABLE_LOG
                ESP_LOGI(TAG, "raw: %s", gps_line);
                #endif

                // validate packet
                if (gps_check_checksum(gps_line)) {
                    if (strncmp(gps_line, "$GPGGA", 6) == 0 || strncmp(gps_line, "$GNGGA", 6) == 0) {
                        // parse_gpgga(dev, gps_line);

                        #ifdef ENABLE_LOG
                        ESP_LOGI(TAG, "Lock=%d, Sats=%lu, Lat=%.6f, Lon=%.6f", dev->locked, dev->sattelites, dev->lat, dev->lon);
                        #endif
                    }
                } else {
                    #ifdef ENABLE_LOG
                    ESP_LOGW(TAG, "Checksum error, NMEA ignored");
                    #endif
                }

                gps_idx = 0;
            } else if (gps_idx < sizeof(gps_line) - 1) {
                gps_line[gps_idx++] = byte;
            } else {
                // FAILSAFE: buffer full and didn't find EOL.
                gps_line[gps_idx] = '\0';
                #ifdef ENABLE_LOG
                ESP_LOGW(TAG, "BUFFER FULL (wrong Baud rate?): %s", gps_line);
                #endif
                gps_idx = 0;
            }
        }
    }

    return ESP_OK;
}

esp_err_t gps_read(gps_dev_t *dev, int32_t *lat, int32_t *lon, uint8_t *satellites) {
    uint8_t data[128];
    int len;

    while (1) {
        len = uart_read_bytes(dev->uart_num, data, sizeof(data), 0);

        if (len <= 0) break;

        for (int i = 0; i < len; i++) {
            uint8_t byte = data[i];

            if (byte == '\r') {
                continue; // ignore
            } else if (byte == '\n') {
                // EOL
                gps_line[gps_idx] = '\0';

                #ifdef ENABLE_LOG
                ESP_LOGI(TAG, "raw: %s", gps_line);
                #endif

                // validate packet
                if (gps_check_checksum(gps_line)) {
                    if (strncmp(gps_line, "$GPGGA", 6) == 0 || strncmp(gps_line, "$GNGGA", 6) == 0) {
                        parse_gpgga(lat, lon, satellites, gps_line);

                        #ifdef ENABLE_LOG
                        ESP_LOGI(TAG, "Lock=%d, Sats=%lu, Lat=%.6f, Lon=%.6f", dev->locked, dev->sattelites, dev->lat, dev->lon);
                        #endif
                    }
                } else {
                    #ifdef ENABLE_LOG
                    ESP_LOGW(TAG, "Checksum error, NMEA ignored");
                    #endif
                }

                gps_idx = 0;
            } else if (gps_idx < sizeof(gps_line) - 1) {
                gps_line[gps_idx++] = byte;
            } else {
                // FAILSAFE: buffer full and didn't find EOL.
                gps_line[gps_idx] = '\0';
                #ifdef ENABLE_LOG
                ESP_LOGW(TAG, "BUFFER FULL (wrong Baud rate?): %s", gps_line);
                #endif
                gps_idx = 0;
            }
        }
    }

    return ESP_OK;
}