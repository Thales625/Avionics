#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/projdefs.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "portmacro.h"

#include <math.h>
#include <stdint.h>

#include "flight_logic.h"
#include "flash_log.h"
#include "flash_interface.h"
#include "tmtc.h"
#include "crc.h"

#include "mpu6050.h"
#include "bmp280.h"
#include "lora.h"
#include "gps.h"
#include "w25q64.h"

#define AVIONICS_ERROR_CHECK(x, code, reason) do { \
    esp_err_t __err_rc = (x); \
    if (__err_rc != ESP_OK) { \
        ESP_LOGE(TAG, \
            "(ABORT) %s (%s:%d): %s", \
            reason, __FILE__, __LINE__, \
            esp_err_to_name(__err_rc) \
        ); \
        avionics_abort(code); \
    } \
} while (0)

typedef enum {
    ABORT_GPIO_INIT = 1,
    ABORT_W25QXX_INIT = 2,
    ABORT_FLASH_LOG_INIT = 3,
    ABORT_I2C_INIT = 4,
    ABORT_BMP280_INIT = 5,
    ABORT_MPU6050_INIT = 6,
    ABORT_LORA_INIT = 7,
    ABORT_GPS_INIT = 8,
    ABORT_SENSOR_READING = 9,
    ABORT_USB_UART_INIT = 10,
} abort_code_t;

#define AVIONICS_INTERVAL pdMS_TO_TICKS(40) // 40 ms = 25 Hz
#define BOOT_TIMEOUT pdMS_TO_TICKS(5000)

#define SPI_MISO GPIO_NUM_22
#define SPI_MOSI GPIO_NUM_19
#define SPI_CLK GPIO_NUM_21
#define W25Q_CS GPIO_NUM_23
#define FLASH_SAMPLING 5

#define USB_BAUD_RATE 115200
#define UART_PORT_USB UART_NUM_0

#define LORA_TX GPIO_NUM_32
#define LORA_RX GPIO_NUM_33
#define LORA_M0 GPIO_NUM_26
#define LORA_M1 GPIO_NUM_25
#define LORA_AUX GPIO_NUM_34
#define LORA_UART UART_NUM_2
#define LORA_BAUD_RATE 9600
#define LORA_SAMPLING 10

#define GPS_TX GPIO_NUM_18
#define GPS_RX GPIO_NUM_5
#define GPS_UART UART_NUM_1

// #define SDA_GPIO_PIN GPIO_NUM_14
// #define SCL_GPIO_PIN GPIO_NUM_13

#define SDA_GPIO_PIN GPIO_NUM_16
#define SCL_GPIO_PIN GPIO_NUM_27
#define I2C_PORT I2C_NUM_0
#define MAX_I2C_RECOVERIES 5

#define BAT_R1 100000.0f
#define BAT_R2 47000.0f
#define BAT_MULTIPLIER ((BAT_R1 + BAT_R2) / BAT_R2)
#define BAT_SENSE_PIN GPIO_NUM_36
#define BAT_SAMPLING 25

#define PARACHUTE_PIN GPIO_NUM_4
#define LED_PIN GPIO_NUM_2
#define BOOT_PIN GPIO_NUM_0

static const char* TAG = "avionics";

static flight_logic_t flight_logic;

static lora_dev_t lora_dev = { 0 };
static gps_dev_t gps_dev = { 0 };
static bmp280_t bmp_dev = { 0 };
static mpu6050_dev_t mpu_dev = { 0 };

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle;

static QueueHandle_t flash_queue;
static QueueHandle_t lora_queue;
static QueueHandle_t telecommand_queue;

static void arm_systems(void) {
    if (flight_logic.should_arm == true){
        return;
    }

    flight_logic.should_arm = true;
    flash_log_start_flight();
}

static void disarm_systems(void) {
    if (flight_logic.should_arm == false){
        return;
    }

    flight_logic.should_arm = false;
    flash_log_finish_flight(flight_logic.state.ut - flight_logic.ut_0);
}

static float read_battery_voltage(void) {
    int raw_adc;
    int gpio_mv;

    adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &raw_adc);

    adc_cali_raw_to_voltage(adc1_cali_handle, raw_adc, &gpio_mv);

    float gpio_voltage = gpio_mv / 1000.0f;

    return gpio_voltage * BAT_MULTIPLIER;
}

static void blink(uint32_t duration) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(duration));
    gpio_set_level(LED_PIN, 0);
}

static void avionics_abort(int code) {
    // set safe state
    gpio_set_level(PARACHUTE_PIN, 0);
    gpio_set_level(LED_PIN, 0);

    // abort loop
    while (1) {
        for (int i=0; i<code; i++) {
            blink(200);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


static void avionics_task(void *arg) {
    // lora
    uint32_t lora_counter = 0;
    lora_payload_t tm_payload;
    telecommand_payload_t tc_payload;

    // flash
    uint32_t flash_counter = 0;
    flash_payload_t flash_payload;

    // i2c sensors
    uint32_t i2c_recoveries = 0;

    // battery
    uint32_t battery_counter = 0;
    float battery_voltage = read_battery_voltage();

    // UTC date & time
    uint32_t utc_time = 0;
    uint32_t utc_date = 0;

    // init flight logic core
    flight_logic.state.ut = (uint32_t)(esp_timer_get_time() / 1000ULL);

    AVIONICS_ERROR_CHECK(
        bmp280_read_float(&bmp_dev, &flight_logic.state.temperature, &flight_logic.state.pressure),
        ABORT_SENSOR_READING,
        "BMP280 initial reading failed"
    );
    AVIONICS_ERROR_CHECK(
        mpu6050_get_motion(&mpu_dev, &flight_logic.state.accel, &flight_logic.state.ang_vel),
        ABORT_SENSOR_READING,
        "MPU6050 initial reading failed"
    );
    flight_logic_init(&flight_logic);

    // frequency
    TickType_t last_tick = xTaskGetTickCount();
    const TickType_t interval = AVIONICS_INTERVAL;

    // main loop
    while (1) {
        vTaskDelayUntil(&last_tick, interval);

        // update ut
        flight_logic.state.ut = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // i2c sensors
        bool i2c_ok = true;

        // BMP280: read data
        if (bmp280_read_float(&bmp_dev, &flight_logic.state.temperature, &flight_logic.state.pressure) != ESP_OK) {
            ESP_LOGW(TAG, "BMP280: read failed");

            flight_logic.state.temperature = NAN;
            flight_logic.state.pressure = NAN;

            i2c_ok = false;
        }

        // MPU6050: read data
        if (mpu6050_get_motion(&mpu_dev, &flight_logic.state.accel, &flight_logic.state.ang_vel) != ESP_OK) {
            ESP_LOGW(TAG, "MPU6050: read failed");

            flight_logic.state.accel.x = NAN;
            flight_logic.state.accel.y = NAN;
            flight_logic.state.accel.z = NAN;

            flight_logic.state.ang_vel.x = NAN;
            flight_logic.state.ang_vel.y = NAN;
            flight_logic.state.ang_vel.z = NAN;

            i2c_ok = false;
        }

        if (i2c_ok) {
            i2c_recoveries = 0;
        } else if (flight_logic.state.phase < PHASE_ASCENT) {
            i2c_recoveries++;

            ESP_LOGW(TAG, "Recovering I2C bus (%d/%d)", i2c_recoveries, MAX_I2C_RECOVERIES);

            i2cdev_bus_recover(I2C_PORT);
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // GPS: read data
        gps_read(&gps_dev, &flight_logic.state.lat_nmea, &flight_logic.state.lon_nmea, &flight_logic.state.satellites, &utc_time, &utc_date);

        // set flash log UTC time if available
        if (utc_time != 0 && utc_date != 0 && flight_logic.state.lat_nmea != 0 && flight_logic.state.lon_nmea != 0) {
            flash_log_set_gps_data(utc_time, utc_date, flight_logic.state.lat_nmea, flight_logic.state.lon_nmea);
        }

        // BAT: read voltage
        if (battery_counter++ >= BAT_SAMPLING) {
            battery_counter = 0;
            float current_vbat = read_battery_voltage();

            battery_voltage = (0.9f * battery_voltage) + (0.1f * current_vbat);
        }

        // consume telecommand
        {
            if (xQueueReceive(telecommand_queue, &tc_payload, 0) == pdTRUE) {
                ESP_LOGI("telecommand", "id=%d param=%d", tc_payload.id, tc_payload.param);
                switch (tc_payload.id) {
                    case TC_DISARM:
                        if (tc_payload.param == TELECOMMAND_MAGIC) {
                            disarm_systems();
                            ESP_LOGI("telecommand", "disarmed");
                        }
                        break;

                    case TC_ARM:
                        if (tc_payload.param == TELECOMMAND_MAGIC) {
                            arm_systems();
                            ESP_LOGI("telecommand", "armed");
                        }
                        break;

                    case TC_PARACHUTE_EJECT:
                        if (flight_logic.state.phase >= PHASE_PRE_FLIGHT && tc_payload.param == TELECOMMAND_MAGIC) {
                            flight_logic.state.phase = PHASE_PARACHUTE_DEPLOY;
                            ESP_LOGI("telecommand", "parachute eject");
                        }
                        break;

                    default:
                        break;
                }
            }
        }

        // update flight logic
        flight_logic_update(&flight_logic);

        // set values
        gpio_set_level(PARACHUTE_PIN, flight_logic.trigger_parachute);

        if (flight_logic.trigger_shutdown) {
            flash_log_finish_flight(flight_logic.state.ut - flight_logic.ut_0);
        }

        // log values
        ESP_LOGI(TAG, "phase: %d, v_bat: %.2f, altitude: %.6f, pressure: %.2f, |accel|: %.2f, sats: %d, lat: %d, lon: %d", flight_logic.state.phase, battery_voltage, flight_logic.altitude_baro, flight_logic.state.pressure, sqrtf(flight_logic.state.accel.x*flight_logic.state.accel.x + flight_logic.state.accel.y*flight_logic.state.accel.y + flight_logic.state.accel.z*flight_logic.state.accel.z), flight_logic.state.satellites, flight_logic.state.lat_nmea, flight_logic.state.lon_nmea);
        // ESP_LOGI(TAG, "ut: %lu, gps_date: %lu, gps_time: %lu, satellites: %d", flight_logic.state.ut, utc_date, utc_time, flight_logic.state.satellites);

        // send data to telemetry
        {
            if (lora_counter++ >= LORA_SAMPLING) {
                lora_counter = 0;

                tm_payload.ut = flight_logic.state.ut;
                tm_payload.accel_mag = sqrtf(flight_logic.state.accel.x*flight_logic.state.accel.x + flight_logic.state.accel.y*flight_logic.state.accel.y + flight_logic.state.accel.z*flight_logic.state.accel.z);
                tm_payload.ang_vel_mag = sqrtf(flight_logic.state.ang_vel.x*flight_logic.state.ang_vel.x + flight_logic.state.ang_vel.y*flight_logic.state.ang_vel.y + flight_logic.state.ang_vel.z*flight_logic.state.ang_vel.z);
                tm_payload.pressure = flight_logic.state.pressure;
                tm_payload.temperature = flight_logic.state.temperature;
                tm_payload.altitude = flight_logic.altitude_baro;
                tm_payload.lat_nmea = flight_logic.state.lat_nmea;
                tm_payload.lon_nmea = flight_logic.state.lon_nmea;
                tm_payload.satellites = flight_logic.state.satellites;
                tm_payload.v_bat = battery_voltage;
                tm_payload.phase = (uint8_t) flight_logic.state.phase;

                xQueueOverwrite(lora_queue, &tm_payload);
            }

            // send data to flash
            if (flight_logic.state.phase >= PHASE_PRE_FLIGHT) {
                if (flash_counter++ >= FLASH_SAMPLING) {
                    flash_counter = 0;

                    flash_payload.ut = flight_logic.state.ut;
                    flash_payload.accel = flight_logic.state.accel;
                    flash_payload.ang_vel = flight_logic.state.ang_vel;
                    flash_payload.pressure = flight_logic.state.pressure;
                    flash_payload.temperature = flight_logic.state.temperature;
                    flash_payload.lat_nmea = flight_logic.state.lat_nmea;
                    flash_payload.lon_nmea = flight_logic.state.lon_nmea;
                    flash_payload.satellites = flight_logic.state.satellites;
                    flash_payload.v_bat = battery_voltage;
                    flash_payload.phase = (uint8_t) flight_logic.state.phase;

                    if (xQueueSend(flash_queue, &flash_payload, 0) != pdTRUE) {
                        flash_payload_t discarded;
                        xQueueReceive(flash_queue, &discarded, 0);
                        xQueueSend(flash_queue, &flash_payload, 0);
                        ESP_LOGW(TAG, "discard old flash sample: flash queue is full!");
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
}

static void flash_task(void *arg) {
    flash_payload_t payload;

    while (1) {
        if (xQueueReceive(flash_queue, &payload, portMAX_DELAY)) {
            flash_log_append(&payload);
        }
    }

    vTaskDelete(NULL);
}

static void lora_task(void *arg) {
    // telemetry
    lora_packet_t packet;
    lora_payload_t payload;
    packet.magic = TELEMETRY_MAGIC;

    // telecommand
    telecommand_payload_t telecommand;
    uint32_t tc_magic = 0;
    uint16_t tc_checksum = 0;
    uint8_t tc_byte;

    while (1) {
        // receive telecommand
        for (uint32_t tc_reads=0; tc_reads<32; tc_reads++) {
            if (lora_receive_bytes(&lora_dev, &tc_byte, 1, 0) <= 0) {
                break;
            }

            tc_magic = (tc_magic >> 8) | ((uint32_t)tc_byte << 24); // little-endian
            // tc_magic = (tc_magic << 8) | tc_byte;

            if (tc_magic == TELECOMMAND_MAGIC) {
                int payload_len = lora_receive_bytes(&lora_dev, (uint8_t *)&telecommand, sizeof(telecommand_payload_t), pdMS_TO_TICKS(100));
                if (payload_len == sizeof(telecommand_payload_t)) {
                    int checksum_len = lora_receive_bytes(&lora_dev, (uint8_t *)&tc_checksum, sizeof(tc_checksum), pdMS_TO_TICKS(50));
                    if (checksum_len == sizeof(tc_checksum)) {
                        if (tc_checksum == crc16((const uint8_t*)&telecommand, sizeof(telecommand_payload_t))) {
                            if (xQueueSend(telecommand_queue, &telecommand, 0) != pdTRUE) {
                                telecommand_payload_t discarded;
                                xQueueReceive(telecommand_queue, &discarded, 0);
                                xQueueSend(telecommand_queue, &telecommand, 0);
                                ESP_LOGW(TAG, "discard old tc sample: telecommand queue is full!");
                            }
                        }
                    }
                }

                tc_magic = 0;
                break;
            }
        }

        // transmit telemetry
        if (xQueueReceive(lora_queue, &payload, 0) == pdTRUE) {
            packet.payload = payload;
            packet.checksum = crc16((const uint8_t*)&payload, sizeof(lora_payload_t));

            if (lora_send_bytes(&lora_dev, (uint8_t *)&packet, sizeof(packet)) == -1) {
                ESP_LOGE(TAG, "LORA send failed");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}


static void flash_interface_task(void *arg) {
    // init usb uart
    {
        uart_config_t uart_config = {
            .baud_rate = USB_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

        esp_err_t err = ESP_OK;

        err |= uart_driver_install(UART_PORT_USB, 256, 256, 0, NULL, 0);
        err |= uart_param_config(UART_PORT_USB, &uart_config);

        err |= uart_set_pin(UART_PORT_USB, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

        err |= uart_flush(UART_PORT_USB);
        err |= uart_flush_input(UART_PORT_USB);

        AVIONICS_ERROR_CHECK(
            err, // error code is not meaningful, only success/failure
            ABORT_USB_UART_INIT,
            "Flash Interface USB UART init failed"
        );
    }

    uint8_t rx_byte;
    uint32_t rx_magic = 0;
    uint32_t rx_id;
    int32_t rx_param;

    while (1) {
        // read usb uart port
        if (uart_read_bytes(UART_PORT_USB, &rx_byte, sizeof(rx_byte), portMAX_DELAY) > 0) {
            // receive usb bytes
            rx_magic = (rx_magic >> 8) | ((uint32_t)rx_byte << 24); // little-endian

            if (rx_magic == FLASH_USB_MAGIC) {
                if (uart_read_bytes(UART_PORT_USB, &rx_id, sizeof(rx_id), pdMS_TO_TICKS(200)) == sizeof(rx_id)) {
                    if (uart_read_bytes(UART_PORT_USB, &rx_param, sizeof(rx_param), pdMS_TO_TICKS(200)) == sizeof(rx_param)) {
                        flash_header_t* headers;
                        uint32_t headers_len;

                        flash_header_t header;

                        uint32_t header_addr;
                        flash_packet_t packet;

                        switch (rx_id) {
                            case CMD_ACK:
                                // send ack
                                uart_write_bytes(UART_PORT_USB, &flash_ack, sizeof(flash_ack));
                                break;
                            case CMD_CLEAR_FLIGHTS:
                                if (flash_log_clear_flights() == ESP_OK) {
                                    uart_write_bytes(UART_PORT_USB, &flash_ack, sizeof(flash_ack));
                                } else {
                                    uart_write_bytes(UART_PORT_USB, &flash_nack, sizeof(flash_nack));
                                }
                                break;
                            case CMD_LIST_HEADERS:
                                headers = flash_log_get_headers(&headers_len);

                                if (headers_len != 0) {
                                    // transmit headers
                                    uart_write_bytes(UART_PORT_USB, (uint8_t *)headers, headers_len*sizeof(flash_header_t));
                                }

                                uart_write_bytes(UART_PORT_USB, &flash_ack, sizeof(flash_ack));

                                free(headers);
                                break;
                            case CMD_READ_HEADER: // flight_number = rx_param
                                if (flash_log_get_header(rx_param, &header, NULL) == ESP_OK) {
                                    uart_write_bytes(UART_PORT_USB, (uint8_t *)&header, sizeof(flash_header_t));
                                    uart_write_bytes(UART_PORT_USB, &flash_ack, sizeof(flash_ack));
                                } else {
                                    uart_write_bytes(UART_PORT_USB, &flash_nack, sizeof(flash_nack));
                                }

                                break;
                            case CMD_READ_FLIGHT: // flight_number = rx_param
                                if (flash_log_get_header(rx_param, &header, &header_addr) == ESP_OK) {
                                    for (uint32_t addr=header_addr+header.header_size; addr<header.next_header_addr; addr+=header.packet_size) {
                                        if (flash_log_get_flight_packet(addr, header.packet_size, &packet) != ESP_OK) {
                                            uart_write_bytes(UART_PORT_USB, &flash_nack, sizeof(flash_nack));
                                            break;
                                        }

                                        uart_write_bytes(UART_PORT_USB, (uint8_t *)&packet, header.packet_size);
                                        vTaskDelay(pdMS_TO_TICKS(30));
                                    }
                                    uart_write_bytes(UART_PORT_USB, &flash_ack, sizeof(flash_ack));
                                } else {
                                    uart_write_bytes(UART_PORT_USB, &flash_nack, sizeof(flash_nack));
                                }
                                break;
                            default:
                                uart_write_bytes(UART_PORT_USB, &flash_nack, sizeof(flash_nack));
                                break;
                        }

                    }
                }
                rx_magic = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    vTaskDelete(NULL);
}

void app_main(void) {
    // GPIO OUT configuration
    {
        gpio_config_t out_conf = {
            .pin_bit_mask = (1ULL << PARACHUTE_PIN) |
                            (1ULL << LED_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        AVIONICS_ERROR_CHECK(
            gpio_config(&out_conf),
            ABORT_GPIO_INIT,
            "GPIO OUT config failed"
        );

        // set default values
        gpio_set_level(PARACHUTE_PIN, 0);
        gpio_set_level(LED_PIN, 0);
    }

    // GPIO IN configuration
    {
        gpio_config_t in_conf = {
            .pin_bit_mask = (1ULL << BOOT_PIN) |
                            (1ULL << BAT_SENSE_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        AVIONICS_ERROR_CHECK(
            gpio_config(&in_conf),
            ABORT_GPIO_INIT,
            "GPIO IN config failed"
        );
    }

    // battery ADC configuration (one-shot)
    {
        gpio_config_t in_bat_conf = {
            .pin_bit_mask = (1ULL << BAT_SENSE_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };

        AVIONICS_ERROR_CHECK(
            gpio_config(&in_bat_conf),
            ABORT_GPIO_INIT,
            "Battery GPIO config failed"
        );

        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
        };

        AVIONICS_ERROR_CHECK(
            adc_oneshot_new_unit(&init_config1, &adc1_handle),
            ABORT_GPIO_INIT,
            "ADC unit failed to init"
        );

        adc_oneshot_chan_cfg_t adc_config = {
            .bitwidth = ADC_BITWIDTH_12,
            .atten = ADC_ATTEN_DB_12,
        };

        AVIONICS_ERROR_CHECK(
            adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &adc_config),
            ABORT_GPIO_INIT,
            "ADC channel config failed"
        );

        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };

        #if CONFIG_IDF_TARGET_ESP32
        adc_cali_line_fitting_efuse_val_t efuse_type;
        esp_err_t err = adc_cali_scheme_line_fitting_check_efuse(&efuse_type);
        if (err == ESP_OK && efuse_type == ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF) {
            cali_config.default_vref = 1100; // used only if no calibration data exists in eFuse
        }
        #endif

        AVIONICS_ERROR_CHECK(
            adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle),
            ABORT_GPIO_INIT,
            "ADC calibration init failed"
        );
    }

    // W25Q64 initialization
    {
        AVIONICS_ERROR_CHECK(
            w25q64_init(SPI_MOSI, SPI_MISO, SPI_CLK, W25Q_CS),
            ABORT_W25QXX_INIT,
            "W25Q64 failed to init"
        );
        ESP_LOGI(TAG, "W25Q64 initialized");
    }

    // Flash Log initialization
    {
        AVIONICS_ERROR_CHECK(
            flash_log_init(),
            ABORT_FLASH_LOG_INIT,
            "Flash log failed to init"
        );
    }

    // check boot button (Flash Interface)
    {
        gpio_set_level(LED_PIN, 1); // LED on

        ESP_LOGI(TAG, "Waiting for boot button press...");

        bool boot_pressed = false;

        // waiting
        TickType_t start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start) < BOOT_TIMEOUT) {
            if (gpio_get_level(BOOT_PIN) == 0) {
                // simple debounce
                vTaskDelay(pdMS_TO_TICKS(30));

                if (gpio_get_level(BOOT_PIN) == 0) {
                    boot_pressed = true;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        gpio_set_level(LED_PIN, 0); // LED off

        // flash interface
        if (boot_pressed) {
            // DEBUG
            /*
            ESP_LOGI(TAG, "Clearing flights");
            if (flash_log_clear(0) == ESP_OK) {
                ESP_LOGI(TAG, "flights cleared");
            } else {
                ESP_LOGE(TAG, "failed to clear flights");
            }
            return;
            */

            ESP_LOGI(TAG, "Listing flights:");
            flash_log_list_flights();

            ESP_LOGI(TAG, "Starting Flash Interface...");
            vTaskDelay(pdMS_TO_TICKS(100));
            xTaskCreate(flash_interface_task, "flash_interface", 4096, NULL, 5, NULL);
            return;
        }

        ESP_LOGI(TAG, "Boot button not pressed");
    }

    // I2C initialization
    {
        AVIONICS_ERROR_CHECK(
            i2cdev_init(),
            ABORT_I2C_INIT,
            "I2C failed to init"
        );
        ESP_LOGI(TAG, "I2C initialized");
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // BMP280 initialization
    {
        bmp280_params_t params;
        params.mode = BMP280_MODE_NORMAL;
        params.filter = BMP280_FILTER_2;
        params.oversampling_pressure = BMP280_STANDARD;
        params.oversampling_temperature = BMP280_ULTRA_LOW_POWER;
        params.standby = BMP280_STANDBY_05;

        bmp280_init_desc(
            &bmp_dev,
            BMP280_I2C_ADDRESS_0,
            I2C_PORT,
            SDA_GPIO_PIN,
            SCL_GPIO_PIN
        );
        vTaskDelay(pdMS_TO_TICKS(200));
        AVIONICS_ERROR_CHECK(
            bmp280_init(&bmp_dev, &params),
            ABORT_BMP280_INIT,
            "BMP280 failed to init"
        );
        ESP_LOGI(TAG, "BMP280 initialized");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // MPU6050 initialization
    {
        mpu6050_init_desc(
            &mpu_dev,
            MPU6050_I2C_ADDRESS_LOW,
            I2C_PORT,
            SDA_GPIO_PIN,
            SCL_GPIO_PIN
        );
        vTaskDelay(pdMS_TO_TICKS(200));
        AVIONICS_ERROR_CHECK(
            mpu6050_init(&mpu_dev),
            ABORT_MPU6050_INIT,
            "MPU6050 failed to init"
        );
        AVIONICS_ERROR_CHECK(
            mpu6050_set_full_scale_gyro_range(&mpu_dev, MPU6050_GYRO_RANGE_250),
            ABORT_MPU6050_INIT,
            "MPU6050 failed to set gyro range"
        );
        AVIONICS_ERROR_CHECK(
            mpu6050_set_full_scale_accel_range(&mpu_dev, MPU6050_ACCEL_RANGE_8),
            ABORT_MPU6050_INIT,
            "MPU6050 failed to set accel range"
        );
        AVIONICS_ERROR_CHECK(
            mpu6050_set_dlpf_mode(&mpu_dev, MPU6050_DLPF_3),
            ABORT_MPU6050_INIT,
            "MPU6050 failed to set DLPF mode"
        );
        ESP_LOGI(TAG, "MPU6050 initialized");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // LoRa initialization
    {
        lora_dev.tx_pin = LORA_RX;
        lora_dev.rx_pin = LORA_TX;
        lora_dev.m0_pin = LORA_M0;
        lora_dev.m1_pin = LORA_M1;
        lora_dev.aux_pin = LORA_AUX;
        lora_dev.uart_num = LORA_UART;
        lora_dev.baud_rate = LORA_BAUD_RATE;
        AVIONICS_ERROR_CHECK(
            lora_init(&lora_dev),
            ABORT_LORA_INIT,
            "LoRa failed to init"
        );
        // lora_set_power(&lora_dev, LORA_POWER_22_DBM);
        // lora_set_power(&lora_dev, LORA_POWER_17_DBM);
        lora_set_power(&lora_dev, LORA_POWER_13_DBM);
        lora_set_rssi(&lora_dev, false);
        lora_set_address(&lora_dev, TMTC_ADDRESS);
        lora_set_channel(&lora_dev, TMTC_CHANNEL);
        lora_set_air_data_rate(&lora_dev, TMTC_AIR_DATA_RATE);

        AVIONICS_ERROR_CHECK(
            uart_flush(lora_dev.uart_num),
            ABORT_LORA_INIT,
            "LoRa uart failed to flush"
        );
        ESP_LOGI(TAG, "LoRa initialized");
    }

    // GPS initialization
    {
        AVIONICS_ERROR_CHECK(
            gps_init_desc(&gps_dev, GPS_TX, GPS_RX, GPS_UART),
            ABORT_GPS_INIT,
            "GPS failed to init"
        );
        ESP_LOGI(TAG, "GPS initialized");
    }

    // create xQueue
    {
        flash_queue = xQueueCreate(32, sizeof(flash_payload_t));
        lora_queue = xQueueCreate(1, sizeof(lora_payload_t)); // mailbox
        telecommand_queue = xQueueCreate(8, sizeof(telecommand_payload_t));
    }

    // create tasks
    {
        xTaskCreatePinnedToCore(avionics_task, "avionics", 4096, NULL, 10, NULL, 1); // APP_CPU
        xTaskCreatePinnedToCore(flash_task, "flash", 4096, NULL, 5, NULL, 0); // PRO_CPU
        xTaskCreatePinnedToCore(lora_task, "lora", 4096, NULL, 5, NULL, 0); // PRO_CPU
    }
}