/* main.c - Sistema IoT Detección de Deslizamientos (ESP-IDF, Dashboard Web)
   VERSIÓN FINAL - ISR + Timer de Reset + Umbrales específicos lluvia+humedad+vibración
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/ip4_addr.h"

// ================= DEFINICIONES =================
static const char *TAG = "LANDSLIDE_MONITOR";

/* Pines y configuración - ISR de vibración */
#define LED_VERDE         GPIO_NUM_2
#define LED_AMARILLO      GPIO_NUM_4
#define LED_ROJO          GPIO_NUM_5

/* Pin ISR para vibración */
#define SW420_OUT_PIN     GPIO_NUM_32  // ISR Digital

/* Pines ADC - Solo 2 sensores analógicos */
#define HW103_ADC_PIN     GPIO_NUM_33  // ADC1_CH5
#define YL83_ADC_PIN      GPIO_NUM_35  // ADC1_CH7

#define HW103_ADC_CHANNEL ADC_CHANNEL_5
#define YL83_ADC_CHANNEL  ADC_CHANNEL_7

#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000
#define MPU6050_ADDR         0x68

/* Configuración Wi-Fi Access Point */
#define WIFI_SSID            "WLAN_Alcaldia_Monitoreo"
#define WIFI_PASS            "MonitoreoTaludes2025"
#define WIFI_CHANNEL         1
#define MAX_STA_CONN         5
#define WEBSERVER_PORT       80

/* Umbrales específicos ajustados */
#define INCLINATION_WARNING  15.0f
#define INCLINATION_CRITICAL 25.0f
#define VIBRATION_COUNT_ALERT 5      // Alerta amarilla
#define VIBRATION_COUNT_HIGH 50      // Muchas vibraciones
#define VIBRATION_COUNT_RAIN_CRITICAL 100  // Crítico con lluvia+humedad
#define VIBRATION_COUNT_CRITICAL 200 // Vibraciones críticas absolutas
#define VIBRATION_RESET_TIMEOUT_SEC 30    // 30 segundos sin vibración para reset
#define NO_OF_SAMPLES        16
#define REGULAR_MEASUREMENT_INTERVAL_MS 5000
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<LED_VERDE) | (1ULL<<LED_AMARILLO) | (1ULL<<LED_ROJO))

// ================= ESTRUCTURAS =================
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    float pitch, roll;
} mpu6050_data_t;

typedef struct {
    uint32_t hw103_value, yl83_value, sw420_value;
    bool rain_detected, soil_dry, vibration_detected;
    int rain_level, soil_moisture_level, vibration_level;
    uint32_t vibration_count;
    uint32_t vibration_intensity;
} sensors_data_t;

typedef struct {
    mpu6050_data_t mpu_data;
    sensors_data_t sensor_data;
    int alert_level; // 0=OK, 1=ALERTA, 2=CRITICO
} system_status_t;

// ================= GLOBALES =================
static system_status_t system_status;
static httpd_handle_t server = NULL;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc1_calibrated = false;

// Variables para ISR de vibración con timer de reset
static volatile bool vibration_event = false;
static volatile uint32_t vibration_count = 0;
static volatile uint64_t last_vibration_time = 0;
static esp_timer_handle_t vibration_reset_timer;

// ================= TIMER DE RESET DE VIBRACIONES =================

/**
 * @brief Callback del timer que resetea el contador de vibraciones
 * @details Se ejecuta 30 segundos después de la última vibración
 */
static void IRAM_ATTR vibration_reset_timer_callback(void* arg) {
    // Solo resetear si realmente han pasado 30 segundos sin vibraciones
    uint64_t current_time = esp_timer_get_time();
    uint64_t time_since_last = (current_time - last_vibration_time) / 1000000; // segundos
    
    if (time_since_last >= VIBRATION_RESET_TIMEOUT_SEC) {
        if (vibration_count > 0) {
            ESP_LOGI(TAG, "RESET AUTOMÁTICO: Contador de vibraciones reseteado tras %lld segundos de inactividad", time_since_last);
        }
        vibration_count = 0;
    }
}

/**
 * @brief Inicializa el timer de reset de vibraciones
 */
static esp_err_t init_vibration_reset_timer(void) {
    const esp_timer_create_args_t timer_args = {
        .callback = &vibration_reset_timer_callback,
        .arg = NULL,
        .name = "vibration_reset"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &vibration_reset_timer));
    ESP_LOGI(TAG, "Timer de reset de vibraciones configurado (30s inactividad)");
    return ESP_OK;
}

/**
 * @brief Reinicia el timer de reset cuando hay nueva vibración
 */
static void restart_vibration_reset_timer(void) {
    // Detener timer actual si está corriendo
    esp_timer_stop(vibration_reset_timer);
    
    // Iniciar nuevo timer de 30 segundos
    esp_timer_start_once(vibration_reset_timer, VIBRATION_RESET_TIMEOUT_SEC * 1000000); // microsegundos
}

// ================= ISR DE VIBRACIÓN =================

/**
 * @brief ISR disparado por detección física de vibración
 */
static void IRAM_ATTR vibration_isr_handler(void* arg) {
    vibration_event = true;
    vibration_count++;
    last_vibration_time = esp_timer_get_time();
    
    // Nota: No podemos llamar funciones complejas desde ISR,
    // el restart del timer se hará en el main loop
}

/**
 * @brief Inicialización de GPIO para ISR de vibración
 */
static esp_err_t init_vibration_isr(void) {
    ESP_LOGI(TAG, "Configurando ISR de vibración en GPIO%d...", SW420_OUT_PIN);
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SW420_OUT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SW420_OUT_PIN, vibration_isr_handler, NULL));
    
    ESP_LOGI(TAG, "ISR de vibración configurado correctamente");
    return ESP_OK;
}

// ================= FUNCIONES I2C/MPU6050 =================
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_init(void) {
    uint8_t who_am_i = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x75, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &who_am_i, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK || who_am_i != 0x68) {
        ESP_LOGE(TAG, "MPU6050 WHO_AM_I error: 0x%02X", who_am_i);
        return ESP_FAIL;
    }
    
    // Wake up MPU6050
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "MPU6050 inicializado correctamente");
    return ESP_OK;
}

static esp_err_t mpu6050_read_data(mpu6050_data_t *data) {
    uint8_t raw_data[6] = {0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, raw_data + 5, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
        data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
        data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

        float ax_g = (float)data->accel_x / 16384.0f;
        float ay_g = (float)data->accel_y / 16384.0f;
        float az_g = (float)data->accel_z / 16384.0f;

        data->pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;
        data->roll = atan2f(ay_g, az_g) * 180.0f / M_PI;
    }
    return ret;
}

// ================= FUNCIONES ADC =================
static esp_err_t init_adc_sensors(void) {
    ESP_LOGI(TAG, "Inicializando ADC1 para sensores analógicos (HW103 y YL83)...");
    
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
    adc_oneshot_chan_cfg_t cfg1 = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, HW103_ADC_CHANNEL, &cfg1));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, YL83_ADC_CHANNEL, &cfg1));
    
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle) == ESP_OK) {
        adc1_calibrated = true;
        ESP_LOGI(TAG, "Calibración ADC1 habilitada");
    }
    
    ESP_LOGI(TAG, "ADC1 inicializado para 2 sensores analógicos");
    return ESP_OK;
}

static esp_err_t read_adc_sensors(sensors_data_t *sd) {
    int hw103_raw = 0, yl83_raw = 0;
    
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        int temp_hw103 = 0, temp_yl83 = 0;
        
        adc_oneshot_read(adc1_handle, HW103_ADC_CHANNEL, &temp_hw103);
        adc_oneshot_read(adc1_handle, YL83_ADC_CHANNEL, &temp_yl83);
        
        hw103_raw += temp_hw103;
        yl83_raw += temp_yl83;
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    hw103_raw /= NO_OF_SAMPLES;
    yl83_raw /= NO_OF_SAMPLES;
    
    if (adc1_calibrated) {
        int mv_hw103 = 0, mv_yl83 = 0;
        adc_cali_raw_to_voltage(adc1_cali_handle, hw103_raw, &mv_hw103);
        adc_cali_raw_to_voltage(adc1_cali_handle, yl83_raw, &mv_yl83);
        
        sd->hw103_value = (uint32_t)mv_hw103;
        sd->yl83_value = (uint32_t)mv_yl83;
    } else {
        sd->hw103_value = (uint32_t)hw103_raw;
        sd->yl83_value = (uint32_t)yl83_raw;
    }
    
    // SW-420: Obtener datos del ISR
    sd->vibration_detected = vibration_event;
    sd->vibration_count = vibration_count;
    sd->sw420_value = vibration_event ? 1 : 0;
    
    if (vibration_event) {
        vibration_event = false;
    }
    
    // Procesar lógica de sensores
    if (adc1_calibrated) {
        if (sd->hw103_value <= 1000) {
            sd->soil_dry = false;
            sd->soil_moisture_level = 2;
        } else if (sd->hw103_value >= 2500) {
            sd->soil_dry = true;
            sd->soil_moisture_level = 0;
        } else {
            sd->soil_dry = false;
            sd->soil_moisture_level = 1;
        }
    } else {
        if (sd->hw103_value <= 1500) {
            sd->soil_dry = false;
            sd->soil_moisture_level = 2;
        } else if (sd->hw103_value >= 3000) {
            sd->soil_dry = true;
            sd->soil_moisture_level = 0;
        } else {
            sd->soil_dry = false;
            sd->soil_moisture_level = 1;
        }
    }
    
    if (adc1_calibrated) {
        if (sd->yl83_value <= 1000) {
            sd->rain_detected = true;
            sd->rain_level = 2;
        } else if (sd->yl83_value <= 2000) {
            sd->rain_detected = true;
            sd->rain_level = 1;
        } else {
            sd->rain_detected = false;
            sd->rain_level = 0;
        }
    } else {
        if (sd->yl83_value <= 1500) {
            sd->rain_detected = true;
            sd->rain_level = 2;
        } else if (sd->yl83_value <= 3000) {
            sd->rain_detected = true;
            sd->rain_level = 1;
        } else {
            sd->rain_detected = false;
            sd->rain_level = 0;
        }
    }
    
    sd->vibration_intensity = sd->vibration_count / 10;
    sd->vibration_level = (sd->vibration_count >= VIBRATION_COUNT_ALERT) ? 2 : 
                         (sd->vibration_count >= 2) ? 1 : 0;
    
    return ESP_OK;
}

// ================= ANÁLISIS DE RIESGOS CON UMBRALES ESPECÍFICOS =================
static void analyze_system_status(system_status_t *st) {
    float max_inclination = fmaxf(fabsf(st->mpu_data.pitch), fabsf(st->mpu_data.roll));
    st->alert_level = 0;
    
    // Variables para lógica avanzada
    bool rain_and_soil_wet = (st->sensor_data.rain_detected && st->sensor_data.soil_moisture_level >= 2);
    bool high_inclination = (max_inclination >= INCLINATION_CRITICAL);
    bool warning_inclination = (max_inclination >= INCLINATION_WARNING);
    bool many_vibrations = (st->sensor_data.vibration_count >= VIBRATION_COUNT_HIGH);
    bool critical_vibrations = (st->sensor_data.vibration_count >= VIBRATION_COUNT_CRITICAL);
    bool rain_critical_vibrations = (st->sensor_data.vibration_count >= VIBRATION_COUNT_RAIN_CRITICAL);
    
    ESP_LOGI(TAG, "ANÁLISIS: Inclin=%.1f° | Lluvia=%s | SueloHúmedo=%s | Vibr=%"PRIu32,
             max_inclination,
             st->sensor_data.rain_detected ? "SÍ" : "NO",
             st->sensor_data.soil_moisture_level >= 2 ? "SÍ" : "NO",
             st->sensor_data.vibration_count);
    
    // ===== CASOS CRÍTICOS (LED ROJO) =====
    
    // Caso 1: Inclinación crítica + vibraciones altas
    if (high_inclination && many_vibrations) {
        st->alert_level = 2;
        ESP_LOGW(TAG, "CRÍTICO: Inclinación alta (%.1f°) + muchas vibraciones (%"PRIu32")",
                 max_inclination, st->sensor_data.vibration_count);
        return;
    }
    
    // Caso 2: Vibraciones extremas (200+)
    if (critical_vibrations) {
        st->alert_level = 2;
        ESP_LOGW(TAG, "CRÍTICO: Vibraciones extremas detectadas (%"PRIu32" eventos)",
                 st->sensor_data.vibration_count);
        return;
    }
    
    // Caso 3: NUEVO - Lluvia + suelo húmedo + 100 vibraciones (umbral específico)
    if (rain_and_soil_wet && rain_critical_vibrations) {
        st->alert_level = 2;
        ESP_LOGW(TAG, "CRÍTICO: Lluvia + suelo saturado + vibraciones críticas (%"PRIu32" eventos >= 100)",
                 st->sensor_data.vibration_count);
        return;
    }
    
    // Caso 4: Lluvia + suelo húmedo + factor adicional no vibratorio
    if (rain_and_soil_wet && high_inclination) {
        st->alert_level = 2;
        ESP_LOGW(TAG, "CRÍTICO: Lluvia + suelo saturado + inclinación crítica (%.1f°)",
                 max_inclination);
        return;
    }
    
    // Caso 5: Inclinación crítica extrema (35° o más)
    if (max_inclination >= (INCLINATION_CRITICAL + 10.0f)) {
        st->alert_level = 2;
        ESP_LOGW(TAG, "CRÍTICO: Inclinación extrema (%.1f°)", max_inclination);
        return;
    }
    
    // ===== CASOS DE ALERTA (LED AMARILLO) =====
    
    // Caso 1: Lluvia + suelo húmedo (condición natural esperada) - INCLUSO CON <100 VIBRACIONES
    if (rain_and_soil_wet) {
        st->alert_level = 1;
        if (st->sensor_data.vibration_count > 0) {
            ESP_LOGI(TAG, "ALERTA: Lluvia + suelo húmedo + vibraciones moderadas (%"PRIu32" eventos < 100)",
                     st->sensor_data.vibration_count);
        } else {
            ESP_LOGI(TAG, "ALERTA: Lluvia + suelo húmedo (condición meteorológica)");
        }
        return;
    }
    
    // Caso 2: Inclinación de advertencia
    if (warning_inclination) {
        st->alert_level = 1;
        ESP_LOGI(TAG, "ALERTA: Inclinación en rango de advertencia (%.1f°)", max_inclination);
        return;
    }
    
    // Caso 3: Muchas vibraciones (pero no críticas)
    if (st->sensor_data.vibration_count >= VIBRATION_COUNT_HIGH && 
        st->sensor_data.vibration_count < VIBRATION_COUNT_CRITICAL) {
        st->alert_level = 1;
        ESP_LOGI(TAG, "ALERTA: Vibraciones frecuentes (%"PRIu32" eventos)", 
                 st->sensor_data.vibration_count);
        return;
    }
    
    // Caso 4: Lluvia intensa sola
    if (st->sensor_data.rain_level >= 2) {
        st->alert_level = 1;
        ESP_LOGI(TAG, "ALERTA: Lluvia intensa detectada");
        return;
    }
    
    // Caso 5: Suelo muy húmedo sin lluvia (posible filtración)
    if (st->sensor_data.soil_moisture_level >= 2 && !st->sensor_data.rain_detected) {
        st->alert_level = 1;
        ESP_LOGI(TAG, "ALERTA: Suelo muy húmedo sin lluvia (posible filtración)");
        return;
    }
    
    // Caso 6: Vibraciones moderadas
    if (st->sensor_data.vibration_count >= VIBRATION_COUNT_ALERT && 
        st->sensor_data.vibration_count < VIBRATION_COUNT_HIGH) {
        st->alert_level = 1;
        ESP_LOGI(TAG, "ALERTA: Vibraciones moderadas (%"PRIu32" eventos)", 
                 st->sensor_data.vibration_count);
        return;
    }
    
    // ===== ESTADO NORMAL (LED VERDE) =====
    st->alert_level = 0;
    ESP_LOGD(TAG, "NORMAL: Todas las condiciones dentro de parámetros seguros");
}

// ================= FUNCIÓN DEBUG DETALLADO =================
static void log_detailed_status(system_status_t *st) {
    ESP_LOGI(TAG, "=== ESTADO DETALLADO DEL SISTEMA ===");
    ESP_LOGI(TAG, "Inclinación: Pitch=%.2f° | Roll=%.2f° | Máx=%.2f°",
             st->mpu_data.pitch, st->mpu_data.roll,
             fmaxf(fabsf(st->mpu_data.pitch), fabsf(st->mpu_data.roll)));
    
    ESP_LOGI(TAG, "Lluvia: %s (Nivel: %d | Valor: %"PRIu32")",
             st->sensor_data.rain_detected ? "DETECTADA" : "NO",
             st->sensor_data.rain_level,
             st->sensor_data.yl83_value);
    
    ESP_LOGI(TAG, "Suelo: %s (Nivel: %d | Valor: %"PRIu32")",
             st->sensor_data.soil_dry ? "SECO" : "HÚMEDO",
             st->sensor_data.soil_moisture_level,
             st->sensor_data.hw103_value);
    
    ESP_LOGI(TAG, "Vibración: %s (Eventos: %"PRIu32" | Valor digital: %"PRIu32")",
             st->sensor_data.vibration_detected ? "ACTIVA" : "INACTIVA",
             st->sensor_data.vibration_count,
             st->sensor_data.sw420_value);
    
    // Información sobre umbrales específicos
    ESP_LOGI(TAG, "Umbrales: 5+ Moderada | 50+ Frecuente | 100+ Crítica(Lluvia+Húmedo) | 200+ Crítica(Absoluta)");
    
    const char* estado_str = (st->alert_level == 2) ? "CRÍTICO (ROJO)" :
                            (st->alert_level == 1) ? "ALERTA (AMARILLO)" : "NORMAL (VERDE)";
    ESP_LOGI(TAG, "ESTADO FINAL: %s", estado_str);
    ESP_LOGI(TAG, "=====================================");
}

// ================= LEDs =================
static void init_leds(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    
    gpio_set_level(LED_VERDE, 0);
    gpio_set_level(LED_AMARILLO, 0);
    gpio_set_level(LED_ROJO, 0);
}

static void update_leds(system_status_t *st) {
    static int prev_alert_level = -1;
    
    if (st->alert_level != prev_alert_level) {
        switch (st->alert_level) {
            case 2: // CRÍTICO
                gpio_set_level(LED_VERDE, 0);
                gpio_set_level(LED_AMARILLO, 0);
                gpio_set_level(LED_ROJO, 1);
                break;
            case 1: // ALERTA
                gpio_set_level(LED_VERDE, 0);
                gpio_set_level(LED_AMARILLO, 1);
                gpio_set_level(LED_ROJO, 0);
                break;
            default: // NORMAL
                gpio_set_level(LED_VERDE, 1);
                gpio_set_level(LED_AMARILLO, 0);
                gpio_set_level(LED_ROJO, 0);
                break;
        }
        prev_alert_level = st->alert_level;
    }
}

// ================= WI-FI Y SERVIDOR WEB =================
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Dispositivo conectado al AP");
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "Dispositivo desconectado del AP");
    }
}

static esp_err_t init_wifi_ap(void) {
    ESP_LOGI(TAG, "Inicializando Wi-Fi Access Point...");
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t *netif_ap = esp_netif_create_default_wifi_ap();
    
    esp_netif_ip_info_t ip_info;
    esp_netif_dhcps_stop(netif_ap);
    IP4_ADDR(&ip_info.ip, 192, 168, 1, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif_ap, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(netif_ap));
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, 
                                               &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "Wi-Fi AP iniciado: %s", WIFI_SSID);
    return ESP_OK;
}

static esp_err_t dashboard_handler(httpd_req_t *req) {
    const char* dashboard_html = 
        "<!DOCTYPE html>"
        "<html><head><title>Sistema Monitoreo - Deslizamientos</title>"
        "<meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial;margin:20px;background:#f0f0f0;}"
        ".container{max-width:1200px;margin:0 auto;}"
        ".header{background:#2c3e50;color:white;padding:20px;border-radius:8px;text-align:center;}"
        ".status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:20px;margin:20px 0;}"
        ".sensor-card{background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);}"
        ".sensor-title{font-weight:bold;margin-bottom:10px;color:#2c3e50;}"
        ".sensor-value{font-size:24px;margin:10px 0;}"
        ".status-normal{color:#27ae60;}.status-warning{color:#f39c12;}.status-critical{color:#e74c3c;}"
        ".alert-banner{padding:15px;margin:20px 0;border-radius:8px;text-align:center;font-weight:bold;}"
        ".alert-normal{background:#d5f4e6;color:#27ae60;border:2px solid #27ae60;}"
        ".alert-warning{background:#fef9e7;color:#f39c12;border:2px solid #f39c12;}"
        ".alert-critical{background:#fadbd8;color:#e74c3c;border:2px solid #e74c3c;}"
        ".refresh-info{text-align:center;margin:20px;color:#7f8c8d;}"
        ".isr-indicator{background:#3498db;color:white;padding:5px 10px;border-radius:4px;font-size:12px;}"
        ".vibration-levels{font-size:12px;color:#7f8c8d;margin-top:5px;}"
        ".timer-info{font-size:11px;color:#8e44ad;margin-top:3px;}"
        "</style></head><body>"
        "<div class='container'>"
        "<div class='header'>"
        "<h1>Sistema IoT - Detección de Deslizamientos</h1>"
        "<p>Región Sabana Centro - Cundinamarca</p>"
        "<p class='isr-indicator'>Sistema ISR + Timer Reset (30s inactividad)</p>"
        "</div>"
        "<div id='alert-status' class='alert-banner alert-normal'>SISTEMA NORMAL</div>"
        "<div class='status-grid'>"
        "<div class='sensor-card'>"
        "<div class='sensor-title'>Inclinación del Talud</div>"
        "<div id='pitch' class='sensor-value status-normal'>-- °</div>"
        "<div>Pitch: <span id='pitch-val'>--</span>° | Roll: <span id='roll-val'>--</span>°</div>"
        "</div>"
        "<div class='sensor-card'>"
        "<div class='sensor-title'>Humedad del Suelo</div>"
        "<div id='soil' class='sensor-value status-normal'>--</div>"
        "<div>Valor: <span id='soil-val'>--</span></div>"
        "</div>"
        "<div class='sensor-card'>"
        "<div class='sensor-title'>Detección de Lluvia</div>"
        "<div id='rain' class='sensor-value status-normal'>--</div>"
        "<div>Valor: <span id='rain-val'>--</span></div>"
        "</div>"
        "<div class='sensor-card'>"
        "<div class='sensor-title'>Vibración (ISR + Timer)</div>"
        "<div id='vibration' class='sensor-value status-normal'>--</div>"
        "<div>Estado: <span id='vib-val'>--</span> | Eventos: <span id='vib-count'>--</span></div>"
        "<div class='vibration-levels'>5+ Moderada | 50+ Frecuente | 100+ Crítica(Lluvia) | 200+ Crítica</div>"
        "<div class='timer-info'>Auto-reset: 30s sin actividad</div>"
        "</div>"
        "</div>"
        "<div class='refresh-info'>"
        "<p>Actualizaciones automáticas cada 5 segundos</p>"
        "<p>Última actualización: <span id='last-update'>--</span></p>"
        "</div>"
        "</div>"
        "<script>"
        "function updateData(){"
        "fetch('/api/sensores/estado')"
        ".then(response=>response.json())"
        ".then(data=>{"
        "document.getElementById('pitch-val').textContent=data.pitch.toFixed(1);"
        "document.getElementById('roll-val').textContent=data.roll.toFixed(1);"
        "document.getElementById('soil-val').textContent=data.soil_value;"
        "document.getElementById('rain-val').textContent=data.rain_value;"
        "document.getElementById('vib-val').textContent=data.vibration_value?'DETECTADA':'NORMAL';"
        "document.getElementById('vib-count').textContent=data.vibration_count;"
        "document.getElementById('pitch').textContent=Math.max(Math.abs(data.pitch),Math.abs(data.roll)).toFixed(1)+'°';"
        "document.getElementById('soil').textContent=data.soil_dry?'SECO':'HÚMEDO';"
        "document.getElementById('rain').textContent=data.rain_detected?'SÍ':'NO';"
        "document.getElementById('vibration').textContent=data.vibration_detected?'ACTIVA':'INACTIVA';"
        "const alertDiv=document.getElementById('alert-status');"
        "if(data.alert_level===2){"
        "alertDiv.className='alert-banner alert-critical';"
        "alertDiv.textContent='ESTADO CRÍTICO - RIESGO INMINENTE';"
        "}else if(data.alert_level===1){"
        "alertDiv.className='alert-banner alert-warning';"
        "alertDiv.textContent='ALERTA - CONDICIONES DE RIESGO';"
        "}else{"
        "alertDiv.className='alert-banner alert-normal';"
        "alertDiv.textContent='SISTEMA NORMAL';"
        "}"
        "document.getElementById('last-update').textContent=new Date().toLocaleString();"
        "})"
        ".catch(error=>console.error('Error:',error));"
        "}"
        "updateData();"
        "setInterval(updateData,5000);"
        "</script>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, dashboard_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_sensores_handler(httpd_req_t *req) {
    char json_response[512];
    
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"pitch\":%.2f,"
        "\"roll\":%.2f,"
        "\"soil_value\":%"PRIu32","
        "\"rain_value\":%"PRIu32","
        "\"vibration_value\":%"PRIu32","
        "\"vibration_count\":%"PRIu32","
        "\"soil_dry\":%s,"
        "\"rain_detected\":%s,"
        "\"vibration_detected\":%s,"
        "\"alert_level\":%d,"
        "\"isr_enabled\":true,"
        "\"timer_reset_enabled\":true,"
        "\"vibration_thresholds\":{\"moderate\":5,\"frequent\":50,\"rain_critical\":100,\"absolute_critical\":200},"
        "\"logic_version\":\"timer_enhanced\""
        "}",
        system_status.mpu_data.pitch,
        system_status.mpu_data.roll,
        system_status.sensor_data.hw103_value,
        system_status.sensor_data.yl83_value,
        system_status.sensor_data.sw420_value,
        system_status.sensor_data.vibration_count,
        system_status.sensor_data.soil_dry ? "true" : "false",
        system_status.sensor_data.rain_detected ? "true" : "false",
        system_status.sensor_data.vibration_detected ? "true" : "false",
        system_status.alert_level
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEBSERVER_PORT;
    config.max_open_sockets = 7;
    config.task_priority = 5;
    
    httpd_uri_t dashboard_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = dashboard_handler,
        .user_ctx = NULL
    };
    
    httpd_uri_t api_sensores_uri = {
        .uri = "/api/sensores/estado",
        .method = HTTP_GET,
        .handler = api_sensores_handler,
        .user_ctx = NULL
    };
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &dashboard_uri);
        httpd_register_uri_handler(server, &api_sensores_uri);
        ESP_LOGI(TAG, "Servidor HTTP iniciado en http://192.168.1.1/");
        return ESP_OK;
    }
    return ESP_FAIL;
}

// ================= TAREA PRINCIPAL CON ISR Y TIMER DE RESET =================
static void monitoring_task(void *arg) {
    ESP_LOGI(TAG, "Iniciando tarea de monitoreo con ISR y timer de reset automático...");
    
    TickType_t last_regular_measurement = xTaskGetTickCount();
    const TickType_t regular_interval = pdMS_TO_TICKS(REGULAR_MEASUREMENT_INTERVAL_MS);
    
    while (1) {
        bool do_measurement = false;
        bool vibration_triggered = false;
        
        // Check 1: Vibración detectada por ISR
        if (vibration_event) {
            do_measurement = true;
            vibration_triggered = true;
            
            // IMPORTANTE: Reiniciar timer de reset cuando hay nueva vibración
            restart_vibration_reset_timer();
            
            ESP_LOGW(TAG, "¡EVENTO SÍSMICO! Vibración detectada por ISR - Timer reset reiniciado");
        }
        
        // Check 2: Medición regular cada 5 segundos
        if ((xTaskGetTickCount() - last_regular_measurement) >= regular_interval) {
            do_measurement = true;
            last_regular_measurement = xTaskGetTickCount();
            
            if (!vibration_triggered) {
                ESP_LOGI(TAG, "Medición regular programada");
            }
        }
        
        // Ejecutar mediciones si es necesario
        if (do_measurement) {
            esp_err_t mpu_ok = mpu6050_read_data(&system_status.mpu_data);
            esp_err_t sensors_ok = read_adc_sensors(&system_status.sensor_data);

            if (mpu_ok == ESP_OK && sensors_ok == ESP_OK) {
                analyze_system_status(&system_status);
                update_leds(&system_status);

                // Log detallado para eventos de vibración
                if (vibration_triggered) {
                    log_detailed_status(&system_status);
                }

                // Log regular cada 10 ciclos normales
                static int log_counter = 0;
                if (!vibration_triggered && (++log_counter >= 10)) {
                    ESP_LOGI(TAG, "Estado: %s | Inclinación: %.1f° | Vibraciones totales: %"PRIu32, 
                             (system_status.alert_level == 2) ? "CRÍTICO" : 
                             (system_status.alert_level == 1) ? "ALERTA" : "NORMAL",
                             fmaxf(fabsf(system_status.mpu_data.pitch), fabsf(system_status.mpu_data.roll)),
                             system_status.sensor_data.vibration_count);
                             
                    ESP_LOGI(TAG, "Sensores: HW103=%"PRIu32"%s(Suelo:%s) | YL83=%"PRIu32"%s(Lluvia:%s) | SW420=ISR(Eventos:%"PRIu32")",
                             system_status.sensor_data.hw103_value, adc1_calibrated ? "mV" : "raw",
                             system_status.sensor_data.soil_dry ? "SECO" : "HÚMEDO",
                             system_status.sensor_data.yl83_value, adc1_calibrated ? "mV" : "raw",
                             system_status.sensor_data.rain_detected ? "LLUVIA" : "SECO",
                             system_status.sensor_data.vibration_count);
                             
                    // Información adicional sobre timer de reset
                    if (system_status.sensor_data.vibration_count > 0) {
                        uint64_t current_time = esp_timer_get_time();
                        uint64_t time_since_last = (current_time - last_vibration_time) / 1000000;
                        ESP_LOGI(TAG, "Timer info: %lld segundos desde última vibración (reset en %d s)", 
                                 time_since_last, VIBRATION_RESET_TIMEOUT_SEC - (int)time_since_last);
                    }
                    log_counter = 0;
                }

                // Ya no necesitamos reset manual cada minuto porque el timer lo maneja automáticamente
                
            } else {
                ESP_LOGW(TAG, "Error sensores: MPU=%s, ADC=%s", 
                         mpu_ok == ESP_OK ? "OK" : "ERROR",
                         sensors_ok == ESP_OK ? "OK" : "ERROR");
                
                // LED rojo intermitente para error
                gpio_set_level(LED_ROJO, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(LED_ROJO, 0);
            }
        }

        // Sleep más corto para mejor responsividad a ISR
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ================= FUNCIÓN PRINCIPAL =================
void app_main(void)
{
    ESP_LOGI(TAG, "=== Sistema IoT con ISR + Timer de Reset Automático ===");
    ESP_LOGI(TAG, "Configuración de hardware:");
    ESP_LOGI(TAG, "  SW-420:  GPIO%d (ISR Digital) - Detección por interrupción", SW420_OUT_PIN);
    ESP_LOGI(TAG, "  HW-103:  GPIO%d (ADC1_CH%d) - Humedad suelo", HW103_ADC_PIN, HW103_ADC_CHANNEL);
    ESP_LOGI(TAG, "  YL-83:   GPIO%d (ADC1_CH%d) - Lluvia", YL83_ADC_PIN, YL83_ADC_CHANNEL);
    ESP_LOGI(TAG, "  MPU6050: I2C 0x68 - Inclinación");
    ESP_LOGI(TAG, "CARACTERÍSTICAS AVANZADAS:");
    ESP_LOGI(TAG, "  * Timer de reset automático: 30s sin vibraciones");
    ESP_LOGI(TAG, "  * Umbral específico lluvia+húmedo: 100 vibraciones para crítico");
    ESP_LOGI(TAG, "  * Umbrales generales: 5+ Moderada | 50+ Frecuente | 200+ Crítica absoluta");

    memset(&system_status, 0, sizeof(system_status_t));
    
    // Inicializar NVS (requerido para Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    init_leds();

    // Prueba secuencial de LEDs
    ESP_LOGI(TAG, "Probando LEDs...");
    for (int i = 0; i < 2; i++) {
        gpio_set_level(LED_VERDE, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_VERDE, 0);
        gpio_set_level(LED_AMARILLO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_AMARILLO, 0);
        gpio_set_level(LED_ROJO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_ROJO, 0);
    }

    // Inicialización I2C y MPU6050
    if (i2c_master_init() == ESP_OK) {
        ESP_LOGI(TAG, "I2C inicializado");
        
        if (mpu6050_init() == ESP_OK) {
            ESP_LOGI(TAG, "MPU6050 inicializado correctamente");
        } else {
            ESP_LOGE(TAG, "Error inicializando MPU6050");
        }
    } else {
        ESP_LOGE(TAG, "Error crítico inicializando I2C");
        while(1) {
            gpio_set_level(LED_ROJO, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_ROJO, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // Inicialización ADC
    if (init_adc_sensors() == ESP_OK) {
        ESP_LOGI(TAG, "Módulos ADC inicializados correctamente");
    } else {
        ESP_LOGE(TAG, "Error inicializando módulos ADC");
    }

    // Inicializar ISR de vibración
    if (init_vibration_isr() == ESP_OK) {
        ESP_LOGI(TAG, "ISR de vibración configurado correctamente");
    } else {
        ESP_LOGE(TAG, "Error crítico: No se pudo configurar ISR de vibración");
        while(1) {
            gpio_set_level(LED_ROJO, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_ROJO, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    // NUEVO: Inicializar timer de reset automático
    if (init_vibration_reset_timer() == ESP_OK) {
        ESP_LOGI(TAG, "Timer de reset automático configurado correctamente");
    } else {
        ESP_LOGE(TAG, "Error crítico: No se pudo configurar timer de reset");
        while(1) {
            gpio_set_level(LED_ROJO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_ROJO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Inicializar Wi-Fi Access Point
    if (init_wifi_ap() == ESP_OK) {
        ESP_LOGI(TAG, "Wi-Fi Access Point inicializado");
        
        // Inicializar servidor web
        if (start_webserver() == ESP_OK) {
            ESP_LOGI(TAG, "Servidor web iniciado correctamente");
        } else {
            ESP_LOGE(TAG, "Error iniciando servidor web");
        }
    } else {
        ESP_LOGE(TAG, "Error inicializando Wi-Fi Access Point");
    }

    ESP_LOGI(TAG, "Creando tarea de monitoreo con ISR y timer...");
    xTaskCreate(monitoring_task, "monitoring", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "=== Sistema Completo Iniciado ===");
    ESP_LOGI(TAG, "Wi-Fi AP: %s", WIFI_SSID);
    ESP_LOGI(TAG, "Dashboard web: http://192.168.1.1/");
    ESP_LOGI(TAG, "FUNCIONALIDADES TÉCNICAS:");
    ESP_LOGI(TAG, "  ✓ ISR de vibración con respuesta inmediata");
    ESP_LOGI(TAG, "  ✓ Timer automático: reset tras 30s sin actividad");
    ESP_LOGI(TAG, "  ✓ Lógica específica: Lluvia+Húmedo+100vibr = Crítico");
    ESP_LOGI(TAG, "  ✓ Umbrales inteligentes: 5|50|100|200 eventos");
    ESP_LOGI(TAG, "PRUEBA: Golpea el SW-420 y observa el comportamiento del timer");
    ESP_LOGI(TAG, "Sistema listo para monitoreo avanzado con gestión automática de eventos");
}