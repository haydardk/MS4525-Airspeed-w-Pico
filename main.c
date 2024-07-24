#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "math.h"

#define PRESSURE_OFFSET 115
#define I2C_PORT i2c0
#define MS4525D0_I2C_ADDR1 0x28

// Fonksiyon Prototipleri
void MS4525_StartMeasurement();
void MS4525_ReadData(uint8_t* data);
void MS4525_Calculate();
float calculate_airspeed(float pressure);

// Global Değişkenler
uint8_t dataPitot[4];
float MS4525_Temperature;
float MS4525_Pressure;
float MS4525_Airspeed;

int main() {
    // Pico'da seri iletişim başlatma
    stdio_init_all();

    // I2C yapılandırma
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    while (1) {
        // Ölçüm başlatma
        MS4525_StartMeasurement();
        // Verileri okuma
        sleep_ms(50);
        MS4525_ReadData(dataPitot);
        // Basınç ve sıcaklık hesaplama
        MS4525_Calculate();

        printf("Temperature: %.2f C, Pressure: %.2f Pa, Airspeed: %.2f m/s\n", MS4525_Temperature, MS4525_Pressure, MS4525_Airspeed);
        sleep_ms(50);
    }
}

void MS4525_StartMeasurement() {
    uint8_t cmd = 0; // Ölçüm başlatma komutu
    int result = i2c_write_blocking(I2C_PORT, MS4525D0_I2C_ADDR1, &cmd, 1, false);
    if (result == PICO_ERROR_GENERIC) {
        printf("I2C transmission error\n");
    }
}

void MS4525_ReadData(uint8_t* data) {
    int result = i2c_read_blocking(I2C_PORT, MS4525D0_I2C_ADDR1, data, 4, false);
    if (result == PICO_ERROR_GENERIC) {
        printf("I2C reception error\n");
    }
}

void MS4525_Calculate() {
    uint8_t status = (dataPitot[0] & 0xC0) >> 6;
    int16_t dp_raw, dT_raw;
    if (status == 0 || status == 1) {
        dp_raw = ((dataPitot[0] & 0x3F) << 8) | dataPitot[1];
        dT_raw = (dataPitot[2] << 3) | ((dataPitot[3] & 0xE0) >> 5);
    }
    const float P_max = 1.0f; // psi
    const float P_min = -1.0f; // psi
    const float PSI_to_Pa = 6894.757f;
    float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
    MS4525_Pressure = diff_press_PSI * PSI_to_Pa;
    MS4525_Pressure = MS4525_Pressure - PRESSURE_OFFSET;
    MS4525_Temperature = ((200.0f * dT_raw) / 2047) - 50;

    // Hava yoğunluğunu sıcaklığa göre hesapla (deniz seviyesinde varsayılan basınç)
    float temperature_K = MS4525_Temperature + 273.15f; // Sıcaklık Kelvin cinsinden
    const float standard_pressure = 101325.0f; // Pa
    const float specific_gas_constant = 287.05f; // J/(kg·K)
    float air_density = standard_pressure / (specific_gas_constant * temperature_K);

    // Hava hızını hesapla
    if (MS4525_Pressure > 0) {
        MS4525_Airspeed = sqrt(2 * MS4525_Pressure / air_density);
    } else {
        MS4525_Airspeed = 0; // Basınç farkı negatif veya sıfırsa hız sıfır olmalı
    }
}
