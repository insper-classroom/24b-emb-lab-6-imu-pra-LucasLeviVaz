#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>

#define SAMPLE_PERIOD 0.01f
#define CLICK 10000
#define MOVEMENT_SCALING_FACTOR 0.5f // Fator de escala para suavizar o movimento

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

QueueHandle_t xQueueI2C;
QueueHandle_t yQueueI2C;

// Estrutura para armazenar os dados do mouse
typedef struct I2C {
    uint8_t axis;  // 0 para eixo X, 1 para eixo Y
    int16_t val;   // Valor da coordenada (posição)
} I2C_t;

// Inicializa a UART
void uart_init_custom() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
}

// Tarefa da UART para enviar os dados do mouse
void uart_task(void *p) {
       I2C_t data;

    while (1) {
        if (xQueueReceive(xQueueI2C, &data, portMAX_DELAY)) {
            uint8_t axis = data.axis;
            uint8_t val_msb = (data.val >> 8) & 0xFF;
            uint8_t val_lsb = data.val & 0xFF;

            uart_putc(UART_ID, axis);
            uart_putc(UART_ID, val_msb);
            uart_putc(UART_ID, val_lsb);
            uart_putc(UART_ID, 0xFF);
        }
    }
}

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];

    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);

    *temp = buffer[0] << 8 | buffer[1];
}

int zona_morta(int euler_angle) {

    if (euler_angle > -5 && euler_angle < 5) {
        euler_angle = 0;
    }

    return euler_angle;
}

void mpu6050_fundido_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();  // Resetar o sensor MPU6050

    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    while (true) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        // Atualiza o filtro AHRS
        const FusionVector gyroscope = {.axis.x = gyro[0] / 131.0f, .axis.y = gyro[1] / 131.0f, .axis.z = gyro[2] / 131.0f};
        const FusionVector accelerometer = {.axis.x = acceleration[0] / 16384.0f, .axis.y = acceleration[1] / 16384.0f, .axis.z = acceleration[2] / 16384.0f};   

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 0.05f);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        int yaw = -zona_morta(euler.angle.yaw);
        int roll = -zona_morta(euler.angle.roll);

        I2C_t data;
        data.axis = 0;
        data.val = yaw;
        xQueueSend(xQueueI2C, &data, portMAX_DELAY);

        data.axis = 1;
        data.val = roll;
        xQueueSend(xQueueI2C, &data, portMAX_DELAY);

        if (acceleration[1] > CLICK) {
            uint8_t axis = data.axis;
            uint8_t val_msb = (data.val >> 8) & 0xFF;
            uint8_t val_lsb = data.val & 0xFF;

            uart_putc(UART_ID, 2);
            uart_putc(UART_ID, val_msb);
            uart_putc(UART_ID, val_lsb);
            uart_putc(UART_ID, 0xFF);
        }

        // Aguardar próximo ciclo
        vTaskDelay(pdMS_TO_TICKS(50));  // Atraso de 50ms para manter o SAMPLE_PERIOD
    }
}

int main() {
    stdio_init_all();
    uart_init_custom();

    xQueueI2C = xQueueCreate(32, sizeof(I2C_t));

    xTaskCreate(mpu6050_fundido_task, "mpu6050_Task 2", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}
