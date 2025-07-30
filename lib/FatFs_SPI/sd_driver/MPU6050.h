#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>     // Para os tipos int16_t

// Endereço padrão do MPU6050
#define MPU6050_ADDR 0x68

// Porta e pinos I2C utilizados para o MPU6050
#define I2C_PORT      i2c0
#define I2C_SDA       0
#define I2C_SCL       1

// Função para inicializar e resetar o MPU6050
void mpu6050_reset(void);

// Função para ler os dados brutos do acelerômetro, giroscópio e temperatura
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

#endif // MPU6050_H
