/*
 * diffdrive_pico_firmware — BNO055 sur I2C0, en mode IMU (fusion 6 axes).
 *
 * Le magnetometre est volontairement desactive : sur une base a moteurs a
 * courant continu il ne mesure que le champ des moteurs. Le mode IMU (0x08)
 * fusionne accelerometre et gyroscope uniquement.
 *
 * Le firmware ne convertit rien : les six entiers 16 bits sortent tels quels
 * dans la trame. Facteurs a appliquer cote Pi, avec UNIT_SEL = 0x00 :
 *     accelerometre : 100 LSB par m/s^2
 *     gyroscope     :  16 LSB par degre/s
 *
 * Absence non fatale : si le composant ne repond pas, le robot roule quand
 * meme, la trame porte des zeros, et une redetection est retentee toutes les
 * IMU_RETRY_TICKS periodes de controle.
 *
 * Toutes les transactions utilisent les variantes _timeout_us du SDK : un bus
 * mort ne peut pas figer la boucle de controle.
 */
#ifndef DIFFDRIVE_PICO_BNO055_H
#define DIFFDRIVE_PICO_BNO055_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int16_t gyro[3];    /* gx, gy, gz — bruts */
    int16_t accel[3];   /* ax, ay, az — bruts */
    bool    present;    /* composant detecte et configure                */
    uint8_t addr;       /* adresse I2C retenue                           */
    uint8_t chip_id;    /* dernier CHIP_ID lu (0xA0 attendu)             */
    uint8_t calib;      /* registre CALIB_STAT : sys<<6|gyr<<4|acc<<2|mag */
    uint32_t read_errors;
} bno055_t;

/* Registres, utiles aussi au mode de test t 2. */
#define BNO055_REG_CHIP_ID      0x00
#define BNO055_REG_PAGE_ID      0x07
#define BNO055_REG_ACC_DATA     0x08
#define BNO055_REG_MAG_DATA     0x0E
#define BNO055_REG_GYR_DATA     0x14
#define BNO055_REG_CALIB_STAT   0x35
#define BNO055_REG_ST_RESULT    0x36
#define BNO055_REG_SYS_STATUS   0x39
#define BNO055_REG_SYS_ERR      0x3A
#define BNO055_REG_UNIT_SEL     0x3B
#define BNO055_REG_OPR_MODE     0x3D
#define BNO055_REG_PWR_MODE     0x3E
#define BNO055_REG_SYS_TRIGGER  0x3F

#define BNO055_CHIP_ID_VALUE    0xA0
#define BNO055_OPMODE_CONFIG    0x00
#define BNO055_OPMODE_IMU       0x08
#define BNO055_PWRMODE_NORMAL   0x00

/* Initialise le bus I2C puis tente la detection et la configuration.
 * Renvoie true si l'IMU est operationnelle. Ne bloque jamais indefiniment. */
bool bno055_init(void);

/* Une lecture en rafale de 18 octets (accel, mag, gyro) — une seule
 * transaction, environ 1.7 ms a 100 kHz. Met les valeurs a zero en cas
 * d'echec. */
void bno055_read(void);

/* Nouvelle tentative de detection quand le composant est absent. Une seule
 * lecture de CHIP_ID, avec temporisation courte. */
void bno055_retry(void);

/* Rafraichit le registre CALIB_STAT (utilise par t 2). */
void bno055_read_calib(void);

const bno055_t *bno055_state(void);

/* Acces bruts pour le mode de test t 2. */
bool bno055_read_reg(uint8_t addr, uint8_t reg, uint8_t *dst, size_t len);
bool bno055_probe_addr(uint8_t addr);

#endif /* DIFFDRIVE_PICO_BNO055_H */
