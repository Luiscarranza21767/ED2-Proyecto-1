/* 
 * File:   DS3231.h
 * Author: Pablo
 *
 * Created on 10 de febrero de 2023, 11:24 AM
 */

#ifndef DS3231_H
#define	DS3231_H

#include "I2C.h"
uint8_t leer_x(uint8_t address);
void enviar_x(uint8_t val1, uint8_t val2);
uint8_t convertir(uint8_t valor);
uint8_t desconvertir(uint8_t valor);

#endif	/* DS3231_H */

