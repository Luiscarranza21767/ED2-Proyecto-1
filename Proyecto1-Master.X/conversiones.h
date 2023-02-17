/* 
 * File:   conversiones.h
 * Author: Pablo
 *
 * Created on 27 de enero de 2023, 12:47 AM
 */

#ifndef CONVERSIONES_H
#define	CONVERSIONES_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

uint8_t descomponer(int pos, uint8_t num);
char inttochar(uint8_t num);

#endif	/* CONVERSIONES_H */

