/*
 * File:   PWM.c
 * Author: Migue
 *
 * Created on 22 de febrero de 2023, 05:55 PM
 */

#include "PWM.h"
#include <xc.h>

void setupPWM(void){

    TRISCbits.TRISC2 = 1; 
    CCP1CON = 0b00001100;        // P1A como PWM 
    //Configuración TMR2
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b01;    // Prescaler de 1:4
    TMR2ON = 1;                 // Encender timer 2 

    while(!TMR2IF);
    TRISCbits.TRISC2 = 0;       // Habilitamos la salida del PWM   
}

// Calcular el Duty Cicle máximo según la frecuencia elegida
uint32_t pwmMaxDuty(const uint32_t freq)
{
  return(_XTAL_FREQ/(freq*TMR2PRESCALE));
}

// Calcular el valor del PR2 según el preescaler y la frecuencia de oscilación
void initPwm(const uint32_t freq)
{
    //calculate period register value
    PR2 = (uint8_t)((_XTAL_FREQ/(freq*4*TMR2PRESCALE)) - 1);
}

// Aplica el duty cycle según la variable y la frecuencia
void applyPWMDutyCycle(uint16_t dutyCycle, const uint32_t freq)
{
    if(dutyCycle<1024)
    {
        //1023 porque la resolución es de 10 bits
        dutyCycle = (uint16_t)(((float)dutyCycle/1023)*pwmMaxDuty(freq));
        CCP1CON &= 0xCF;                 // Cambia bit4 y 5 a cero 
        CCP1CON |= (0x30&(dutyCycle<<4)); // Asigna los dos bits menos significativos a CCP1CON
        CCPR1L = (uint8_t)(dutyCycle>>2); // Pone los bits más significativos a CCPR1L
    }
}