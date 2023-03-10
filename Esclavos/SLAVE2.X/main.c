/* Universidad del Valle de Guatemala
 IE3054 Electrónica Digital 2
 Autor: Luis Pablo Carranza y Miguel Chacón
 Compilador: XC8, MPLAB X IDE (v6.00)
 Proyecto: Proyecto 1 - Slave 2 - Sensor DHT11
 Hardware PIC16F887
 Creado: 09/02/23
 Última Modificación: 7/03/23*/
//*****************************************************************************
// Palabra de configuración
//*****************************************************************************
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//******************************************    ***********************************
// Definición e importación de librerías
//*****************************************************************************
#include <stdint.h>
#include <pic16f887.h>
#include "I2C.h"
#include "setupADC.h"
#include "oscilador.h"
#include <xc.h>
#include "dht11.h"                      // Libreria del sensor DHT11
//*****************************************************************************
// Definición de variables
//*****************************************************************************
//#define _XTAL_FREQ 500000
#define _XTAL_FREQ 8000000
uint8_t dato;
//uint8_t SERVO = 0;

//*****************************************************************************
// Definición de funciones para que se puedan colocar después del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void portsetup(void);
void setup_portb(void);
void setupTMR0(void);
void leer_temp(void);
    
short dht_ok;                           // Flag de verificacion del bit de paridad
uint8_t temperaturai;                      // Almacena la temperatura
uint8_t z;
uint8_t x;
uint8_t check;

//*****************************************************************************
// Código de Interrupción 
//*****************************************************************************

void __interrupt() isr(void){
    if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;                 // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;       // Clear the overflow flag
            SSPCONbits.WCOL = 0;        // Clear the collision bit
            SSPCONbits.CKP = 1;         // Enables SCL (Clock)
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            //__delay_us(7);
            z = SSPBUF;                 // Lectura del SSBUF para limpiar el buffer y la bandera BF
            //__delay_us(2);
            PIR1bits.SSPIF = 0;         // Limpia bandera de interrupción recepción/transmisión SSP
            SSPCONbits.CKP = 1;         // Habilita entrada de pulsos de reloj SCL
            while(!SSPSTATbits.BF);     // Esperar a que la recepción se complete
            //z = SSPBUF;             // Guardar en el PORTD el valor del buffer de recepción
            dato = SSPBUF;
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF;
            SSPSTATbits.BF = 0;
            leer_temp();
            SSPBUF = temperaturai;
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
            
        }
       
        PIR1bits.SSPIF = 0;    
    }
    
}

//*****************************************************************************
// Main
//*****************************************************************************
void main(void) {
    setupINTOSC(7);
    portsetup();
    dato = 0;
    
    //*************************************************************************
    // Loop infinito
    //*************************************************************************
    while(1){
        __delay_ms(50);   

    }
}
//*****************************************************************************
// Función de Inicialización
//*****************************************************************************

void portsetup(){
    ANSEL = 0;
    ANSELH = 0;

    //Configuración del TMR1
    T1CONbits.TMR1CS = 0;           // Oscilador Interno
    T1CONbits.T1CKPS = 0b01;        // Prescaler 2
    T1CONbits.TMR1ON = 0;           // Apagamos el TMR1 
    I2C_Slave_Init(0xa0); 
}

void leer_temp(void){
    __delay_ms(800);
    DHT11_Start();
    check = DHT11_Response();
    
    if(check == 1){
        x = DHT11_Read();
        x = DHT11_Read();
        temperaturai = DHT11_Read();
        x = DHT11_Read();
        x = DHT11_Read();
        T1CONbits.TMR1ON = 0;       // Apagamos el TMR1
    }
       
}