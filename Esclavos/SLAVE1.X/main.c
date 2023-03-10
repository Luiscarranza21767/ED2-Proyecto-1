/* Universidad del Valle de Guatemala
 IE3054 Electrónica Digital 2
 Autor: Luis Pablo Carranza y Miguel Chacón
 Compilador: XC8, MPLAB X IDE (v6.00)
 Proyecto: Proyecto 1 - Slave 1 - Sensor de velocidad
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

//*****************************************************************************
// Definición de variables
//*****************************************************************************
#define _XTAL_FREQ 8000000
#define valTMR0 6;

uint8_t dato;
uint8_t z;
uint8_t i = 0;
uint16_t cont;
uint16_t rpm, rpm_Htemp, rpm_Ltemp;
uint8_t rpm_inst;
float rpm_temp = 0;
uint8_t rpm_H;
uint8_t rpm_L;

void setup(void);
void setupTMR0(void);
void setupTMR1(void);
//*****************************************************************************
// Código de Interrupción 
//*****************************************************************************
void __interrupt() isr(void){
    if (PIR1bits.TMR1IF){
        TMR1IF = 0;
        TMR1H = 11;                 // preset for timer1 MSB register
        TMR1L = 220;                // preset for timer1 LSB register
        cont = 0; 
        rpm_inst = ((cont*0.08333)/0.25)*60;
               
    }
    
    if (INTCONbits.RBIF){
        if(PORTBbits.RB7){
            cont++;
        }
    }
    
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
            dato = SSPBUF;             // Guardar en el PORTD el valor del buffer de recepción
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF;
            BF = 0;
            SSPBUF = rpm_inst;
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
    setupINTOSC(7); //INTRC A 8MHz
    setup();
    setupTMR1();
    dato = 0;
    //*************************************************************************
    // Loop infinito
    //*************************************************************************
    while(1){ 

    }       
       
}
//*****************************************************************************
// Función de Inicialización
//*****************************************************************************
void setup(void){
    ANSEL = 0;
    ANSELH = 0;
          
    TRISDbits.TRISD6 = 1;       // Pin que toca parte del SCL pero no se toma en cuenta
    TRISAbits.TRISA0 = 0;
    PORTA = 0;
    
    TRISB = 0b10000100;  
    PORTB = 0;
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    INTCONbits.RBIE = 1;    // Habilita interrupción del puerto B
    INTCONbits.RBIF = 0;    // Apaga la bandera de interrupción del puerto B
    IOCB = 0b10000000;      // Habilita la interrupción en cambio (IoC)
    OPTION_REGbits.nRBPU = 0;   // Deshabilita el bit de RBPU
    
    I2C_Slave_Init(0x80);   
}

void setupTMR1(void){

    T1CONbits.T1CKPS1 = 1;   // bits 5-4  Prescaler Rate Select bits
    T1CONbits.T1CKPS0 = 1;   // bit 4
    T1CONbits.T1OSCEN = 1;   // bit 3 Timer1 Oscillator Enable Control bit 1 = on
    T1CONbits.T1SYNC = 1;    // bit 2 Timer1 External Clock Input Synchronization Control bit...1 = Do not synchronize external clock input
    T1CONbits.TMR1CS = 0;    // bit 1 Timer1 Clock Source Select bit...0 = Internal clock (FOSC/4)
        //Timer1 Registers Prescaler= 8 - TMR1 Preset = 3036 - Freq = 4.00 Hz - Period = 0.250000 seconds
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    PIE1bits.TMR1IE = 1;       // Interrupción habilitada por desbordamiento
    PIR1bits.TMR1IF = 0;       // Poner a 0 la bandera de bit del TMR1IF
    
    T1CONbits.TMR1ON = 1;    // bit 0 enables timer
    TMR1H = 11;             // preset for timer1 MSB register
    TMR1L = 220;             // preset for timer1 LSB register

    
}
