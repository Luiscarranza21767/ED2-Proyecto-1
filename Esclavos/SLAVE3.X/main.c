/* Universidad del Valle de Guatemala
 IE3054 Electrónica Digital 2
 Autor: Luis Pablo Carranza y Miguel Chacón
 Compilador: XC8, MPLAB X IDE (v6.00)
 Proyecto: Proyecto 1 - Slave 3 - Motores
 Hardware PIC16F887
 Creado: 09/02/23
 Última Modificación: 8/03/23*/
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
#include "PWM.h"
//*****************************************************************************
// Definición de variables
//*****************************************************************************
//#define _XTAL_FREQ 500000
#define _XTAL_FREQ 8000000
#define valTMR0 100
uint8_t dato = 0;
uint8_t SERVO = 0;
uint8_t ADC;
uint16_t dutycycle = 0;
uint16_t dutyCycleApply = 0;
const uint32_t pwmFreq = 5000;

//*****************************************************************************
// Definición de funciones para que se puedan colocar después del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*****************************************************************************
void portsetup(void);
void setupPWM(void);                //setup del primer pwm
void setupTMR0(void);

    
uint8_t z;


//*****************************************************************************
// Código de Interrupción 
//*****************************************************************************

void __interrupt() isr(void){
    // Interrupción de TMR0
    if(INTCONbits.T0IF){
        INTCONbits.T0IF = 0;
        TMR0 = valTMR0;                   // Valor inicial del TMR0
        // Revisa el valor de la variable para colocar la posición del servo
        if(SERVO == 0){                   // Periodo para 0°
            PORTDbits.RD1 = 1;
            __delay_us(900);
            PORTDbits.RD1 = 0;
        }                                 // Periodo para 90° aprox
        else if (SERVO == 1){
            PORTDbits.RD1 = 1;
            __delay_us(1900);
            PORTDbits.RD1 = 0;
        }
    }
    //Interrupción del I2C
    if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            z = SSPBUF;                 // Read the previous value to clear the buffer
            SSPCONbits.SSPOV = 0;       // Clear the overflow flag
            SSPCONbits.WCOL = 0;        // Clear the collision bit
            SSPCONbits.CKP = 1;         // Enables SCL (Clock)
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            z = SSPBUF;                 // Lectura del SSBUF para limpiar el buffer y la bandera BF
            //PIR1bits.SSPIF = 0;       // Limpia bandera de interrupción recepción/transmisión SSP
            SSPCONbits.CKP = 1;         // Habilita entrada de pulsos de reloj SCL
            while(!SSPSTATbits.BF);     // Esperar a que la recepción se complete
            dato = SSPBUF;
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF;
            SSPSTATbits.BF = 0;
            SSPBUF = z;
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
    setupINTOSC(7);     //8MHz
    portsetup();
    setupTMR0();
    setupPWM();
    initPwm(pwmFreq); // Initialize PWM
    applyPWMDutyCycle(dutycycle,pwmFreq);
    ADC_config(0x01);
    
    //*************************************************************************
    // Loop infinito
    //*************************************************************************
    while(1){
        if (dato == 0){
            SERVO = 0;
        }
        else if(dato == 1){
            SERVO = 1;
        }
        else if (dato == 2){
            ADC = ADC_read(0);          // Hace la lectura del ADC
            dutycycle = 4*ADC;          // Mapea a un valor de 1024
            if (dutycycle != dutyCycleApply){   //Compara Dutycycle actual con el nuevo
                applyPWMDutyCycle(dutycycle,pwmFreq);
                dutyCycleApply = dutycycle;     // Aplica el nuevo Duty Cycle
            }  
        }
        else if (dato == 3){
            dutycycle = 0;              // Si recibe un 3 se apaga el motor
            if (dutycycle != dutyCycleApply){
                applyPWMDutyCycle(dutycycle,pwmFreq);
                dutyCycleApply = dutycycle;     // Aplica el Duty Cycle = 0
            }  
        } 
    }
}
//*****************************************************************************
// Función de Inicialización
//*****************************************************************************

void portsetup(){
    ANSEL = 0;
    ANSELH = 0;
    TRISDbits.TRISD1 = 0;       // Pin de salida del PWM para servomotor
    PORTDbits.RD1 = 0;
    I2C_Slave_Init(0xb0);       // Inicia esclavo con dirección 0xb
}

void setupTMR0(void){
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    INTCONbits.T0IE = 1;        // Habilitar interrupción de TMR0
    INTCONbits.T0IF = 0;        // Desactivar la bandera de TMR0
    
    OPTION_REGbits.T0CS = 0;    // Fosc/4
    OPTION_REGbits.T0SE = 0;    // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;     // Prescaler para TMR0
    OPTION_REGbits.PS = 0b111;  // Prescaler 1:64
    TMR0 = valTMR0;             // Valor inicial del TMR0
}


