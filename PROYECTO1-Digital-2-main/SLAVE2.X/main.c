//SLAVE2 MOTOR SERVO
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
void setupPWM(void);                //setup del primer pwm
void setup_portb(void);
void setupTMR0(void);
void readTemp(void);
    
short dht_ok;                           // Flag de verificacion del bit de paridad
uint8_t temperaturai;                      // Almacena la temperatura
uint8_t z;
uint8_t x;
uint8_t check;

//*****************************************************************************
// Código de Interrupción 
//*****************************************************************************

void __interrupt() isr(void){
//    if(INTCONbits.T0IF){
//        
//        INTCONbits.T0IF = 0;
//        TMR0 = 100;                   // Valor inicial del TMR0
//        if (SERVO == 0){
//            PORTDbits.RD1 = 1;
//            __delay_us(990);
//            PORTDbits.RD1 = 0;
//        }
//        else {
//            PORTDbits.RD1 = 1;
//            __delay_us(1500);
//            PORTDbits.RD1 = 0;
//        }
//    }
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
            //SSPCONbits.CKP = 1;
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            z = SSPBUF;
            SSPSTATbits.BF = 0;
            readTemp();
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
    //setup_portb();
    //setupTMR0();        // Configura el TMR0
    dato = 0;
    
    //*************************************************************************
    // Loop infinito
    //*************************************************************************
    while(1){
        //PORTA = temperaturai;
        //dht_ok = DHT11_Read_Data(&temperaturai, &temperaturad);
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
    //INTCONbits.GIE = 1;         // Habilitamos interrupciones
    //dht_ok = DHT11_Read_Data(&temperaturai, &temperaturad);    
    I2C_Slave_Init(0xa0); 
}

void setup_portb(void){
    TRISB = 0b11100000;
    //INTCONbits.RBIE = 1;    // Habilita interrupción del puerto B
    //INTCONbits.RBIF = 0;    // Apaga la bandera de interrupción del puerto B
    //IOCB = 0b11100000;      // Habilita la interrupción en cambio (IoC)
    WPUB = 0b11100000;      // Habilita el Weak Pull-Up en el puerto B
    OPTION_REGbits.nRBPU = 0;   // Deshabilita el bit de RBPU
}



void setupTMR0(void){
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    INTCONbits.T0IE = 1;        // Habilitar interrupción de TMR0
    INTCONbits.T0IF = 0;        // Desactivar la bandera de TMR0
    
    OPTION_REGbits.T0CS = 0;    // Fosc/4
    OPTION_REGbits.PSA = 0;     // Prescaler para TMR0
    OPTION_REGbits.PS = 0b111;  // Prescaler 1:16
    TMR0 = 100;                   // Valor inicial del TMR0
}

void readTemp(void){
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