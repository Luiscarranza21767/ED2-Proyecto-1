/* Universidad del Valle de Guatemala
 IE3054 Electrónica Digital 2
 Autor: Luis Pablo Carranza
 Compilador: XC8, MPLAB X IDE (v6.00)
 Proyecto: Laboratorio No.04
 Hardware PIC16F887
 Creado: 09/02/23
 Última Modificación: 15/02/23*/

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSC 
//oscillator without clock out)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and 
//can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR 
//pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
//protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
//protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/
//External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-
//Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin 
//has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit 
//(Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
//(Write protection off)

#include <xc.h> // include processor files - each processor file is guarded.  
#include <pic16f887.h>
#include "LCD.h"
#include "oscilador.h"
#include "conversiones.h"
#include "I2C.h"
#include <stdint.h>
#include <stdio.h>
#include "DS3231.h"


#define _XTAL_FREQ 8000000

uint8_t lecADC;
float conver;
char valADC[3];
char unidad;
char decena;

uint8_t sec, segundos;
uint8_t min, minutos;

uint8_t modo;

void portsetup(void);
void Escribir_dato(uint8_t dato, uint8_t posx, uint8_t posy);

void main(void) {
    setupINTOSC(7);     //Oscilador a 8MHz
    portsetup();
    // Inicio y constantes del display
    Lcd_Init();
    Lcd_Clear();
    Lcd_Set_Cursor(1,13);
    Lcd_Write_Char(':');
    Lcd_Set_Cursor(2,9);
    Lcd_Write_String("S:  :");
    

    modo = 0;
    sec = 0;
    min = 0;
    // Valores iniciales de fecha y hora
    enviar_x(0, 0);
    
    while(1){
        PORTA = modo;
        
        enviar_x(0, 0); // Enviar datos al RTC
        // Comunicación con DS3231
        sec = leer_x(0x00);     // Leer segundo
        Escribir_dato(sec, 14, 1);
        
        min = leer_x(0x01);     // Leer minuto
        Escribir_dato(min, 11, 1);
              
        if(!PORTBbits.RB4){     // Entrar al modo de configuración de tiempo
            __delay_ms(20);
            while(PORTBbits.RB3){   // Sale del modo hasta que se presiona B3
                __delay_ms(30);
                Escribir_dato(sec, 14, 1);
                Escribir_dato(min, 11, 1);
                // Cambio de modo
                if(PORTBbits.RB7 == 0){ // Botón que cambia entre minutos y segundos
                    __delay_ms(20);
                    if (modo < 1){
                        modo += 1;
                    }
                    else {
                        modo = 0;
                    }
                }
                // Incrementos
                if(PORTBbits.RB6 == 0){ // Si se presiona incrementa
                    __delay_ms(50);
                    if (modo == 0){
                        if (sec<59){
                            sec ++;   
                        } 
                        else {
                            sec = 0;
                        }
                    }
                    else if(modo == 1){
                        if (min<59){
                            min++;
                        }
                        else {
                            min = 0;
                        }
                    }           
                    
                }

                // Decrementos
                if(PORTBbits.RB5 == 0){ // Si se presiona decrementa
                    __delay_ms(50);

                    if (modo == 0){
                        if (sec > 0){
                            sec--;   
                        } 
                        else {
                            sec = 59;
                        }
                    }
                    else if(modo == 1){
                        if (min > 0){
                            min--;
                        }
                        else {
                            min = 59;
                        }
                    }
                }  
            }
            enviar_x(0,0);
            Escribir_dato(sec, 14, 2);
            Escribir_dato(min, 11, 2);
            while(PORTBbits.RB2 & !((sec == segundos) & (min == minutos)) ){ //Mientras no se presione cancelar o no termine
                segundos = leer_x(0x00);
                minutos = leer_x(0x01);
                Escribir_dato(segundos, 14, 1);
                Escribir_dato(minutos, 11, 1);
                
            
            }
            
            
            //
//            segundos = desconvertir(sec);
//            minutos = desconvertir(min);
            
        }
    }
}
void portsetup(){
    ANSEL = 0;
    ANSELH = 0;
    TRISA = 0;
    PORTA = 0; 
    TRISD = 0;
    PORTD = 0;
    
    // Configuración del puerto B 
    TRISB = 0b11111100;
    PORTB = 0b11111100;
    WPUB = 0b11111100;      // Habilita el Weak Pull-Up en el puerto B
    OPTION_REGbits.nRBPU = 0;   // Deshabilita el bit de RBPU
    
    I2C_Master_Init(100000);        // Inicializar Comuncación I2C

}

void Escribir_dato(uint8_t dato, uint8_t posx, uint8_t posy){
    Lcd_Set_Cursor(posy, posx+1);
    unidad = inttochar(descomponer(0, dato));
    Lcd_Write_Char(unidad);
    Lcd_Set_Cursor(posy, posx);
    decena = inttochar(descomponer(1, dato));
    Lcd_Write_Char(decena);
}