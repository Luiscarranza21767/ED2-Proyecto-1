#include "DS3231.h"
#include "I2C.h"


uint8_t convertir(uint8_t valor){
    uint8_t unidad;
    uint8_t decena;
    uint8_t conversion;
    unidad = valor & 0x0F;
    decena = ((valor & 0xF0) >> 4);
    conversion = unidad+(decena*10);
    return conversion;
}

uint8_t desconvertir(uint8_t valor){
    uint8_t decena;
    uint8_t unidad;
    decena = (valor/10);
    decena = (decena << 4);
    unidad = (valor%10);
    return (decena + unidad);
}

uint8_t leer_x(uint8_t address){
    uint8_t valor;
    I2C_Master_Start();     // Inicia la comunicación I2C
    I2C_Master_Write(0xD0);        //Escoje dirección del reloj
    I2C_Master_Write(address);        //Posición donde va leer
    I2C_Master_RepeatedStart();    //Reinicia la comuniación I2C
    I2C_Master_Write(0xD1);        //Leer posición
    valor = I2C_Master_Read(0);      //lee posicion de reloj
    I2C_Master_Stop();             //Termina comunicaion I2C
    valor = convertir(valor);

    return valor;
}

void enviar_x(uint8_t val1, uint8_t val2){
    I2C_Master_Start();            //Incia comunicaión I2C
    I2C_Master_Write(0xD0);        //Escoje dirección del reloj
    I2C_Master_Write(0);
    I2C_Master_Write(val1);        // Escribir minutos
    I2C_Master_Write(val2);        // Escribir hora
    I2C_Master_Stop(); 
}


   