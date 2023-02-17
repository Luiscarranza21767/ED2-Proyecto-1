/* 
 * File:   conversiones.h
 * Author: Pablo
 *
 * Created on 27 de enero de 2023, 12:47 AM
 */
#include "conversiones.h"
#include <stdint.h>

//Funcion para mover dos posiciones decimales

char inttochar(uint8_t num){
    if(num == 0){
        return '0';
    }
    else if(num == 1){
        return '1';
    }
    else if(num == 2){
        return '2';
    }
    else if(num == 3){
        return '3';
    }
    else if(num == 4){
        return '4';
    }
    else if(num == 5){
        return '5';
    }
    else if(num == 6){
        return '6';
    }
    else if(num == 7){
        return '7';
    }
    else if(num == 8){
        return '8';
    }
    else if(num == 9){
        return '9';
    }
}
//Funciones para separar un numero en unidades, decenas y centenas
uint8_t descomponer(int pos, uint8_t num){
    uint8_t cent;
    uint8_t dec;
    uint8_t uni;
    cent = (num/100);
    dec = (num%100);
    uni = (dec%10);
    if(pos == 2){
        return cent;
    }
    else if(pos == 1){
        return (dec/10);
    }
    else if (pos == 0){
        return uni;
    }  
    
}


