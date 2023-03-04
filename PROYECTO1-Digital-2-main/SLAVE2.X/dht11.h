/* #include "dht11.h"
*
* Creada por: Ing. Abiezer Hernandez O.
* Fecha de creacion: 10/11/2020
* Electronica y Circuitos
*
*/

#include <xc.h>
#define _XTAL_FREQ 8000000

#define DHT11_PIN_PORT 	PORTDbits.RD0
#define DHT11_PIN_DIR 	TRISDbits.TRISD0

void DHT11_Start(void);
void DHT11_Response(void);
int DHT11_Read_Byte(void);
short DHT11_Read_Data(uint8_t *temint, uint8_t *tempdec);
