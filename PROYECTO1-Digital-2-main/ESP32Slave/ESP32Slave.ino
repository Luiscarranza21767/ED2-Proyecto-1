//#include "config.h"

#define IO_USERNAME  "LuisCarranza21767"
#define IO_KEY "aio_CHkT24LMKM2RAbTAaJ8M6J0ZClrX"


#define WIFI_SSID "Nexxt_662768-2.4G"
#define WIFI_PASS "12345678"

#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_DEV_ADDR 0x48

#include "AdafruitIO_WiFi.h"
#include <Arduino.h>
#include "Wire.h"


AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
// this int will hold the current count for our sketch
int temp;
int vel;
int seg, minuto;
int sega, mina;
int a = 1;
int b = 1;
int c = 0;
int d = 0;
int e = 1;
int f = 1;


uint32_t i = 0;

void onRequest(){
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
}

void onReceive(int len){
  //Serial.printf("onReceive[%d]: ", len);
  while(Wire.available()){
    temp = Wire.read();
    vel = Wire.read();
    minuto = Wire.read();
    seg = Wire.read();
    mina = Wire.read();
    sega = Wire.read();

  }
}

// set up the 'counter' feed
AdafruitIO_Feed *temperatura = io.feed("temperatura");
AdafruitIO_Feed *velocidad = io.feed("velocidad");
AdafruitIO_Feed *seg_stop = io.feed("seg_stop");
AdafruitIO_Feed *min_stop = io.feed("min_stop");
AdafruitIO_Feed *seg_a = io.feed("seg_a");
AdafruitIO_Feed *min_a = io.feed("min_a");
// AdafruitIO_Feed *velocidad = io.feed("velocidad");
// AdafruitIO_Feed *minutos = io.feed("minutos");

void escribir_datos(){
  Serial.println("sending: ");
  Serial.print("Temperatura: ");
  Serial.println(temp);
  Serial.print("Velocidad: ");
  Serial.println(vel);
  Serial.print("Tiempo stop: ");
  Serial.print(minuto);
  Serial.print(":");
  Serial.println(seg);
  Serial.print("Tiempo actual: ");
  Serial.print(mina);
  Serial.print(":");
  Serial.println(sega);
}

void setup() {

  // start the serial connection
  
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  Serial.begin(9600);

  // wait for serial monitor to open
  while(! Serial);
  Serial.print("Connecting to Adafruit IO");
  // connect to io.adafruit.com
  io.connect();
  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
}

void loop() {
  io.run();

  if ((a != seg) || (b!=minuto)){
    seg_stop->save(seg);
    min_stop->save(minuto);
    a = seg;
    b = minuto;
    escribir_datos();
    delay(2000);
  }
  if (c != temp){
    temperatura->save(temp);
    c = temp;
    escribir_datos();
    delay(1000);
  }
  if (d != vel){
    velocidad->save(vel);
    d = vel;
    escribir_datos();
    delay(1000);
  }

  if ((e != sega) || (f!=mina)){
    seg_a->save(sega);
    min_a->save(mina);
    e = sega;
    f = mina;
    escribir_datos();
    delay(3000);
  }

}
