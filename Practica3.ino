#include <Controllino.h>
#include "Stone_HMI_Define.h"
#include "Procesar_HMI.h"

const int led              = CONTROLLINO_D0; // Salida digital D0 
const int led1             = CONTROLLINO_D6; // Salida digital D6
const int btn1             = CONTROLLINO_I16; // Salida digital I16 
const int btn2             = CONTROLLINO_I17; // Salida digital I17 
int       pwmValue         = 0;              // valor convertido (0-255)
int       pwmValue2         = 0;              // valor convertido (0-255)
float     dutyCyclePercent = 0;              // valor en porcentaje (0-100)
float     dutyCyclePercent1 = 0;              // valor en porcentaje (0-100)
int       bandera1          = 0;              // Bandera de encendido/apagado del led1
int       bandera2          = 0;              // Bandera de encendido/apagado del led2



void setup() {
  Serial.begin(115200);   // Comunicación serial con el PC
  Serial2.begin(115200);  // Comunicación serial con el HMI
  pinMode(led, OUTPUT);   // led como salida
  pinMode(led1, OUTPUT);   // led1 como salida
  pinMode(btn1, INPUT);   // bont1 como entrada
  pinMode(btn2, INPUT);   // bont2 como entrada
  HMI_init();             // Inicializa el sistema de colas para las respuestas el HMI
  Stone_HMI_Set_Value("spin_box", "spin_box1", NULL, 0);  // Pone en 0 el valor del spin box en el HMI. 
  Stone_HMI_Set_Value("spin_box", "spin_box2", NULL, 0);  // Pone en 0 el valor del spin box 2 en el HMI.
}

void loop() {
  dutyCyclePercent=HMI_get_value("spin_box", "spin_box1"); // Obtiene el valor del spin_box1
  dutyCyclePercent1=HMI_get_value("spin_box", "spin_box2"); // Obtiene el valor del spin_box2

  if (dutyCyclePercent >= 0 && dutyCyclePercent <=100) {
    if (digitalRead(btn1) == HIGH){ // Verifica si el btn1 esta pulsado 
      if (bandera1 == 1){ // Cambia el estado de la bandera 
        bandera1 = 0;
      }
      else {
        bandera1 = 1;
      }
    }
    if (bandera1 == 1){                                       // Verifica la bandera de encendido
      pwmValue = map(dutyCyclePercent, 0, 100, 0, 255);      // Mapea el valor de duty cycle en porcentaje a valores de 0 a 255 
      analogWrite(led, pwmValue);                             //  Enciende el led1 con el valor mapeado
    }
    else{                                                       
      analogWrite(led, 0);                                     // Apaga por completo el led
    }
  }
  if (dutyCyclePercent1 >= 0 && dutyCyclePercent1 <=100) {
    if (digitalRead(btn2) == HIGH){ // Verifica si el btn2 esta pulsado 
      if (bandera2 == 1){               // Cambia el estado de la bandera 2
        bandera2 = 0;
      }
      else {
        bandera2 = 1;
      }
    }
    if (bandera2 == 1){                   // Verifica la bandera de encendido
      pwmValue2 = map(dutyCyclePercent1, 0, 100, 0, 255);      // Mapea el valor de duty cycle en porcentaje a valores de 0 a 255
      analogWrite(led1, pwmValue2);                               //Enciende el led1 con el valor mapeado    
    }
    else {
      analogWrite(led1, 0);                                   // Apaga por completo el led1
    }
  }
  else {
    Serial.println("Ingresa un valor entre 0 y 100.");
  }
}