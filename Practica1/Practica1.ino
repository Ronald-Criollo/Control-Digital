//Practica 2: Encender todos los leds del tablero secuencialmente
#include <Controllino.h> // Librería de controllino

// Retardo no Bloqueante
unsigned long t_antes = 0;
unsigned long t_actual = 0;
unsigned long intervalo = 500; //ms

int i = 0;

int D0 = CONTROLLINO_D0;
int D1 = CONTROLLINO_D1;
  int D2 = CONTROLLINO_D2;
  int D6 = CONTROLLINO_D6;
  int D7 = CONTROLLINO_D7;
  int D8 = CONTROLLINO_D8;
  int D12 = CONTROLLINO_D12;
  int D13 = CONTROLLINO_D13;
  int D14 = CONTROLLINO_D14;
  
int leds[9] = {D0, D6, D12, D13, D14, D8, D2, D1 ,D7};

// Puntero 
  int* ptr = leds;

void setup() {
  pinMode(CONTROLLINO_D0, OUTPUT);  // Salida digital D0
  pinMode(CONTROLLINO_D1, OUTPUT);  // Salida digital D1
  pinMode(CONTROLLINO_D2, OUTPUT);  // Salida digital D2
  pinMode(CONTROLLINO_D6, OUTPUT);  // Salida digital D6
  pinMode(CONTROLLINO_D7, OUTPUT);  // Salida digital D7
  pinMode(CONTROLLINO_D8, OUTPUT);  // Salida digital D8
  pinMode(CONTROLLINO_D12, OUTPUT); // Salida digital D12
  pinMode(CONTROLLINO_D13, OUTPUT); // Salida digital D13
  pinMode(CONTROLLINO_D14, OUTPUT); // Salida digital D14
}

void loop() {
  
  t_actual = millis();

  digitalWrite(*(ptr+i),HIGH);

  if (t_actual - t_antes >= intervalo){
    digitalWrite(*(ptr+i),LOW);
    t_antes = t_actual;
    i += 1;
    if (i == 9){
      i = 0;
}
}
}