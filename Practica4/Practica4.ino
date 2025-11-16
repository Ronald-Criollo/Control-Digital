//LIBRERIAS
#include <Controllino.h>
#include "Stone_HMI_Define.h"
#include "Procesar_HMI.h"    

// VARIABLES DEL MOTOR
const int pin_motor        = CONTROLLINO_D0;                // Pin de salida PWM al motor
volatile int8_t    slider_DutyCycle  = 0;                   // Valor leído del slider (para el Setpoint)
char      label2_text[10];                                  // Char para mostrar el Setpoint en el label2 del HMI

// VARIABLES PARA CONTEO DE PULSOS y RPM
const int entrada                = CONTROLLINO_IN1;         // Pin de entrada de pulsos
volatile unsigned long conteo_pulsos = 0;                   // Contador de pulsos
char label4_text[10];                                       // Char para mostrar las RPM en el label4 del HMI
float rpm                      = 0;                         // RPM calculadas
const uint16_t PULSOS_POR_REV  = 36;                        // Número de pulsos por revolución (Datos del EPC)

// Frecuencia de muestreo (Hz)
const float fs = 20; 

// Tiempo de muestreo (T) en segundos
const double T = 1.0 / fs; 

//VARIABLES PID
double kp = 0.1;  // Ganancia Proporcional
double Ti = 0.1;  // Tiempo Integral 
double Td = 0.01;  // Tiempo Derivativo 


// Variables de estado del PID
volatile double Setpoint_RPM = 0; 
volatile double Input_RPM = 0;
volatile double PID_Output = 0;

// Variables para la ecuación de recurrencia
double error           = 0; // Error actual (ek)
double last_error      = 0; // Error anterior (ek1)
double prev_last_error = 0; // Error de hace dos momentos (ek2)
double last_output     = 0; // Salida anterior (uk1)

// Variables para asignar los valores del spin_box
float kp_ui = 0;
float Ti_ui =0;
float Td_ui = 0;

// Limitacion del valor de salida del PID
const double PID_Output_Min = 0;
const double PID_Output_Max = 255;

// VARIABLES PARA CONTROLAR EL TIEMPO DE ENVIO DE DATOS AL HMI
unsigned long t_previo=0;
unsigned long t_previo1=0;

// FUNCIONES ADICIONALES 
void contarPulso();

void setup() {
  Serial.begin(115200);   // Comunicación serial con el PC
  Serial2.begin(115200);  // Comunicación serial con el HMI
  STONE_push_series("line_series", "line_series1", 0); //Envía un valor del eje X a graficar en el line_series1 que se pondrá al final
  STONE_push_series("line_series", "line_series2", 0); //Envía un valor del eje X a graficar en el line_series2 que se pondrá al final
  STONE_push_series("line_series", "line_series3", 0); //Envía un valor del eje X a graficar en el line_series1 que se pondrá al final
  Stone_HMI_Set_Value("slider", "slider1", NULL, 0); // Setea el slider de referencia en 0
  Stone_HMI_Set_Value("spin_box", "spin_box1", NULL, 0.1); // Setea el spinbox de Kp 0.1
  Stone_HMI_Set_Value("spin_box", "spin_box2", NULL, 0.1); // Setea el spinbox de Ti 0.1
  Stone_HMI_Set_Value("spin_box", "spin_box3", NULL, 0.01); // Setea el spinbox de Td 0.01
  
  pinMode(entrada, INPUT);   
  pinMode(pin_motor, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(entrada), contarPulso, FALLING);
  noInterrupts();
  TCCR1A = 0b00000000;     // Todo apagado, modo normal registro A. Counter1 del ATMEGA2560
  TCCR1B = 0b00000000;     // Todo apagado, modo normal registro B. Coun ter1 del ATMEGA2560
  TCCR1B |= B00000100;     // Configuración de preescaler a 256 (BIT CS12)
  TIMSK1 |= B00000010;     // Habilitar interrupción por comparación usando el registro TIMSK1 (modo CTC)
  OCR1A = 62500/fs; 
  
  // INICIALIZACIÓN DE VARIABLES PARA EL PID 
  Setpoint_RPM = 0; 
  Input_RPM = 0;
  PID_Output = 0;
  error = 0;
  last_error = 0;
  prev_last_error = 0;
  last_output = 0;
  rpm = 0;
  analogWrite(pin_motor, 0);
  interrupts();
  HMI_init(); 
  kp = 0.1;
  Ti = 0.1;
  Td = 0.01;
}

void loop() {
  

  if(millis()-t_previo1>=100){
    
    // LEE EL VALOR DEL HMI
    slider_DutyCycle = HMI_get_value("slider", "slider1"); // Asigna el valor del slider1 a slider_DutyCycle
    kp_ui = HMI_get_value("spin_box", "spin_box1"); // Asigna el valor del spin_box1 a Kp_ui
    Ti_ui = HMI_get_value("spin_box", "spin_box2"); // Asigna el valor del spin_box2 a Ti_ui
    Td_ui = HMI_get_value("spin_box", "spin_box3"); // Asigna el valor del spin_box3 a Td_ui
     
    // CONVIERTYE EL VALOR DEL HMI EN SETPOINT EN RPM DESEADAS
    Setpoint_RPM = map (slider_DutyCycle,0,100, 0, 5000);
    t_previo1=millis();
    
    
  }
  // Asigna los valores leidos del spinbox a los parametros del PID
  float kp = kp_ui;
  float Td = Td_ui;
  float Ti = Ti_ui;
  
  if(millis()-t_previo>=100){
    t_previo=millis();

    // MANDA A GRAFICAR EL VALOR DE REFERENCIA, LAS RPM DEL MOTOR Y LA SALIDA DEL PID
    dtostrf(Setpoint_RPM, 7, 2, label2_text);   
    dtostrf(Input_RPM, 7, 2, label4_text);      
    Stone_HMI_Set_Text("label","label2",label2_text); 
    Stone_HMI_Set_Text("label","label4",label4_text); 
    STONE_push_series("line_series", "line_series1", Setpoint_RPM); 
    STONE_push_series("line_series", "line_series2", Input_RPM);
    STONE_push_series("line_series", "line_series3", map(PID_Output,0,255,0,5));  // SE MAPEA DE 0 A 5 PARA OBTENER UNA REFERENCIA DE VOLATJE
    Serial.println(kp); 
    Serial.println(Td); 
    Serial.println(Ti);  
  }
}


// Interrupción por TIMER1:
ISR(TIMER1_COMPA_vect){   
 
  TCNT1=0;       // Resetea el timer
   
  //  Calcula las RPM 
  rpm = (float(conteo_pulsos) * 60) * fs / (PULSOS_POR_REV);
  
  // Actualizar la entrada del PID
  Input_RPM = (double)rpm; 

  //  Se calcula el error 
  error = Setpoint_RPM - rpm;

  // Cálculo PID incremental 
  double a0 = kp * (1 + (T / Ti) + (Td / T));
  double a1 = -kp * (1 + (2 * Td / T));
  double a2 = kp * (Td / T);

  double delta_output = a0 * error + a1 * last_error + a2 * prev_last_error;
  PID_Output = last_output + delta_output;


  // Limitación de la salida del PID
  if (PID_Output > PID_Output_Max) PID_Output = PID_Output_Max;
  else if (PID_Output < PID_Output_Min) PID_Output = PID_Output_Min;
  
  // Actualiza el valor de voltaje en el motor
  analogWrite(pin_motor, PID_Output);

  // Actualiza las variables 
  prev_last_error = last_error;
  last_error = error;
  last_output = PID_Output;

  // Resetea los pulsos 
  conteo_pulsos=0;   
}

// Interrupción por Hardware para contar los pulsos del motor
void contarPulso() {
  conteo_pulsos++;   // Incrementar contador al detectar pulso
}