#include <motorPID.h>
#include "BluetoothSerial.h"

// Verifica si la configuración de Bluetooth está habilitada en el ESP32
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT; // Objeto para manejar Bluetooth

//// Tiempo de muestreo //////
unsigned long lastTime, sampleTime = 100;

///// COMUNICACION SERIAL ///////
String inputString = "";
bool stringComplete = false;
const char separador = ',';
const int dataLength = 3;     //Recibir 3 velocidades de referencia
double data[dataLength];

// Variables PWM
const int freq = 1000;  // Frecuencia en Hz
const int resolution = 10;  // Resolución en bits

int dutyCycle1 = 0;  // Ciclo de trabajo inicial para el motor 1
int dutyCycle2 = 0;  // Ciclo de trabajo inicial para el motor 2
int dutyCycle3 = 0;  // Ciclo de trabajo inicial para el motor 3

// Pines del Motor 1
const int motor1PinPWM = 21;  //Pin para mandar la señal PWM
const int motor1Pin1 = 23;  // Pin para controlar direccion
const int motor1Pin2 = 22;  // Pin para controlar direccion 
const int encoder1PinA = 18;  // Pin para la señal A del encoder
const int encoder1PinB = 19;  // Pin para la señal B del encoder

/// Variables para conteo Encoder////  Cuenta de 1980 = 4 (resolucion) * 11 (ppr) * 45(ratio engranes)
volatile int n1 = 0; 
volatile int antA1 = 0;
volatile int antB1 = 0;
volatile int actA1 = 0;
volatile int actB1 = 0;

// Pines del Motor 2
const int motor2PinPWM = 5; 
const int motor2Pin1 = 16;
const int motor2Pin2 = 17; 
const int encoder2PinA = 14; 
const int encoder2PinB = 4;  

//vARIABLES ENCODER 2
volatile int n2 = 0; 
volatile int antA2 = 0;
volatile int antB2 = 0;
volatile int actA2 = 0;
volatile int actB2 = 0;

// Pines del Motor 3
const int motor3PinPWM = 32;  
const int motor3Pin1 = 25;  
const int motor3Pin2 = 33; 
const int encoder3PinA = 27; 
const int encoder3PinB = 26;

//VARIABLES ENCODER 3
volatile int n3 = 0; 
volatile int antA3 = 0;
volatile int antB3 = 0;
volatile int actA3 = 0;
volatile int actB3 = 0;

///// Variables para velocidad ////// Velocidad maxima 10.0 rad/s
// VELOCIDADES REALES
double w1 = 0;
double w2 = 0;
double w3 = 0;
const double constValue = 3.1733; // (1000 * 2 * pi)/R ---> R=1980 resolucion Encoder cuadruple

// VELOCIDADES DESEADEADAS (ROBOT)
double Vx = 0; 
double Vy = 0;
double Wa = 0; 

// VELOCIDADES DESEADAS (RUEDAS)
double VM1 = 0;
double VM2 = 0;
double VM3 = 0;

// VElocidades limite
double vMax = 10.5;
double vMin = 0.0;
double pwmMin = 0;
double pwmMax = 1023;

//////////////////////PID//////////////////////////////////////////
// PID parameters
double Kd = 0.40;
double Ti = 0.15;
double Td = 0.04;

/// Control del Motor//////
motorPID motor1(Kd, Ti, Td, sampleTime, vMax, vMin, pwmMax, pwmMin);
motorPID motor2(Kd, Ti, Td, sampleTime, vMax, vMin, pwmMax, pwmMin);
motorPID motor3(Kd, Ti, Td, sampleTime, vMax, vMin, pwmMax, pwmMin);

/////////////////////ROBOT////////////////////////////////////////
// Variables relacionadas al robot DEBEN DE ESTAR EN METROS Y RAD
const float L = 10;    //10 cm del centro del robot a la rueda
const float r = 2.3;   //radio de la llanta 2.3cm
const float Alpha = 30 * (M_PI / 180.0);  //Angulo alpha 30° a radianes

void setup() {
    //Comunicacion serial
  Serial.begin(115200);

  //Bluetooth
  SerialBT.begin("ESP32"); //Bluetooth device name
  Serial.println("Bluetooth iniciado");

  // Motor 1
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor1PinPWM, OUTPUT);

  // Motor 2
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor2PinPWM, OUTPUT);

  // Motor 3
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor3PinPWM, OUTPUT);

  // Configura las características del PWM para cada motor
  ledcSetup(0, freq, resolution);
  ledcSetup(1, freq, resolution);
  ledcSetup(2, freq, resolution);

  // Asocia los pines del motor con los canales PWM
  ledcAttachPin(motor1PinPWM, 0);
  ledcAttachPin(motor2PinPWM, 1);
  ledcAttachPin(motor3PinPWM, 2);

  // Establece los ciclos de trabajo iniciales
  ledcWrite(0, dutyCycle1);
  ledcWrite(1, dutyCycle2);
  ledcWrite(2, dutyCycle3);

  // Encoder 1
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), encoder1, CHANGE);

  // Encoder 2
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), encoder2, CHANGE);

  // Encoder 1
  pinMode(encoder3PinA, INPUT);
  pinMode(encoder3PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder3PinA), encoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinB), encoder3, CHANGE);
}

void loop() {
  if (SerialBT.available()) {
    serialEventBT();
  }

  //Comunicacion serial
  if(stringComplete){
    for (int i = 0; i <dataLength; i++){
      int index = inputString.indexOf(separador);
      data[i] = inputString.substring(0,index).toDouble();
      inputString = inputString.substring(index+1);
    }

    Vx = antiWindup(data[0]); 
    Vy = antiWindup(data[1]);
    Wa = data[2];

    inputString = "";
    stringComplete = false;
  }

  //Ejecutar esto cada 100ms 
  if(millis()-lastTime >= sampleTime){  
    
    // Medir/Calcular Velocidad Real del motor
    //Calculo de velocidad en rad/s
    w1 = (constValue*n1) / (millis()-lastTime);
    w2 = (constValue*n2) / (millis()-lastTime);
    w3 = (constValue*n3) / (millis()-lastTime);

    //Actualizar last time
    lastTime = millis();

    // Resetear contadores
    n1 = 0;
    n2 = 0;
    n3 = 0; 
    
    //Calculo de velocidad de cada motor Relacion VLineales y Vangulares /// Setpoints
    double m1 = (2*L*Wa - Vx + sqrt(3)*Vy) / (2*r);
    double m2 = (2*L*Wa - Vx - sqrt(3)*Vy) / (2*r);
    double m3 = (L*Wa + Vx) / r;

    //Calculo de velocidad del robot a partir de cada llanta
    double vx = -(r*(w1 + w2 - 2*w3)) /3;
    double vy = (sqrt(3) *r* (w1-w2)) /3;
    double w = (r*(w1 + w2 + w3)) / (3*L);

    // Vx max --> 0.25,0
    // Vx max y Vy --> 0.25,0.14
    // Vx max --> 0.2,0
    // Vx max y Vy --> 0.2,0.17

    // Vy max --> 0,0.28,0
    // Vy max y Vx --> 0.02,0.28
    // Vy max --> 0,0.2,0
    // Vy max y Vx --> 0.15,0.2

    ////////  CONCLUSION -> Vmax en X y Y es de 0.18 m/s

    // Calculo del PID
    VM1 = motor1.PID(m1, w1);
    VM2 = motor2.PID(m2, w2);
    VM3 = motor3.PID(m3, w3);

    // Checar sentido de giro
    dutyCycle1 = direccion(VM1, motor1Pin1, motor1Pin2);
    dutyCycle2 = direccion(VM2, motor2Pin1, motor2Pin2);
    dutyCycle3 = direccion(VM3, motor3Pin1, motor3Pin2);

    // Actualiza el ciclo de trabajo de la señal PWM
    ledcWrite(0, dutyCycle1);  
    ledcWrite(1, dutyCycle2);
    ledcWrite(2, dutyCycle3);
    
    SerialBT.println(vx);
    //Serial.print(",");
    SerialBT.println(vy);
    //Serial.print(",");
    SerialBT.println(w);
  }
}

/////////////////COMUNICACION SERIAL/////////////////////////////////7
/*
void serialEvent(){
  while(Serial.available()){
    char inChar = (char)Serial.read();
    inputString += inChar;

    if(inChar == '\n'){
      stringComplete = true;
    }
  }
}*/

void serialEventBT() {
    while (SerialBT.available()) {
        char inChar = (char)SerialBT.read();
        inputString += inChar;

        if (inChar == '\n') {
            stringComplete = true;
        }
    }
}

//////////////////////////SENTIDO DE GIRO///////////////////////////
int direccion(int duty, int motorPin1, int motorPin2){
  if (duty >= 0 && duty <= 1023) {  //If positive turn to right
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);

  } else if (duty < 0 && duty >= -1023){ //If negative turn to left
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin1, LOW);
  }

  if (abs(duty)<100){
    duty = 0;
  }
  return abs(duty);
}

double antiWindup(double vel){
  ///Primero definir velocidades maximas
  double max = 25;
  if(vel > max){
    vel = max;
  }else if(vel < -max){
    vel = -max;
  }
  return vel;
}

/////////////////////////////////LEER ENCODERS//////////////////////////////////////
void encoder1(void){
  antA1 = actA1;
  antB1 = actB1;

  actA1 = digitalRead(encoder1PinA);
  actB1 = digitalRead(encoder1PinB);

  // Incrementos
  if(antA1 == 0 && antB1 == 0 && actA1 == 0 && actB1 == 1) n1++;
  if(antA1 == 0 && antB1 == 1 && actA1 == 1 && actB1 == 1) n1++;
  if(antA1 == 1 && antB1 == 0 && actA1 == 0 && actB1 == 0) n1++;
  if(antA1 == 1 && antB1 == 1 && actA1 == 1 && actB1 == 0) n1++;

  // Decrementos  
  if(antA1 == 0 && antB1 == 0 && actA1 == 1 && actB1 ==  0) n1--;
  if(antA1 == 0 && antB1 == 1 && actA1 == 0 && actB1 ==  0) n1--;
  if(antA1 == 1 && antB1 == 0 && actA1 == 1 && actB1 ==  1) n1--;
  if(antA1 == 1 && antB1 == 1 && actA1 == 0 && actB1 ==  1) n1--;
}

void encoder2(void){
  antA2 = actA2;
  antB2 = actB2;

  actA2 = digitalRead(encoder2PinA);
  actB2 = digitalRead(encoder2PinB);

  // Incrementos
  if(antA2 == 0 && antB2 == 0 && actA2 == 0 && actB2 == 1) n2++;
  if(antA2 == 0 && antB2 == 1 && actA2 == 1 && actB2 == 1) n2++;
  if(antA2 == 1 && antB2 == 0 && actA2 == 0 && actB2 == 0) n2++;
  if(antA2 == 1 && antB2 == 1 && actA2 == 1 && actB2 == 0) n2++;

  // Decrementos  
  if(antA2 == 0 && antB2 == 0 && actA2 == 1 && actB2 ==  0) n2--;
  if(antA2 == 0 && antB2 == 1 && actA2 == 0 && actB2 ==  0) n2--;
  if(antA2 == 1 && antB2 == 0 && actA2 == 1 && actB2 ==  1) n2--;
  if(antA2 == 1 && antB2 == 1 && actA2 == 0 && actB2 ==  1) n2--;
}

void encoder3(void){
  antA3 = actA3;
  antB3 = actB3;

  actA3 = digitalRead(encoder3PinA);
  actB3 = digitalRead(encoder3PinB);

  // Incrementos
  if(antA3 == 0 && antB3 == 0 && actA3 == 0 && actB3 == 1) n3++;
  if(antA3 == 0 && antB3 == 1 && actA3 == 1 && actB3 == 1) n3++;
  if(antA3 == 1 && antB3 == 0 && actA3 == 0 && actB3 == 0) n3++;
  if(antA3 == 1 && antB3 == 1 && actA3 == 1 && actB3 == 0) n3++;

  // Decrementos  

  if(antA3 == 0 && antB3 == 0 && actA3 == 1 && actB3 ==  0) n3--;
  if(antA3 == 0 && antB3 == 1 && actA3 == 0 && actB3 ==  0) n3--;
  if(antA3 == 1 && antB3 == 0 && actA3 == 1 && actB3 ==  1) n3--;

  if(antA3 == 1 && antB3 == 1 && actA3 == 0 && actB3 ==  1) n3--;
}
