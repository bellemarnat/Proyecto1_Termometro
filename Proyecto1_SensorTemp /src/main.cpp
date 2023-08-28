// Librerias
#include <Arduino.h>
#include "config.h"

// Definición de pines
#define servoPin 25 // PWM para el servo
#define buttonPin 15 // pin para el botón

// LEDs
#define ledRoja 12
#define ledAmarillo 26
#define ledVerde 14
int ledSelec = 0;

// Sensor
#define LM35 34 // Pin para el sensor

// Pines para los 4 digitos del display
#define D1 13
#define D2 33
#define D3 32

// Pines para los segmento del display
#define A 21
#define B 19
#define C 18
#define D 5
#define E 17
#define f 16
#define G 4
#define DP 2

const int FreqPWM = 50; // Frecuencia en Hz
const int canalServo = 0; 

bool botonEstado = false;
bool ultimobotonEstado = false;
unsigned long ultimotiempoRebote = 0;
unsigned long retrasoRebote = 50;
unsigned long tiempo = 0; 

//void handleMessage(AdafruitIO_Data *data);
float temperatura = 0.0;
int decenas = 0; 
int unidades = 0;
int decimales = 0;
int valor = 0; 
//AdafruitIO_Feed *tempcanal = io.feed("tempcanal");

void desplegar7seg(uint8_t digito);
void mostrarTemperatura(float temp);
void presionBoton();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  /*Serial.print("Conectando a Adafruit IO");
  io.connect();
  tempcanal->onMessage(handleMessage);

  // Esperar conexión
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  tempcanal->get();
  
  // Conexión exitosa
  Serial.println();
  Serial.println(io.statusText());
  estado = false;*/

  // Configurar el servo
  pinMode(servoPin, OUTPUT);
  ledcSetup(canalServo, FreqPWM, 8); // Resolución: 8 bits
  ledcAttachPin(servoPin, canalServo);

  // Configurar el botón
  pinMode(buttonPin, INPUT_PULLUP);

  // Configurar los LEDs
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAmarillo, OUTPUT);
  pinMode(ledRoja, OUTPUT);

  // Configurar pines para los segmentos del display de 7 segmentos
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(D, OUTPUT);
  pinMode(E, OUTPUT);
  pinMode(f, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(DP, OUTPUT);
  
  // Configurar pines para el display de 4 dígitos de 7 segmentos
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  //Serial.print("Conectando a Adafruit IO...");
}

void loop() {
  //io.run();
  // Leer el estado del botón y aplicar debounce
  int lectura = digitalRead(buttonPin);
  if (lectura != ultimobotonEstado) {
    ultimotiempoRebote = millis();
  }
  if ((millis() - ultimotiempoRebote) > retrasoRebote) {
    if (lectura != botonEstado) {
      botonEstado = lectura;
      if (botonEstado == LOW) {
        presionBoton();
      }
    }
  }

  // Mostrar la parte de las decenas de la parte entera en el primer dígito
  desplegar7seg(D1);
  digitalWrite(D1, HIGH); // Activar primer dígito
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  desplegar7seg(decenas);
  tiempo = millis();
  while (millis()< tiempo +5);

  // Mostrar la parte de las unidades de la parte entera en el segundo dígito
  desplegar7seg(D2);
  digitalWrite(DP, HIGH); // Encender el punto decimal
  digitalWrite(D1, LOW);
  digitalWrite(D2, HIGH); // Activar segundo dígito
  digitalWrite(D3, LOW);
  desplegar7seg(unidades);
  tiempo = millis();
  while (millis()< tiempo +5);

  // Mostrar la parte decimal en el tercer dígito
  desplegar7seg(D3);
  digitalWrite(DP, LOW); // Apagar el punto decimal
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(D3, HIGH); // Activar tercer dígito
  desplegar7seg(decimales);
  tiempo = millis();
  while (millis()< tiempo +5);
  
  ultimobotonEstado = lectura;
}

void desplegar7seg(uint8_t digito) {
  decenas = temperatura/10;
  unidades = temperatura - decenas*10;
  decimales = (temperatura*10) - (decenas*100) - (unidades*10); 

  switch (digito) {
    case 0:
      digitalWrite(A, HIGH);
      digitalWrite(B, HIGH);
      digitalWrite(C, HIGH);
      digitalWrite(D, HIGH);
      digitalWrite(E, HIGH);
      digitalWrite(f, HIGH);
      digitalWrite(G, LOW);
      break;
    case 1:
      digitalWrite(A, LOW);
      digitalWrite(B, HIGH);
      digitalWrite(C, HIGH);
      digitalWrite(D, LOW);
      digitalWrite(E, LOW);
      digitalWrite(f, LOW);
      digitalWrite(G, LOW);
      break;
    case 2:
      digitalWrite(A, HIGH);
      digitalWrite(B, HIGH);
      digitalWrite(C, LOW);
      digitalWrite(D, HIGH);
      digitalWrite(E, HIGH);
      digitalWrite(f, LOW);
      digitalWrite(G, HIGH);
      break;
    case 3:
      digitalWrite(A, HIGH);
      digitalWrite(B, HIGH);
      digitalWrite(C, HIGH);
      digitalWrite(D, HIGH);
      digitalWrite(E, LOW);
      digitalWrite(f, LOW);
      digitalWrite(G, HIGH);
      break;
    case 4:
      digitalWrite(A, LOW);
      digitalWrite(B, HIGH);
      digitalWrite(C, HIGH);
      digitalWrite(D, LOW);
      digitalWrite(E, LOW);
      digitalWrite(f, HIGH);
      digitalWrite(G, HIGH);
      break;
    case 5:
      digitalWrite(A, HIGH);
      digitalWrite(B, LOW);
      digitalWrite(C, HIGH);
      digitalWrite(D, HIGH);
      digitalWrite(E, LOW);
      digitalWrite(f, HIGH);
      digitalWrite(G, HIGH);
      break;
    case 6:
      digitalWrite(A, HIGH);
      digitalWrite(B, LOW);
      digitalWrite(C, HIGH);
      digitalWrite(D, HIGH);
      digitalWrite(E, HIGH);
      digitalWrite(f, HIGH);
      digitalWrite(G, HIGH);
      break;
    case 7:
      digitalWrite(A, HIGH);
      digitalWrite(B, HIGH);
      digitalWrite(C, HIGH);
      digitalWrite(D, LOW);
      digitalWrite(E, LOW);
      digitalWrite(f, LOW);
      digitalWrite(G, LOW);
      break;
    case 8:
      digitalWrite(A, HIGH);
      digitalWrite(B, HIGH);
      digitalWrite(C, HIGH);
      digitalWrite(D, HIGH);
      digitalWrite(E, HIGH);
      digitalWrite(f, HIGH);
      digitalWrite(G, HIGH);
      break;
    case 9:
      digitalWrite(A, HIGH);
      digitalWrite(B, HIGH);
      digitalWrite(C, HIGH);
      digitalWrite(D, HIGH);
      digitalWrite(E, LOW);
      digitalWrite(f, HIGH);
      digitalWrite(G, HIGH);
      break;
    default:
      break;
  }
}

void mostrarTemperatura(float temp) {
  temperatura = temp; // Asignar la temperatura recibida a la variable global
  
  // Mostrar la parte de las decenas, unidades y decimales en los dígitos correspondientes
  desplegar7seg(D1);
  digitalWrite(D1, HIGH);
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);
  desplegar7seg(decenas);
  delay(5);
  
  desplegar7seg(D2);
  digitalWrite(DP, HIGH);
  digitalWrite(D1, LOW);
  digitalWrite(D2, HIGH);
  digitalWrite(D3, LOW);
  desplegar7seg(unidades);
  delay(5);
  
  desplegar7seg(D3);
  digitalWrite(DP, LOW);
  digitalWrite(D1, LOW);
  digitalWrite(D2, LOW);
  digitalWrite(D3, HIGH);
  desplegar7seg(decimales);
  delay(5);
}

void presionBoton() {
  int valorCrudo = analogRead(LM35);
  float tempC = (valorCrudo * 3.3 / 4095.0 - 0.5) * 100.0; // Fórmula de conversión
  int angulo = 0;

  Serial.print("Valor Crudo: ");
  Serial.println(valorCrudo);
  
  Serial.print("Temperatura (C): ");
  Serial.println(tempC);

  if (tempC >= -50.0) {
    tempC = (valorCrudo * 3.3 / 4095.0) * 100.0; // Nueva fórmula de conversión para temperaturas positivas
  }
  // Determinar el ángulo según la temperatura
  if (tempC < 36.0) {
    analogWrite(ledVerde, 255);
    analogWrite(ledAmarillo, 0);
    analogWrite(ledRoja, 0);
    angulo = 0; // Establecer ángulo en 0°
  } else if (tempC >= 36.5 && tempC < 37.5) {
    analogWrite(ledVerde, 0);
    analogWrite(ledAmarillo, 255);
    analogWrite(ledRoja, 0);
    angulo = 90; // Establecer ángulo en 90°
  } else {
    analogWrite(ledVerde, 0);
    analogWrite(ledAmarillo, 0);
    analogWrite(ledRoja, 255);
    angulo = 180; // Establecer ángulo en 180°
  }

  Serial.print("Temperatura (˚C): ");
  Serial.println(tempC);

  // Mapear el ángulo al ciclo de trabajo PWM según el ángulo deseado
  int cicloTrabajo = map(angulo, 0, 180, 102, 716); // Mapear 0-180° al rango de ciclo de trabajo PWM

  // Establecer ciclo de trabajo PWM para el control del servo
  ledcWrite(canalServo, cicloTrabajo);

  // Mostrar temperatura en el display de 4 dígitos de 7 segmentos
  mostrarTemperatura(tempC);

}

/*void handleMessage(AdafruitIO_Data *data) {
  Serial.print("Recibido <- ");
  Serial.println(data->value());

  if (*data->value() == '1') {
    estado = true;
  } else {
    estado = false;
  }
}*/