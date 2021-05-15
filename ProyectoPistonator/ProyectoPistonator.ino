#include <ColorConverterLib.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <NewPing.h>
#include <SoftwareSerial.h>


int portSignalDriverPulse = 4;
int portSignalDriverDir = 5;
int giraMotorDerecha = 7;
int giraMotorIzquierda = 6;
int activaMotor = 2;
String material = "";

int AnalogPin = 0;   // Sensor conectado a Analog 0
int LED_PRESION = 10;      // LED conectado a Pin 6 (PWM)
int ResRead = 0;       // La Lectura de la Resistencia por División de Tensión
int BrilloLED = 0;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);


const int velocidadGiroMotorDC = 82; // VELOCIDAD MOTOR CC
int posicionCaja = 1; // 0 -> izquierda, 1 -> centro, 2 -> derecha

// ultrasonidos y bluetooth

const int echo1 = 49;
const int trigger1 = 48;


const int echo2 = 50;
const int trigger2 = 51;
const int MaxDistance = 14;

int Sonar_Papel = 0;
int led1 = 44;

int Sonar_Metal = 0;
int led2 = 45;
bool llenoMetal = false;
bool llenoPapel = false;


NewPing sonar(trigger1, echo1, MaxDistance);
NewPing sonar2(trigger2, echo2, MaxDistance);

SoftwareSerial BT1(17, 16); // pines usados para bluetooth 7 y 8

void setup() {
  Serial.begin(9600);
  pinMode(portSignalDriverPulse, OUTPUT);
  pinMode(portSignalDriverDir, OUTPUT);
  pinMode(giraMotorIzquierda, OUTPUT);
  pinMode(giraMotorDerecha, OUTPUT);
  pinMode(activaMotor, OUTPUT);
  pinMode(LED_PRESION, OUTPUT);
  tcs.begin();
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  Serial.println("Levantando el modulo bluetooth");
  BT1.begin(9600);
  Serial1.begin(9600);
  


}

void loop() {

  delay(1000);
  Sonar_Papel = sonar2.ping_cm();
  delay(50);
  Sonar_Metal = sonar.ping_cm();

  if (Sonar_Metal <= 8 ) {
    if(!llenoMetal){
      digitalWrite(led2, HIGH);
      Serial1.println("Recipiente de metal al limite de su capacidad");
    }
    llenoMetal = true; 
  } else {
    if(llenoMetal){
      Serial1.println("Recipiente de metal ha sido vaciado");
    }
    llenoMetal = false;
    digitalWrite(led2, LOW);
  }
  
  if (Sonar_Papel <= 9) {
    if(!llenoPapel){
      digitalWrite(led1, HIGH);
      Serial1.println("Recipiente de papel al limite de su capacidad");
    }
    llenoPapel = true;
  } else {
    if(llenoPapel){
      Serial1.println("Recipiente de papel ha sido vaciado");
    }
    llenoPapel = false;
    digitalWrite(led1, LOW);
  }

  /* //esperar 50ms entre pings (29ms como minimo)
    Serial.print(Sonar_Papel); // obtener el valor en cm (0 = fuera de rango)
    Serial.println("cm 1");
    Serial.print(Sonar_Metal); // obtener el valor en cm (0 = fuera de rango)
    Serial.println("cm 2");*/

  char dato = Serial1.read();
  if (dato == 'O') {
    peso();
    if (ResRead <= 400) {
      color();
      if (material.equals("Rojo")) {
        if (llenoMetal) {
          digitalWrite(led2, HIGH);
          Serial1.println("Recipiente de metal al limite de su capacidad");
        } else {
          Metal();
        }

      } else {
        if (llenoPapel) {
          digitalWrite(led1, HIGH);
          Serial1.println("Recipiente de papel al limite de su capacidad");
        } else {
          Papel();
        }

      }
    }

  }

  if (dato == 'G') {
    peso();
  }

  if (dato == 'C') {
    color();
  }

  if (dato == 'S') {
    subirPrensa();
  }
  if (dato == 'B') {
    bajarPrensa();
  }
  if (dato == 'D') {
    moverMotorDerecha();
  }
  if (dato == 'I') {
    moverMotorIzquierda();
  }
  if (dato == 'M') {
    Metal();
  }
  if (dato == 'P') {
    Papel();
  }

  delay(1000);
}

void Metal() {
  bajarPrensa();
  delay(1000);
  subirPrensa();
  delay(3000);
  moverMotorIzquierda();
  delay(1000);
  moverMotorDerecha();
  delay(1000);

}

void Papel() {
  bajarPrensa();
  delay(1000);
  subirPrensa();
  delay(3000);
  moverMotorDerecha();
  delay(1000);
  moverMotorIzquierda();
  delay(1000);
}

void moverMotorDerecha() {
  digitalWrite(giraMotorIzquierda, LOW);
  digitalWrite(giraMotorDerecha, HIGH);
  analogWrite(activaMotor, velocidadGiroMotorDC);
  if (posicionCaja == 0) {
    Serial.println("caja en la izquierda");
    delay(9200);
  } else {
    Serial.println("caja en el centro");
    delay(10000);
  }
  posicionCaja = 2;
  pararMotor();
}
void moverMotorIzquierda() {
  digitalWrite(giraMotorIzquierda, HIGH);
  digitalWrite(giraMotorDerecha, LOW);
  analogWrite(activaMotor, velocidadGiroMotorDC);
  if (posicionCaja == 2) {
    Serial.println("caja en la derecha");
    delay(9800);
  } else {
    Serial.println("caja en el centro");
    delay(8800);
  }

  posicionCaja = 0;
  pararMotor();
}

void pararMotor() {
  digitalWrite(giraMotorIzquierda, LOW);
  digitalWrite(giraMotorDerecha, LOW);
  analogWrite(activaMotor, 0);
}
void bajarPrensa() {
  // Asignamos direccion HIGH derecha, LOW izquierda
  digitalWrite(portSignalDriverDir, HIGH); // HIGH derecha LOW izquierda
  delayMicroseconds(100);

  // bucle para ir a la derecha
  for (int i = 0; i < 8200; i++) {
    digitalWrite(portSignalDriverPulse, HIGH);
    delayMicroseconds(1000);
    digitalWrite(portSignalDriverPulse, LOW);
    delayMicroseconds(1000);
  }
}
void subirPrensa() {
  // Asignamos direccion HIGH derecha, LOW izquierda
  digitalWrite(portSignalDriverDir, LOW);
  delayMicroseconds(100);

  // bucle para ir a la izquierda
  for (int i = 0; i < 8200; i++) { //8200
    digitalWrite(portSignalDriverPulse, HIGH);
    delayMicroseconds(1000);
    digitalWrite(portSignalDriverPulse, LOW);
    delayMicroseconds(1000);
  }
}

void peso() {
  ResRead = analogRead(AnalogPin); // La Resistencia es igual a la lectura del sensor (Analog 0)
  Serial.print("Lectura Analogica = ");
  Serial.println(ResRead);

  BrilloLED = map(ResRead, 0, 1023, 0, 255);
  // Cambiar el rango de la lectura analógica (0-1023)
  // Utilizamos en analogWrite 8 bits (0-255) configurados en el map
  if (BrilloLED > 50) {
    analogWrite(LED_PRESION, BrilloLED);
  }else{
    analogWrite(LED_PRESION, 0);
  }

  delay(100); //Cien “ms” de espera
}

void color() {
  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false);
  delay(60); // Cuesta 50ms capturar el color
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);
  // Hacer rgb medición relativa
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  // Escalar rgb a bytes
  r *= 256; g *= 256; b *= 256;
  // Convertir a hue, saturation, value
  double hue, saturation, value;
  ColorConverter::RgbToHsv(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b), hue, saturation, value);
  // Mostrar nombre de color
  printColorName(hue * 360);
}

void printColorName(double hue)
{
  if (hue < 15)
  {
    Serial.println("Rojo");
    material = "Rojo";
  }
  else if (hue < 45)
  {
    Serial.println("Naranja");
    material = "Naranja";
  }
  else if (hue < 90)
  {
    Serial.println("Amarillo");
    material = "Amarillo";
  }
  else if (hue < 150)
  {
    Serial.println("Verde");
    material = "Verde";
  }
  else if (hue < 210)
  {
    Serial.println("Cyan");
    material = "Cyan";
  }
  else if (hue < 270)
  {
    Serial.println("Azul");
    material = "Azul";
  }
  else if (hue < 330)
  {
    Serial.println("Magenta");
    material = "Magenta";
  }
  else
  {
    Serial.println("Rojo");
    material = "Rojo";
  }
}
