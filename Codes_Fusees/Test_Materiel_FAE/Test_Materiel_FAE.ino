//Test des elements de la fusée

//Bibliotèques
#include <Servo.h>

//Déclaration des élements
Servo motor;

//Constantes
const int buttonPin = 8;
const int ledOnBoardPin = 13;
const int addedLedPin = 10;
const int motorPin = 9;

//Variables        
int pushed = 0;    

//Initialisation
void setup() {
  
//Ouverture d'un port de Debug
Serial.begin(9600);

//Declaration des pins
pinMode(ledOnBoardPin, OUTPUT);       //LED de la carte rouge
pinMode(addedLedPin, OUTPUT);         //LED de la carte beige
pinMode(buttonPin, INPUT);            //Bouton

//Test moteur
MotorTest();

//Test leds
LedTest(ledOnBoardPin);
LedTest(addedLedPin);

//Test Button
ButtonTest(addedLedPin);

}

//Programme qui tourne en continue
void loop() {

//Test Button
ButtonTest(addedLedPin);

}


//Test de fonctionnement du moteur.
//Devrait s'ouvrir et se fermer 2 fois.
void MotorTest() {

//Initialisation du moteur
motor.attach(motorPin);
delay(50);

for (int i=0 ; i<=1 ; i++) {
motor.write(90); //Ouvert
delay(1000);
motor.write(25); //Fermé
delay(5000);
}

}

//Test de fonctionnement des leds
//Devrait clignoter 3 fois
void LedTest (int ledPin) {

for (int i=0 ; i<=2 ; i++) {
digitalWrite(ledPin, LOW);  //Eteinte
delay(500);
digitalWrite(ledPin, HIGH); //Allumé
}

}

//Test de fonctionnement du bouton
void ButtonTest(int ledPin) {
  SerialPort.println(digitalRead(buttonPin));
}

  
