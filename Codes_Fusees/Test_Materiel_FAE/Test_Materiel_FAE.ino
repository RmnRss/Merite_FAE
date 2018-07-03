 /*
 * Projet MERITE  - Code de test des différents éléments de la carte accompagnant les cartes Sparkfun
 * 
 * Objectif : Verifier le fonctionnement des éléments suivants du PCB : 
 *              - Des deux LEDs (rouge et verte)
 *              - Du bouton
 *              - Du servomoteur
 *    
 * Auteur(s) : Romain Rousseau
 * Dernière modification : 03/07/2018
 */

//Bibliotèques
#include <Servo.h>

//Déclaration des élements
Servo motor;

//Constantes
const int buttonPin = 12;
const int redLedPin = 9;
const int greenLedPin = 8;
const int servoPin = 13;  

#define SerialPort SerialUSB

//Initialisation
void setup() 
{
  //Ouverture d'un port de Debug
  SerialPort.begin(9600);

  //Déclaration des entrées/sorties
  pinMode(redLedPin, OUTPUT);           //LED rouge
  pinMode(greenLedPin, OUTPUT);         //LED VERTE
  pinMode(buttonPin, INPUT);            //Bouton

  //Test moteur
  MotorTest();

  //Test des leds
  //Devrait les allumer
  digitalWrite(redLedPin, HIGH); 
  digitalWrite(greenLedPin, HIGH); 
}

//Programme qui tourne en continue
void loop()
{
  //Test du bouton
  //Devrait afficher : "Bouton" dans la console série à chaque appui"
  if (digitalRead(buttonPin) == 1)
  {
    SerialPort.println("Bouton");
  }else{
    
  }
}

//Test de fonctionnement du servomoteur.
//Devrait s'ouvrir et se fermer 2 fois.
void MotorTest() 
{
  //Initialisation du moteur
  motor.attach(servoPin);
  delay(50);

  for (int i=0 ; i<=1 ; i++) 
  {
    motor.write(90); //Ouvert
    delay(500);
    motor.write(25); //Fermé
    delay(1000);
  }
}

  
