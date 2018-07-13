 /*
 * Projet MERITE  - Code fusées à eau
 * 
 * Objectifs :    - Gestion du parachute
 *                - Récuperation des données pour analyse ultérieure sur carte SD
 *                - Données à récupérer : Accelération - Vitesse - Position
 *                
 * Materiel : - Sparkfun 9DoF Razor IMU M0 (ATSAMD21G18 - Accéleromètre - Gyroscope - Magnétomètre).
 *            - Sparkfun BME280 (Température - Pression - Humidité - Altitude).
 *            - Sparkfun ADXL377 (Accéleromètre 200 G)
 *            
 * Auteur(s) : Cyril Zwick, Justin Maréchal, Hugo Vaille, Romain Rousseau
 * Dernière modification : 03/07/2018
 */

// -------- BIBLIOTEQUES -------- //

#include <SparkFunMPU9250-DMP.h>
#include <SparkFunBME280.h>

#include <SD.h>
#include <Servo.h>

// --------    DEBUG    -------- //

#define SerialPort SerialUSB

// --------  PINS/BRANCHEMENTS -------- //

const int buttonPin = 8;
const int redLedPin = 10;
const int motorPin = 9;
const int sdPin = 38;

/*
const int buttonPin = 12;
const int redLedPin = 9;
const int greenLedPin = 8;
const int motorPin = 13;
const int sdPin = 38;
*/

// -------- DECLARATION DES COMPOSANTS -------- //

Servo servo_parachute;
MPU9250_DMP imu;        //Inertial Mesurement Unit (Centrale Inertielle)
BME280 bme;             //Capteur de pressure, humidité, temperature

// -------- VARIABLES D'ACQUISITION -------- //²

// TEMPS
float flightTime, launchTime, triggeringTime;
bool first_update;

// ALTITUDE
float altitude, altitudeRef, lastAltitude, diffAltitude = 0;

// TEMPERATURE
float temp;
float tempRef;

// PRESSION
float pressure;
float pressureRel;
float pressureRef;
float lastPressure;
float diffPressure;

// -------- VARIABLES DE SAUVEGARDE/LOG -------- //

String logFileName;                        // Nom du fichier modifié
const int maxOfLogFiles = 999;             // Nombre max de fichiers
const int maxFileSize = 5000000;           // Taille max de chaque fichier 5MB
const String logFilePrefix = "lancer_";    // Prefix du fichier
const String logFileSuffix = "csv";        // Suffix du fichier

// -------- FONCTION D'INITIALISATION -------- //
// N'est effectuée qu'une fois

void setup()
{
  //Ouverture du port de debug baud rate 9600
  SerialPort.begin(9600);
  delay(100);

  //Initialisation des E/S
  hardwareIO();

  //On lance la procédure d'initialisation
  initProcedure();
}

// -------- FIN DE L'INITIALISATION ------- //

// -------- BOUCLE PRINCIPALE -------- //

void loop()
{
  //Si le parachute est vérouillé
  if (parachuteLocked())
  {
    //Alors on récupère les données
    getData();
    
    //Alors on enregistre les données sur un fichier de la carte SD
    logData();
    
    //Test de déclenchement du parachute
    //On déclenche le parachute si on a une différence d'altitude négative de 5 cm et si on est à plus de 1m de hauteur
    if (diffAltitude <-0.05 && altitude > 1)
    { 
      //On ouvre le parachute
      servo_parachute.write(90);
      triggeringTime=flightTime;
      //digitalWrite(greenLedPin, LOW);
    }
   }

  //Si le parachute est déployé alors la LED clignote en attendant que l'on rappui sur le bouton
  if (!parachuteLocked())
  {
    //On continue l'acquisition et l'enregistrement des données 5 secondes après le déclenchement du parachute
    while(flightTime-triggeringTime < 5)
    {
      getData();
      logData();
    }
    
    //Si on appui sur le bouton, alors on relance une procédure d'initialisation
    if  (digitalRead(buttonPin) == HIGH)
    {
      //Stabilise la led pendant 2 secondes pour montrer que l'intialisation recommence
      digitalWrite(redLedPin,HIGH);
      delay(2000);
      //Relance de l'initialisation
      newLaunch();
    }

    redLedBlink();
  }
}

// -------- FIN BOUCLE PRINCIPALE -------- //

// -------- FONCTION D'INITIALISATION DES ENTREES-SORTIES -------- //

void hardwareIO(void)
{
  //Initialisation de la led verte
  //pinMode(greenLedPin, OUTPUT);
  //digitalWrite(greenLedPin, LOW);

  //Initialisation de la led rouge
  pinMode(redLedPin, OUTPUT);
  digitalWrite(redLedPin, LOW);

  //Intialisation de la centrale inertielle
  pinMode(4, INPUT_PULLUP);

  //Initialisation du bouton
  pinMode(buttonPin, INPUT);

  //Initialisation du servo
  servo_parachute.attach(motorPin);
}

// -------- FIN D'INITIALISATION DES ENTREES-SORTIES -------- //


// -------- PROCEDURE D'INITIALISATION DE LA FUSEE -------- //

void initProcedure(void)
{
  String firstLine = "Temps,Altitude,Pression,Parachute\n";
  
  //On allume la LED rouge et on éteint la verte pour indiquer qu'un traitement est en cours
  digitalWrite(redLedPin, HIGH);
  //digitalWrite(Pin, LOW);

  //Verifie si une Carte SD est connectée et initialise le lecteur + Premiere ligne tableur
  if ( SD.begin(sdPin) )
  {
    SerialPort.println("Carte SD detectée");
    logFileName = nextLogFile();
    logString(firstLine);
    displaySuccess();
  } else {
    SerialPort.println("Carte SD non detectée");
  }

  //Verifie si la centrale inertielle est connectée et l'initialise
  if ( !initIMU() )
  {
    SerialPort.println("Erreur de connexion avec le MPU-9250. Redémarrer la carte.");
  } else {
    SerialPort.println("Centrale Inertielle connectée");
    displaySuccess();
  }

  //Initialisation du capteur BME_280
  initBME();

  //Initialisation du servo verouillant le parachute en position ouverte
  servo_parachute.write(90);

  SerialPort.println("Verouillez le parachute.");

  delay(1000);
  
  //Attend le verouillage du parachute
  while (!parachuteLocked())
  {
    //Tant qu'on a pas d'appui sur le bouton on fait clignoter la LED et on attend.
    while (digitalRead(buttonPin) == LOW) 
    {
      redLedBlink();
    }

    //Verouille le parachute
    servo_parachute.write(25);
  }

  SerialPort.println("Placez la fusée dans le lanceur et rappuyez sur le bouton");

  //Stabilise la LED pour indiquer que l'appui a été pris en compte
  digitalWrite(redLedPin, HIGH);
  delay(2000);

  //Fait clignoter la LED et attend l'appui sur le bouton pour calibrer les composants
  while (digitalRead(buttonPin) == LOW) 
  {
    redLedBlink();
  }

  //Calibration
  setupHardware();

  SerialPort.println("La fusée est prete à etre lancée.");

  //Allume la LED verte et éteint la rouge pour indiquer qu'on est prêt à lancer
  //digitalWrite(greenLedPin, HIGH);
  digitalWrite(redLedPin, LOW);
}

//Procédure de relance.
void newLaunch(void)
{
  String firstLine = "Temps,Altitude,Pression,Parachute\n";
  
  //On allume la LED rouge et on éteint la verte pour indiquer qu'un traitement est en cours
  digitalWrite(redLedPin, HIGH);
  //digitalWrite(greenLedPin, LOW); 

  logFileName = nextLogFile();
  SerialPort.print("Données sauvegardées sur :");
  SerialPort.println(logFileName);
  logString(firstLine);
  
  //Verifie si la centrale inertielle est connectée et l'initialise
  if ( !initIMU() )
  {
    SerialPort.println("Erreur de connexion avec le MPU-9250. Redémarrer la carte.");
  } 
  else 
  {
    SerialPort.println("Centrale Inertielle connectée");
  }

  //Initialisation du capteur BME_280
  initBME();

  //Initialisation du servo verouillant le parachute en position ouverte
  servo_parachute.write(90);

  SerialPort.println("Verouillez le parachute.");

  delay(1000);
  
  //Attend le verouillage du parachute
  while (!parachuteLocked())
  {
    //Tant qu'on a pas d'appui sur le bouton on fait clignoter la LED et on attend.
    while (digitalRead(buttonPin) == LOW) 
    {
      redLedBlink();
    }

    //Verouille le parachute
    servo_parachute.write(25);
  }

  SerialPort.println("Placez la fusée dans le lanceur et rappuyez sur le bouton");

  //Stabilise la LED pour indiquer que l'appui a été pris en compte
  digitalWrite(redLedPin, HIGH);
  delay(2000);

  //Fait clignoter la LED et attend l'appui sur le bouton pour calibrer les composants
  while (digitalRead(buttonPin) == LOW) 
  {
    redLedBlink();
  }

  //Calibration
  setupHardware();

  SerialPort.println("La fusée est prete à etre lancée.");

  //Allume la LED verte et éteint la rouge pour indiquer qu'on est prêt à lancer
  //digitalWrite(greenLedPin, HIGH);
  digitalWrite(redLedPin, LOW);
}

// -------- FIN DE LA PROCEDURE D'INITIALISATION DE LA FUSEE -------- //

//////////////////////////////////////////////////////////////////////////////////////////
//                        ELEMENTS DE LA PROCEDURE D'INITALISATION                      //
//////////////////////////////////////////////////////////////////////////////////////////

// -------- FONCTION D'INITIALISATION DE LA CARTE BME 280 -------- //

void initBME(void)
{
  //Le BME est branché sur les port I²C
  //Adresse I²C par defaut est 0x77
  bme.settings.commInterface = I2C_MODE;
  bme.settings.I2CAddress = 0x77;

  //Mode de fonctionnement normal
  bme.settings.runMode = 3;

  //tStandby 0 correspond a 0.5 ms d'attente
  //tStandby 6 correspond a 10 ms d'attente
  
  bme.settings.tStandby = 0;

  //filtre de fréquence
  bme.settings.filter = 2;

  //Oversampling (1 echantillon en plus pour chaque mesure)
  bme.settings.tempOverSample = 2;
  bme.settings.pressOverSample = 5;
  bme.settings.humidOverSample = 1;

  delay(500); //On s'assure que le BME a eu le temps de démarrer

  //On implémente cette configuration avec bme.begin
  if (!bme.begin())
  {
    SerialPort.println("Impossible de détecter le BME_280.");
    delay(100);
    SerialPort.println("Erreur de connexion avec le BME_280. Redémarrer la carte.");
  } else {
    SerialPort.println("BME_280 connecté.");
    displaySuccess();
  }

  delay(500);
}

// -------- FONCTION D'INITIALISATION DE LA CENTRALE INTERTIELLE -------- //
// Retourne vrai si le composant est détecté et initialisé , faux sinon

bool initIMU(void) 
{
  if (imu.begin() != INV_SUCCESS)
    return false;

  delay(1000);

  return true;
}

// -------- FONCTION DE TEST DE L'ETAT DU PARACHUTE -------- //
// Retourne vrai s'il est fermé, faux sinon

bool parachuteLocked(void) 
{
  if (servo_parachute.read() <= 50) 
  {
    return true;
  } else {
    return false;
  }
}

// -------- FONCTIONS DES LEDS -------- //

void redLedBlink(void) 
{
  static bool redLedState = false;
  digitalWrite(redLedPin, redLedState);
  redLedState = !redLedState;
  delay(500);
}

void displaySuccess(void)
{
    digitalWrite(redLedPin, HIGH);
    //digitalWrite(greenLedPin, HIGH);
    delay(500);
    digitalWrite(redLedPin, LOW);
    //digitalWrite(greenLedPin, LOW);
}

// -------- FONCTION DE CALIBRAGE DES COMPOSANTS -------- //

void setupHardware(void) 
{
  // Led rouge allumé pendant le calibrage
  digitalWrite(redLedPin, HIGH);
  //digitalWrite(greenLedPin, LOW);

  first_update  = true;
  flightTime = launchTime = 0;
  
  lastAltitude = diffAltitude = 0;
  lastPressure = diffPressure = 0;

  // Temperature de reference convertie en Kelvin
  tempRef = bme.readTempC() + 273,15; 
  delay(500);

  // Tant que la lecture de la pression ne donne pas une valeur cohérente on réinitialise le BME280
  while (bme.readFloatPressure() < 80000)
  {
    initBME();
    delay(2000);
  }

  pressureRef = bme.readFloatPressure()/101325.0000;
  altitudeRef = 2*1006*(tempRef/(-7*9.81))*log(pressureRef);

  SerialPort.println("Calibrage effectuée");
  
  delay(2000);
}

//////////////////////////////////////////////////////////////////////////////////////////
//                   FIN DES ELEMENTS DE LA PROCEDURE D'INITALISATION                   //
//////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////
//                          ELEMENTS DE LA  BOUCLE PRINCIPALE                           //
//////////////////////////////////////////////////////////////////////////////////////////

// -------- FONCTION DE RECUPERATION DES DONNEES -------- //

void getData(void) 
{
  if (imu.dataReady())
  {
    updateData();
    printData();
  }
}

// -------- FONCTION D'ACTUALISATION DES DONNEES -------- //

void updateData(void) 
{ 
  SerialPort.println("Updating Data");

  // ---- RECUPERATION DES DONNEES ---- //

  imu.update();
  
  // Temps de vol
  if (first_update) 
  {
    // Temps au moment du lancement
    launchTime = imu.time;
    first_update = false;
  }

  flightTime = (imu.time - launchTime) / 1000;
  
  // Temperature
  temp = bme.readTempC() + 273,15;
  
  // Recuperation de l'altitude
  pressure = bme.readFloatPressure();
  altitude = 2 * 1006 * (temp/(-7 * 9.81)) * log(pressure/101325.0) - altitudeRef;
  
  if( (altitude < -1 || altitude > 1) && flightTime < 2)
  { 
    pressureRef = bme.readFloatPressure() / 101325.0000;
    altitudeRef = 2*1006 * (temp/(-7 * 9.81)) * log(pressureRef);
    altitude = 0;
  }
  
  // Calcul de la différence d'altitude entre 2 mesures
  diffAltitude = altitude - lastAltitude;
  lastAltitude = altitude;

  // Calcul la difference de pression
  diffPressure = pressure - lastPressure;
  lastPressure = pressure;
}


// -------- FONCTION D'AFFICHAGE DES DONNEES SUR LE TERMINAL SERIE -------- //

void printData(void) 
{
  SerialPort.println("Printing Data");

  SerialPort.println("Temps: "    + String(flightTime) + " s");
  SerialPort.println("Altitude: " + String(altitude)   + " m");
}

//----------------------------------------------------------//
//                    GESTION DES LOGS                      //
//----------------------------------------------------------//

// -------- FONCTION DE CREATION DU NOM DU FICHIER DE SAUVEGARDE  -------- //
// On cherche parmis les fichiers de log le premier qui n'existe pas
// Retourne null si on a atteint le max de fichiers

String nextLogFile(void)
{
  String filename;
  int logIndex = 0;

  for (int i = 0; i < maxOfLogFiles; i++)
  {
    //On construit une chaine de caractère telle que : logFilePrefix[logIndex].logFileSuffix
    filename = String(logFilePrefix);
    filename += String(logIndex);
    filename += ".";
    filename += String(logFileSuffix);

    //SD.exists recherche un fichier du nom de filename et retourne vrai s'il existe
    if (!SD.exists(filename))
    {
      //S'il n'existe pas de fichier ayant ce nom alors on peut l'utiliser
      return filename;
    }
    //Sinon on incrémente l'index pour verifier si le suivant existe
    logIndex++;
  }
 
  //Si tout les fichiers existent c'est qu'on a atteint la limite de fichiers
  return "";
}

// -------- FONCTION D'ECRITURE DANS UN FICHIER  -------- //
// Ecrit une chaine de caractère dans un fichier et renvoie le succes ou non

bool logString(String toLog)
{
  //Ouvre le fichier actuel
  File logFile = SD.open(logFileName, FILE_WRITE);

  //Securité
  //Si la chaine est trop grande pour la taille maximum du fichier alors on passe au suivant
  if (logFile.size() > (maxFileSize - toLog.length()))
  {
    logFileName = nextLogFile();
    logFile = SD.open(logFileName, FILE_WRITE);
  }

  // Si le fichier est ouvert alors écrire la chaine.
  if (logFile)
  {
    logFile.print(toLog);
    logFile.close();

    return true;
  }

  return false;
}

// -------- FONCTION D'ENREGISTREMENT DES DONNEES  -------- //

void logData() 
{
  SerialPort.println("Saving Data");

  String logLine = "";

  logLine += String(flightTime) + "," + String(altitude) + "," + String(pressure) + "," + String(parachuteLocked()) + "\n";
  
  logString(logLine);

  SerialPort.println("-----");
}
