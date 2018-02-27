/*
 * Projet MERITE  - Code fusées à eau
 * 
 * Objectifs :    - Gestion du parachute
 *                - Récuperation des données pour analyse ultérieure sur carte SD
 *                - Données à récupérer : Accelération - Vitesse - Position
 *                
 * Materiel : - Sparkfun 9DoF Razor IMU M0 (Accéleromètre - Gyroscope - Magnétomètre).
 *            - Sparkfun BME280 (Température - Pression - Humidité - Altitude).
 *            
 * Auteur(s) : Romain Rousseau
 * Dernière modification : 27/02/2018
 */

// -------- BIBLIOTEQUES -------- //

#include <SparkFunMPU9250-DMP.h>
#include <SparkFunBME280.h>

#include <SD.h>
#include <Servo.h>

#include <Adafruit_Sensor.h>

//#include "config.h"

// --------    DEBUG    -------- //

#define SerialPort SerialUSB

// --------  PINS/BRANCHEMENTS -------- //

const int buttonPin = 8;
const int addedLedPin = 10;
const int boardLedPin = 13;
const int motorPin = 9;
const int sdPin = 38;
const float seaLevelPressure = 1013.25;

// -------- DECLARATION DES COMPOSANTS -------- //
Servo parachute_motor;
MPU9250_DMP imu;        //Inertial Mesurement Unit (Centrale Inertielle)
BME280 bme;             //Capteur de pression, humidité, temperature

// -------- VARIABLES D'ACQUISITION -------- //

//TEMPS
float acquisitionTime, referenceTime;
bool first_update;

//ACCELEROMETRE - GYROSCOPE - MAGNETOMETRE
float accelX, accelY, accelZ = 0;
//float gyroX ,  gyroY,  gyroZ = 0;
//float magX  ,   magY,   magZ = 0;

//VITESSE ET POSITION
//float velocityX, velocityY, velocityZ, lastVelocityX, lastVelocityY, lastVelocityZ = 0;
//float posX, posY, posZ, lastPosX, lastPosY, lastPosZ = 0;

//ALTITUDE
//float altitude, altitudeReference, lastAltitude, diffAltitude = 0;

// -------- VARIABLES DE SAUVEGARDE/LOG -------- //

String logFileName;   // Nom du fichié modifié
String logFileBuffer; // Buffer for logged data. Max is set in config

const int maxOfLogFiles = 999;             // Nombre max de fichiers
const String logFilePrefix = "log";        // Prefix du fichier
const String logFileSuffix = "txt";        // Suffix du fichier
const int maxFileSize = 5000000;           // 5MB max
const int maxFileBufferSize = 1024;

// -------- FONCTION D'INITIALISATION -------- //
// N'est effectuée qu'une fois

void setup()
{
  //Ouverture des ports de debug
  SerialPort.begin(9600);
  delay(100);
  SerialPort.println("Debug ouvert :");

  //Initialisation des E/S
  hardwareIO();

  //On lance la procédure d'initialisation
  initProcedure();
}

// -------- FIN SETUP ------- //

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
    //On déclenche le parachute si acceleration selon X est inférieure à -5 m.s^-2

    if (accelX < -5)
    {
      //On ouvre le parachute
      parachute_motor.write(90);
      SerialPort.println("Parachute déployé");
      logString("Parachute Deployé \n");
      delay(100);
    }

   }

  //Si le parachute est déployé alors la LED clignote en attendant que l'on rappui sur le bouton
  if (!parachuteLocked())
  {
    //Si on appui sur le bouton, alors on relance une procédure d'initialisation
    if  (digitalRead(buttonPin) == HIGH)
    {
      //Stabilise la led pendant 2 secondes pour montrer que l'intialisation recommence
      digitalWrite(addedLedPin,HIGH);
      delay(2000);
      //Relance de l'initialisation
      initProcedure();
    }

    redLedBlink();
  }
}

// -------- FIN BOUCLE PRINCIPALE -------- //

// -------- FONCTION D'INITIALISATION DES ENTREES-SORTIES -------- //

void hardwareIO(void)
{
  //Initialisation de la led de la carte Adafruit
  pinMode(boardLedPin, OUTPUT);
  digitalWrite(boardLedPin, LOW);

  //Initialisation de la led de la carte beige
  pinMode(addedLedPin, OUTPUT);
  digitalWrite(addedLedPin, LOW);

  //Intialisation de la centrale inertielle
  pinMode(4, INPUT_PULLUP);

  //Initialisation du bouton
  pinMode(buttonPin, INPUT);

  //Initialisation du moteur
  parachute_motor.attach(motorPin);
}

// -------- FIN D'INITIALISATION DES ENTREES-SORTIES -------- //


// -------- PROCEDURE D'INITIALISATION DE LA FUSEE -------- //

void initProcedure(void)
{
  //On allume la LED pour indiquer qu'un traitement est en cours
  digitalWrite(addedLedPin, HIGH);

  //Verifie si une Carte SD est connectée et initialise le lecteur
  if ( SD.begin(sdPin) )
  {
    SerialPort.println("Carte SD detectée");
    logFileName = nextLogFile();
    SerialPort.print("Données sauvegardées sur :");
    SerialPort.println(logFileName);
  } else {
    SerialPort.println("Carte SD non detectée");
    //Allume la LED bleue de la carte en cas de problème
    digitalWrite(boardLedPin, HIGH);
  }

  //Verifie si la centrale inertielle est connectée et l'initialise
  if ( !initIMU() )
  {
    SerialPort.println("Erreur de connexion avec le MPU-9250. Redémarrer la carte.");
    //Allume la LED bleue de la carte en cas de problème
    digitalWrite(boardLedPin, HIGH);
  } else {
    SerialPort.println("Centrale Inertielle connectée");
  }

  //Initialisation du capteur BME_280
  //initBME();

  //Initialisation du moteur verouillant le parachute en position ouverte
  parachute_motor.write(90);

  SerialPort.println("Verouillez le parachute.");

  delay(1000);
  
  //Attend le verouillage du parachute
  while (!parachuteLocked())
  {
    //Tant qu'on a pas d'appui sur le bouton on fait clignoter la LED et on attend.
    while (digitalRead(buttonPin) == LOW) {
      redLedBlink();
    }

    //Verouille le parachute
    parachute_motor.write(25);
  }

  SerialPort.println("Placez la fusée dans le lanceur et rappuyez sur le bouton");

  //Stabilise la LED pour indiquer que l'appui a été pris en compte
  digitalWrite(addedLedPin, HIGH);
  delay(2000);

  //Fait clignoter la LED et attend l'appui sur le bouton pour calibrer les composants
  while (digitalRead(buttonPin) == LOW) {
    redLedBlink();
  }

  //Calibration
  setupHardware();

  SerialPort.println("La fusée est prete à etre lancée.");

  //Eteint la LED pour indiquer que tout est prêt
  digitalWrite(addedLedPin, LOW);
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
  bme.settings.tStandby = 0;

  //filtre de fréquence
  bme.settings.filter = 5;

  //Oversampling (1 echantillon en plus pour chaque mesure)
  bme.settings.tempOverSample = 2;
  bme.settings.pressOverSample = 2;
  bme.settings.humidOverSample = 1;

  delay(100); //On s'assure que le BMP a eu le temps de démarrer

  //On implémente cette configuration avec bme.begin
  if (!bme.begin())
  {
    SerialPort.println("Impossible de détecter le BME_280.");
    delay(100);
    
    SerialPort.println("Erreur de connexion avec le MPU-9250. Redémarrer la carte.");
    //Allume la LED bleue de la carte en cas de problème
    digitalWrite(boardLedPin, HIGH);
  } else {
    SerialPort.println("BME_280 connecté.");
  }

  delay(500);
}

// -------- FONCTION D'INITIALISATION DE LA CENTRALE INTERTIELLE -------- //
// Retourne vrai si le composant est détecté et initialisé , faux sinon

bool initIMU(void) 
{
  if (imu.begin() != INV_SUCCESS)
    return false;

  //Active le gyro, le magnetomètre et l'acceleromètre
  //imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  //Active l'acceleromètre
  imu.setSensors(INV_XYZ_ACCEL);

  //Configure la plage de donnée de l'accel et du gyro
  //imu.setGyroFSR(500);
  imu.setAccelFSR(2);

  //Configure le filtre basse fréquence
  imu.setLPF(50);

  //Configure le taux d'échantillonage pour l'acceleromètre, le magnetomètre et le gyroscope
  imu.setSampleRate(50);
  imu.setCompassSampleRate(10);
  
  //Configure les données qui vont etre envoyé dans le buffer FIFO
  imu.configureFifo(INV_XYZ_ACCEL);

  delay(1000);

  return true;
}

// -------- FONCTION DE TEST DE L'ETAT DU PARACHUTE -------- //
// Retourne vrai s'il est fermé, faux sinon

bool parachuteLocked(void) 
{
  if (parachute_motor.read() <= 40) 
  {
    return true;
  } else {
    return false;
  }
}

// -------- FONCTION CLIGNOTAGE DE LA LED ROUGE (CARTE BEIGE) -------- //

void redLedBlink(void) 
{
  static bool redLedState = false;
  digitalWrite(addedLedPin, redLedState);
  redLedState = !redLedState;
  delay(500);
}


// -------- FONCTION DE CALIBRAGE DES COMPOSANTS -------- //

void setupHardware(void) 
{
  //Led allumé pendant le calibrage
  digitalWrite(addedLedPin, HIGH);

  SerialPort.println("Calibration en cours...");

  // Recalibre le gyroscope après 8 secondes d'immobilité
  // imu.dmpBegin(DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO, 10);
  // delay(8000);

  //Remise à zéro des variables
  first_update  = true;
  acquisitionTime = referenceTime = 0;
  //lastAltitude    = diffAltitude  = altitudeReference = 0;
  accelX          = accelY        = accelZ            = 0;
  velocityX       = velocityY     = velocityZ         = 0;
  lastVelocityX   = lastVelocityY = lastVelocityZ     = 0;
  posX            = posY          = posZ              = 0;
  lastPosX        = lastPosY      = lastPosZ          = 0;

  //Recupère la température locale, permet de calibrer le BME280
  //bme.readTempC();
  //delay(50);

  //Altitude au moment du lancement
  //altitudeReference = bme.readFloatAltitudeMeters();

  SerialPort.println("Calibrage effectuée");

  delay(2000);

  //Eteint la LED
  digitalWrite(addedLedPin, LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////
//                   FIN DES ELEMENTS DE LA PROCEDURE D'INITALISATION                   //
//////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////
//                          ELEMENTS DE LA  BOUCLE PRINCIPALE                           //
//////////////////////////////////////////////////////////////////////////////////////////

// -------- FONCTION DE RECUPERATION DES DONNEES -------- //

void getData(void) {

  //fifoAvailable donne le nombre de bits dans le buffer FIFO
  if ( imu.fifoAvailable() > 0) 
  {

    //On actualise nos variables et affiche les resultats.
    updateData();
    printData();

  } else {
    delay(100);
  }

  //Autre solution
  /*
  if (imu.dataReady())
  {
    updateData();
    printData();
  }else{
  delay(500);
  }
  */
}

// -------- FONCTION D'ACTUALISATION DES DONNEES -------- //

void updateData(void) {
  
  //Mise à jour des données de l'acceleromètre et du gyroscope
  //imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

  //MAJ de tous les capteurs
  //imu.update();
  
  //Mise à jour des données de l'acceleromètre
  imu.update(UPDATE_ACCEL);

  //Recuperation des données

  //Temps
  if (first_update) {
  //Temps au moment du lancement
  referenceTime = imu.time;
  first_update = false;
  }
  acquisitionTime = (imu.time - referenceTime)/1000 ;

  //Données acceleromètre en g converties en m/s^-2
  accelX = imu.calcAccel(imu.ax) * 9.8 - 9.8;
  accelY = imu.calcAccel(imu.ay) * 9.8;
  accelZ = imu.calcAccel(imu.az) * 9.8;

  //Données gyroscope en degrés par secondes
  //gyroX = imu.calcGyro(imu.gx);
  //gyroY = imu.calcGyro(imu.gy);
  //gyroZ = imu.calcGyro(imu.gz);


  //Données magnetomètre en micro Tesla
  //magX = imu.calcMag(imu.mx);
  //magY = imu.calcMag(imu.my);
  //magZ = imu.calcMag(imu.mz);  
  
  /*
    //CALCULS DE POSITION

    //Vitesse = VitesseInitiale + Accel*t
    velocityX = lastVelocityX + accelX * acquisitionTime;
    velocityY = lastVelocityY + accelY * acquisitionTime;
    velocityZ = lastVelocityZ + accelZ * acquisitionTime;

    lastVelocityX = velocityX;
    lastVelocityY = velocityY;
    lastVelocityZ = velocityZ;

    //Postion = PositionInitiale + Vitesse*t
    posX = lastPosX + velocityX * acquisitionTime;
    posY = lastPosY + velocityY * acquisitionTime;
    posZ = lastPosZ + velocityZ * acquisitionTime;

    lastPosX = posX;
    lastPosY = posY;
    lastPosZ = posZ;

  */

  /*
  //Recuperation de l'altitude
  if (acquisitionTime > 5){
  altitude = bme.readFloatAltitudeMeters() - altitudeReference - 0.01*acquisitionTime;
  }else{
  altitude = bme.readFloatAltitudeMeters() - altitudeReference;
  }
  
  //Calcul de la différence d'altitude entre 2 mesures (toutes les 700 ms)
  diffAltitude = altitude - lastAltitude;
  lastAltitude = altitude;
  */
  
  delay(300);
}


// -------- FONCTION D'AFFICHAGE DES DONNEES SUR LE TERMINAL -------- //

void printData(void) {

  SerialPort.println("Time: " + String(acquisitionTime) + " s");
  SerialPort.println("Accel: " + String(accelX) + ", " + String(accelY) + ", " + String(accelZ) + " m/s^-2");
  SerialPort.println("Vit: " + String(velocityX) + ", " + String(velocityY) + ", " + String(velocityZ) + " m/s^-1");
  SerialPort.println("Pos: " + String(posX) + ", " + String(posY) + ", " + String(posZ) + " m");

  //SerialPort.println("Gyro: " + String(gyroX) + ", " + String(gyroY) + ", " + String(gyroZ) + " dps");
  
  //SerialPort.println("Altitude de réference: " + String(altitudeReference) + " m");
  //SerialPort.println("Delta Altitude: " + String(diffAltitude) + " m");
  //SerialPort.println("Altitude: " + String(altitude) + " m");
  
  SerialPort.print("Parachute : ");
  if (parachuteLocked())
  {
    SerialPort.println("Verrouillé");
  }else{
    SerialPort.println("Deployé");
  }
  SerialPort.println("--------------------------------");

}

//----------------------------------------------------------//
//                    GESTION DES LOGS                      //
//----------------------------------------------------------//

// -------- FONCTION DE CREATION DU NOM DU FICHIER DE SAUVEGARDE  -------- //
// On cherche parmis les fichiers de log le premier qui n'existe pas
// Retourne un null si on a atteint le max de fichiers

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

void logData(void) 
{
  String logLine = "";

  logLine += "Temps: " + String(acquisitionTime) + " s; \n" ;

  logLine += "Accel: " + String(accelX) + "; " + String(accelY) + "; " + String(accelZ) + " m/s^-2; \n";

  //logLine += "Vit: " + String(velocityX) + ", " + String(velocityY) + ", " + String(velocityZ) + " m/s^-1; \n";

  //logLine += "Pos: " + String(posX) + ", " + String(posY) + ", " + String(posZ) + " m; \n";

  //logLine += "Gyro: " + String(gyroX) + ", " + String(gyroY) + ", " + String(gyroZ) + " dps; \n";

  //logLine += "Altitude: " + String(altitude) + " m; \n";

  //logLine += "Diff_Alt: " + String(diffAltitude) + " m; \n";

  if (parachuteLocked())  
  {
    logLine += "Parachute verrouille; \n";
  }else{
    logLine += "Parachute ouvert; \n";
  }
  
  logString(logLine);
}

