/*********************************************************** 
  
          ASTROLABE EXPEDITIONS - PROGRAMME SENSOCEAN
  Sonde de mesures de temperature et de salitnité de surface
                www.astrolabe-expeditions.org

Composants : 
-----------
base : Carte ESP32 Firebeetle programmé avec arduino ide
Ecran Epaper 2.9"                                   - Connexion SPI
Carte SD                                            - connexion SPI
GPS  : neo6m                                        - Connexion Serie
Horloge RTC                                         - Connexion I2C
Conductivité : atlas scientifique EC + isolator     - Connexion I2C
Tempéraure : atlas scientifique RTD                 - Connexion I2C 
Lipobabysister                                      - Connexion I2C


Branchement : 
-------------
Ecran : BUSY -> 4, RST -> 25, DC -> 13, CS -> 0, CLK -> SCK(18), DIN -> MOSI(23), GND -> GND, 3.3V -> 3.3V,
SD Reader : CS -> 5; 

Remarque : attention pour les lib ecran : penser a changer les point cpp en .h

Etat du programme : 
-------------------
fonctionne ok - le 02/07/2023
Remplacement des fichiers multiple par un  unique fichier datalog.csv
version adaté au PCB V2.0
Allumage de tous les composants en début de boucle et on éteint tout à chaque fin de boucle

Pour la prochain version : 
------------------------
Syncro GPS et RTC
Utiliser les nouvelles librairies Atlas

***********************************************************/


// ---------------------   PARAMETRES MODIFIABLE DU PROGRAM    -----------------------------------
char versoft[] = "7.1";                 // version du code
String fichier_config = "/config.txt";  // nom du fichier de configuration
int delay_affichage_ecran = 4;          // en seconde

// --------------------     FIN DES PARAMETRES MODIFIABLES     -----------------------------------

// Library
// if epaper v1
//#include <GxEPD.h>                      // Epaper Screen
////#include <GxGDEW042T2/GxGDEW042T2.h>    // Epaper Screen 4.2" b/w
//#include <GxGDEH029A1/GxGDEH029A1.h>    // Epaper Screen 2.9" b/w
//#include <GxIO/GxIO_SPI/GxIO_SPI.h>     // epaper sceeen
//#include <GxIO/GxIO.h>                  // epaper screen

// if epaper v2
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>

#include <Fonts/FreeMonoBold9pt7b.h>   // font for epaper sreen
#include <Fonts/FreeMonoBold12pt7b.h>  // font for epaper sreen
#include <Fonts/FreeMonoBold18pt7b.h>  // font for epaper sreen
#include <Fonts/FreeMonoBold24pt7b.h>  // font for epaper sreen
#include <Wire.h>                      //enable I2C.
#include <DS3231.h>                    // Pour horloge RTC
#include "SPI.h"                       // pour connection ecran bus SPI
#include <SD.h>                        // pour carte SD
#include <SparkFunBQ27441.h>           // Batterie fuel gauge lipo babysister
#include <TinyGPS++.h>                 // Gps
#include <Adafruit_BMP280.h>           // capteur de pression température atmosphérique
#include <OneWire.h>                   // pour capteur ds18b20
#include <DallasTemperature.h>         // pour capteur ds18b20

//SPI pin definitions pour ecran epaper V1
//GxIO_Class io(SPI, /*CS=5*/ 0, /*DC=*/ 13, /*RST=*/ 25); // arbitrary selection of 17, 16  //SS remplacé par 0
//GxEPD_Class display(io, /*RST=*/ 25, /*BUSY=*/ 4); // arbitrary selection of (16), 4

// spi pin definitons pour ecran V2
GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(GxEPD2_290_T94_V2(/*CS=5*/ 0, /*DC=*/13, /*RST=*/25, /*BUSY=*/4));  // GDEM029T94, Waveshare 2.9" V2 variant


// définition pour les fonts ecran
const GFXfont* f1 = &FreeMonoBold9pt7b;
const GFXfont* f2 = &FreeMonoBold12pt7b;
const GFXfont* f3 = &FreeMonoBold18pt7b;
const GFXfont* f4 = &FreeMonoBold24pt7b;

// definition pour la fonction deepsleep de l'ESP32
#define uS_TO_S_FACTOR 1000000    // Conversion factor for micro seconds to seconds
RTC_DATA_ATTR int bootCount = 0;  // utile pour enregistrer un compteur dans la memoire rtc de l'ULP pour un compteur permettant un suivi entre chaque veille
//RTC_DATA_ATTR int filenumber = 0;      // nom de fichier pour le garder entre l'initialisation (1er boot) et les mesures (tout les autres boots)
int TIME_TO_SLEEP = 600;  // Durée d'endormissement entre 2 cycles complets de mesures (in seconds) par défault 600 (mais rédéfinie par le fichier config)
int nbrMes = 3;           // nombre de mesure de salinité et température par cycle par déafault 3 (mais redéfinie par le fichier config


// definition pour les carte Atlas
#define ecAddress 100
#define rtdAddress 102  //default I2C ID number for EZO RTD Circuit.
byte rtdCode = 0;       // Used to hold the I2C response code.
byte rtdInChar = 0;     // Used as a 1 byte buffer to store in bound bytes from the RTD Circuit.
char rtdData[20];       // We make a 20 byte character array to hold incoming data from the RTD circuit.
int rtdDelay = 600;     // Used to change the delay needed depending on the command sent to the EZO Class RTD Circuit. 600 by default. It is the correct amount of time for the circuit to complete its instruction.
byte ecCode = 0;        // Used to hold the I2C response code.
byte ecInChar = 0;      // Used as a 1 byte buffer to store in bound bytes from the EC Circuit.
char ecData[48];        // We make a 48 byte character array to hold incoming data from the EC circuit.
int ecDelay = 600;      // Used to change the delay needed depending on the command sent to the EZO Class EC Circuit. 600 par defaut. It is the correct amount of time for the circuit to complete its instruction.
char* ec;               // Char pointer used in string parsing.
char* tds;              // Char pointer used in string parsing.
char* sal;              // Char pointer used in string parsing.
char* sg;               // Char pointer used in string parsing.

String datachain;  // chaine de donnée texte de mesure
//int ecpin =12;
//int rtdpin=14;

// déclaration initiale des variables de la fonction cal_sal (Aminot A., Kérouel R. (2004), Hydrologie des écosystèmes marins. Paramètres et analyses. Cf pages 74-78)
int Rp = 1;
float a[] = { 0.0080, -0.1692, 25.3851, 14.0941, -7.0261, 2.7081 };
float b[] = { 0.0005, -0.0056, -0.0066, -0.0375, 0.0636, -0.0144 };
float c[] = { 0.6766097, 0.0200564, 0.0001104259, -6.9698E-07, 1.0031E-09 };
float k = 0.0162;
float rt, R_t, S, modA, modB;


//déclaration pour horloge
DS3231 Clock;
bool Century = false;
bool h12, PM;
String second, minute, hour, date, month, year;
String datenum, timenum, fulltime, fulldate;  // pour format de date en 1 seule écriture

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 3400;  // e.g. 3400mAh battery

// definition pour GPS
TinyGPSPlus gps;
HardwareSerial Serial_GPS(1);
//int gpspin=27;
unsigned long t0;
int gps_tempo_start = 0;
int gps_tempo = 0;

// déclaration pour capteur de pression (bmp280)
Adafruit_BMP280 bmp;  // I2C Interface
float temp_ext;
float pres_ext;

//déclaration pour capteur de température interne ds18b20
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float tempint;

//déclaration pour la gestion de fichier sur la carte SD
const int cspin_SD = 5;
//String filename, filename_temp, str_index, filetrans;
int ind;  // index vérification de fichier
int gps_timer = 0;
String id_logger, number_measures, delay_series, mode_instrum, clef_test, nom_bateau, nom_skipper, gps_1stDelay, gps_delay, mode_pompe, delay_pompe;
File confFile;
int pompe, tps_pompe;

String filename = "/datalog.csv";

String message = "";

// déclaration pour la pompe à eau
// Pour le moment la pompe à eau est branché sur la prise Vcc du GPS, il faut donc gérer le temps d'allumage de la pompe avec le temps d'allumage du GPS
const int pompepin = 14;  // anciennement 26

//alimentation 5v pour tout les composant 5v
const int En5v = 27;

void setup() {
  // ------------------        HERE IS NEEDED THE PARAMETER FOR THE ENTIER PROGRAMM   ------------------------------------------
  delay(500);
  //power on all component or
  pinMode(En5v, OUTPUT);
  digitalWrite(En5v, HIGH);
  pinMode(pompepin, OUTPUT);
  digitalWrite(pompepin, LOW);
  delay(500);

  Serial.begin(115200);                        // communication PC
  Serial_GPS.begin(9600, SERIAL_8N1, 16, 17);  // communication serie avec GPS
  Wire.begin();                                // for I2C communication
  setupBQ27441();                              // for Lipo Babysister sparkfun lipo fuelgauge
  display.init();                              // enable display Epaper
  delay(200);                                  //Take some time to open up the Serial Monitor and enable all things


  test_sd();
  lecture_RTC();  // lecture de l'horloge rtc
  Serial.println("Testing RTC...");
  Serial.println(fulltime);
  Serial.println(fulldate);


  affichageintro1();  // texte d'intro et cadre initiale
  delay(10 * 1000);
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);
  display.nextPage();  



  unsigned int soc = lipo.soc();        // Read state-of-charge (%)
  unsigned int volts = lipo.voltage();  // Read battery voltage (mV)

  message = "SOC: ";
  message += String(soc);
  message = "volts: ";
  message += String(volts);
  Serial.println(message);

  //Read meteo sensor (bmp280)
  mes_pressure();
  //Read internal temperature
  mes_temp_int();

  mesureRTD();
  mesureEC();
  float temp = atof(rtdData);
  float cond = atof(ecData)/1000;
  Serial.print(" Temp = "); Serial.println(temp);
  Serial.print(" Cond = "); Serial.println(cond);
  cal_sal(temp, cond);

  Serial.println("Testing GPS...");
  while (millis() < 1000000) {  // attends le delay imposer par config.txt, puis passe à la suite fix ou pas fix
    //Read the gps
    smartDelay(1000);
    Serial.print("...");
    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));
    // if fix ok, break the while loop
    if (gps.location.isUpdated()) {
      Serial.println("Ok GPS fix");
      break;
    }
    delay(500);
  }
  Serial.println("End");

  try_sleep();
}


void loop() {
  //never use because of the sleeping of the princess esp32
  // peut être à utiliser pour la fonction wifi ??
}



/*
 * *****************************        LISTE DES FONCTIONS UTILISEES DANS LE PROGRAMME            ***********************************
 */

void try_sleep() {
  // endormissement esp32 pour 30s
  Serial.println("Going to sleep");


  digitalWrite(En5v, LOW);  // extinction des composants
  delay(500);

  esp_sleep_enable_timer_wakeup(30 * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}


void affichageintro1() {  // texte intro à l'allumage
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);

  //titre
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f4);
  display.setCursor(25, 55);
  display.println("SENSOCEAN ");

  //version
  display.setFont(f1);
  display.setCursor(30, 90);
  display.print("Num Serie :");
  display.println(id_logger);
  display.setCursor(30, 110);
  display.print("Ver Soft  :");
  display.print(versoft);

  //display.update();        // pour ecran V1
  display.nextPage();  //  pour ecran V2
}


void lecture_RTC() {
  // lecture de l'horloge rtc
  int sec = Clock.getSecond();
  if (sec < 10) {
    second = String(0) + String(sec);
  } else {
    second = sec;
  }
  int minu = Clock.getMinute();
  if (minu < 10) {
    minute = String(0) + String(minu);
  } else {
    minute = minu;
  }
  int heure = Clock.getHour(h12, PM);
  if (heure < 10) {
    hour = String(0) + String(heure);
  } else {
    hour = heure;
  }
  int jour = Clock.getDate();
  if (jour < 10) {
    date = String(0) + String(jour);
  } else {
    date = jour;
  }
  int mois = Clock.getMonth(Century);
  if (mois < 10) {
    month = String(0) + String(mois);
  } else {
    month = mois;
  }
  year = Clock.getYear();
  datenum = "";
  datenum += year;
  datenum += month;
  datenum += date;
  timenum = "";
  timenum += hour;
  timenum += minute;
  timenum += second;
  fulldate = "";
  fulldate += year;
  fulldate += "/";
  fulldate += month;
  fulldate += "/";
  fulldate += date;
  fulltime = "";
  fulltime += hour;
  fulltime += ":";
  fulltime += minute;
  fulltime += ":";
  fulltime += second;
}

void mesureRTD() {
  Wire.beginTransmission(rtdAddress);   // Call the circuit by its ID number.
  Wire.write('r');                      // Transmit the command that was sent through the serial port.
  Wire.endTransmission();               // End the I2C data transmission.
  delay(rtdDelay);                      //wait the correct amount of time for the circuit to complete its instruction. 600ms by default.
  Wire.requestFrom(rtdAddress, 20, 1);  // Call the circuit and request 20 bytes (this may be more than we need)
  rtdCode = Wire.read();                // The first byte is the response code, we read this separately.
  switch (rtdCode) {                    // Switch case based on what the response code is.
    case 1:                             // Decimal 1.
      Serial.println("RTD Success");    // Means the command was successful.
      break;                            // Exits the switch case.
    case 2:                             // Decimal 2.
      Serial.println("RTD Failed");     // Means the command has failed.
      break;                            // Exits the switch case.
    case 254:                           // Decimal 254.
      Serial.println("RTD Pending");    // Means the command has not yet been finished calculating.
      break;                            // Exits the switch case.
    case 255:                           // Decimal 255.
      Serial.println("RTD No Data");    // Means there is no further data to send.
      break;                            // Exits the switch case.
  }
  byte i = 0;                  // Counter used for RTD_data array.
  while (Wire.available()) {   // Are there bytes to receive.
    rtdInChar = Wire.read();   // Receive a byte.
    rtdData[i] = rtdInChar;    // Load this byte into our array.
    i += 1;                    // Incur the counter for the array element.
    if (rtdInChar == 0) {      // If we see that we have been sent a null command.
      i = 0;                   // Reset the counter i to 0.
      Wire.endTransmission();  // End the I2C data transmission.
      break;                   // Exit the while loop.
    }
  }
  Serial.println(rtdData);  //print the data.
}


void mesureEC() {
  Wire.beginTransmission(ecAddress);   // Call the circuit by its ID number.
  Wire.write('r');                     // r for reading sensor
  Wire.endTransmission();              // End the I2C data transmission.
  delay(ecDelay);                      // Reading time needing, 600 by default
  Wire.requestFrom(ecAddress, 48, 1);  // Call the circuit and request 48 bytes (this is more than we need)
  ecCode = Wire.read();                // The first byte is the response code, we read this separately.
  byte i = 0;                          // Counter used for EC_data array.
  while (Wire.available()) {           // Are there bytes to receive.
    ecInChar = Wire.read();            // Receive a byte.
    ecData[i] = ecInChar;              // Load this byte into our array.
    i += 1;                            // Incur the counter for the array element.
    if (ecInChar == 0) {               // If we see that we have been sent a null command.
      i = 0;                           // Reset the counter i to 0.
      Wire.endTransmission();          // End the I2C data transmission.
      break;                           // Exit the while loop.
    }
  }
  switch (ecCode) {                  // Switch case based on what the response code is.
    case 1:                          // Decimal 1.
      Serial.println("EC Success");  // Means the command was successful.
      break;                         // Exits the switch case.
    case 2:                          // Decimal 2.
      Serial.println("EC Failed");   // Means the command has failed.
      break;                         // Exits the switch case.
    case 254:                        // Decimal 254.
      Serial.println("EC Pending");  // Means the command has not yet been finished calculating.
      break;                         // Exits the switch case.
    case 255:                        // Decimal 255.
      Serial.println("EC No Data");  // Means there is no further data to send.
      break;                         // Exits the switch case.
  }
  ec = strtok(ecData, ",");
  tds = strtok(NULL, ",");
  sal = strtok(NULL, ",");
  sg = strtok(NULL, ",");  // Let's pars the string at each comma.

  Serial.println(ecData);
  //  Serial.print("EC:");                //we now print each value we parsed separately.
  //  Serial.println(ec);                 //this is the EC value.
  //  Serial.print("TDS:");               //we now print each value we parsed separately.
  //  Serial.println(tds);                //this is the TDS value.
  //  Serial.print("SAL:");               //we now print each value we parsed separately.
  //  Serial.println(sal);                //this is the salinity value.
  //  Serial.print("SG:");                //we now print each value we parsed separately.
  //  Serial.println(sg);                 //this is the specific gravity.
}


void cal_sal(float t, float C) {  // fonction de calcul de la salinité
  // calcul intermédiaire
  int i;
  rt = 0;                    // réinitilisation entre chaque boucle
  for (i = 0; i < 5; i++) {  // calcul de rt
    rt += c[i] * pow(t, float(i));
  }

  R_t = C / (42.914 * rt);  // calcul de R_t

  modA = 0;
  modB = 0;                  // réinitilisation entre chaque boucle
  for (i = 0; i < 6; i++) {  // cal modA et modB
    modA += a[i] * pow(R_t, float(i / 2));
    modB += b[i] * pow(R_t, float(i / 2));
  }

  //cal salinité
  S = modA + ((t - 15) / (1 + k * (t - 15))) * modB;
  Serial.print("Salinité = ");
  Serial.println(S, 4);
}


void setupBQ27441(void) {
  // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!lipo.begin())  // begin() will return true if communication is successful
  {
    // If communication fails, print an error message and loop forever.
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
  }else{
  Serial.println("Connected to BQ27441!");
  }
}


static void smartDelay(unsigned long ms)  //fonction pour GPS
{
  unsigned long start = millis();
  do {
    while (Serial_GPS.available())
      gps.encode(Serial_GPS.read());
  } while (millis() - start < ms);
}


void mes_temp_int() {  // température interne du boitier
  sensors.begin();
  delay(500);
  sensors.requestTemperatures();  // Send the command to get temperatures
  tempint = sensors.getTempCByIndex(0);
  Serial.print("Temperature : ");
  Serial.println(tempint, 3);  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
}


void mes_pressure() {  // capteur meteo, température et pression de l'air
  // init sensor
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  delay(600);

  temp_ext = bmp.readTemperature();
  pres_ext = bmp.readPressure() / 100;  // pressure in Hpa
  Serial.print("pres ext : ");
  Serial.println(pres_ext);
  Serial.print("temp ext : ");
  Serial.println(temp_ext);
}


void veille_ezos() {  //mise en veille du capteur aprés utilisation (la fonction "r" permettra de reveiller et lire le capteur).
                      //veille temperature
  delay(800);
  Wire.beginTransmission(rtdAddress);  // Call the circuit by its ID number.
  Wire.write("Sleep");                 // Transmit the command that was sent through the serial port.
  Wire.endTransmission();              // End the I2C data transmission.
                                       //veille EC
  delay(100);
  Wire.beginTransmission(ecAddress);  // Call the circuit by its ID number.
  Wire.write("Sleep");                // Transmit the command that was sent through the serial port.
  Wire.endTransmission();             // End the I2C data transmission.
}

void test_sd() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(cspin_SD)) {  // // see if the card is present and can be initialized, ajouter ici chipSelect ou 5 pour la pin 5 par default
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1)
      ;
  }
  Serial.println("card initialized.");
}
