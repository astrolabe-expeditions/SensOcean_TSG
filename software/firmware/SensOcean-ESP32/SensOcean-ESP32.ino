/*********************************************************** 
  
          ASTROLABE EXPEDITIONS - PROGRAMME SENSOCEAN
  Sonde de mesures de temperature et de salitnité de surface
                www.astrolabe-expeditions.org

Composants : 
-----------
base : Carte ESP32 Firebeetle programmé avec arduino ide
Ecran Epaper 4"                                     - Connexion SPI
Carte SD                                            - connexion SPI
GPS  : neo6m                                        - Connexion Serie
Horloge RTC                                         - Connexion I2C
Conductivité : atlas scientifique EC + isolator     - Connexion I2C
Tempéraure : atlas scientifique RTD                 - Connexion I2C 
Lipobabysister                                      - Connexion I2C


Branchement : 
-------------
Ecran : BUSY -> 4, RST -> 25, DC -> 13, CS -> SS(5), CLK -> SCK(18), DIN -> MOSI(23), GND -> GND, 3.3V -> 3.3V,
SD Reader : CS -> 5; 

Remarque : attention pour les lib ecran : penser a changer les point cpp en .h

Etat du programme : 
-------------------
Le code fonctionne, mais : 
il y avait une reaction bizarre du boitier, qui ne marchait qu'apres avoir appuyer sur le bouton reset quand il était sur batterie.
Le bug vient du bloc "test de la carte SD", si le bloc test se trouve avant le grand if du setup, alors bug (no sd card), si le bloc test se trouve dans le if/Else, alors ca marche
bug reproductible y compris avec la version 1 du code.
Résolution du bug en placant le bloc test dans la boucle if/else (2 fois donc), mais sans comprendre pourquoi ca buguait.

Pour la prochain version : 
------------------------
Tester Gerer les alimentations/veille des composants
Ajouter les branchement
Changer le support de carte SD
Ajouter les seconds dans la datachain à enregistrer

***********************************************************/


// ---------------------   PARAMETRES MODIFIABLE DU PROGRAM    -----------------------------------
// Version et numero de serie
char numserie[] = "AESO19004";      // Numero de serie de la sonde
char versoft[] = "5.2-pcb";             // version du code

#define TIME_TO_SLEEP  10           // Durée d'endormissement entre 2 cycles complets de mesures (in seconds)
int nbrMes = 2;                     // nombre de mesure de salinité et température par cycle

// --------------------     FIN DES PARAMETRES MODIFIABLES     ------------------------------------


// Library
#include <GxEPD.h>                      // Epaper Screen
//#include <GxGDEW042T2/GxGDEW042T2.h>    // Epaper Screen 4.2" b/w
#include <GxGDEH029A1/GxGDEH029A1.h>    // Epaper Screen 2.9" b/w
#include <Fonts/FreeMonoBold9pt7b.h>    // font for epaper sreen
#include <Fonts/FreeMonoBold12pt7b.h>   // font for epaper sreen
#include <Fonts/FreeMonoBold18pt7b.h>   // font for epaper sreen
#include <Fonts/FreeMonoBold24pt7b.h>   // font for epaper sreen
#include <GxIO/GxIO_SPI/GxIO_SPI.h>     // epaper sceeen
#include <GxIO/GxIO.h>                  // epaper screen
#include <Wire.h>                       //enable I2C.
#include <DS3231.h>                     // Pour horloge RTC
#include "SPI.h"                        // pour connection ecran bus SPI
#include <SD.h>                         // pour carte SD
#include <SparkFunBQ27441.h>            // Batterie fuel gauge lipo babysister
#include <TinyGPS++.h>                  // Gps


//SPI pin definitions pour ecran epaper
GxIO_Class io(SPI, /*CS=5*/ 0, /*DC=13*/ 2, /*RST=25*/ 27); // arbitrary selection of 17, 16  //SS remplacé par 0
GxEPD_Class display(io, /*RST=25*/ 27, /*BUSY=4*/ 26); // arbitrary selection of (16), 4

// définition pour les fonts ecran
const GFXfont* f1 = &FreeMonoBold9pt7b;
const GFXfont* f2 = &FreeMonoBold12pt7b;
const GFXfont* f3 = &FreeMonoBold18pt7b;
const GFXfont* f4 = &FreeMonoBold24pt7b;

// definition pour la fonction deepsleep de l'ESP32
#define uS_TO_S_FACTOR 1000000      // Conversion factor for micro seconds to seconds
RTC_DATA_ATTR int bootCount = 0;    // utile pour enregistrer un compteur dans la memoire rtc de l'ULP pour un compteur permettant un suivi entre chaque veille


// definition pour les carte Atlas
#define ecAddress 100 
#define rtdAddress 102              //default I2C ID number for EZO RTD Circuit.
byte rtdCode=0;                     // Used to hold the I2C response code. 
byte rtdInChar=0;                   // Used as a 1 byte buffer to store in bound bytes from the RTD Circuit.   
char rtdData[20];                   // We make a 20 byte character array to hold incoming data from the RTD circuit. 
int rtdDelay = 600;                 // Used to change the delay needed depending on the command sent to the EZO Class RTD Circuit. 600 by default. It is the correct amount of time for the circuit to complete its instruction.
byte ecCode = 0;                    // Used to hold the I2C response code.
byte ecInChar = 0;                  // Used as a 1 byte buffer to store in bound bytes from the EC Circuit.
char ecData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
int ecDelay = 600;                  // Used to change the delay needed depending on the command sent to the EZO Class EC Circuit. 600 par defaut. It is the correct amount of time for the circuit to complete its instruction.
char *ec;                           // Char pointer used in string parsing.
char *tds;                          // Char pointer used in string parsing.
char *sal;                          // Char pointer used in string parsing.
char *sg;                           // Char pointer used in string parsing.

String datachain;                   // chaine de donnée texte de mesure                 
int ecpin =12;
int rtdpin=14;


//déclaration pour horloge
DS3231 Clock;
bool Century=false;
bool h12;
bool PM;
//byte ADay, AHour, AMinute, ASecond, ABits;
//bool ADy, A12h, Apm;
//byte year, month, date, DoW, hour, minute, second;
int second,minute,hour,date,month,year; 

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 3400; // e.g. 3400mAh battery

// definition pour GPS
TinyGPSPlus gps;                           
HardwareSerial Serial1(1);        


void setup()
{
  // ------------------        HERE IS NEEDED THE PARAMETER FOR THE ENTIER PROGRAMM   ------------------------------------------
  Serial.begin(115200);                        // communication PC
  Serial1.begin(9600, SERIAL_8N1, 16, 17);     // communication serie avec GPS
  Wire.begin();                                // for I2C communication
  setupBQ27441();                              // for Lipo Babysister sparkfun lipo fuelgauge
  display.init();                              // enable display Epaper
  delay(500); //Take some time to open up the Serial Monitor and enable all things


//  pinMode(rtdpin, OUTPUT);            // pin temperature
//  pinMode(ecpin, OUTPUT);            // pin EC
//  digitalWrite(rtdpin, LOW);   // temp
//  digitalWrite(ecpin, LOW);   // ec

  if(bootCount == 0) //Run this only the first time
  {
      // --------------        HERE IS ONLY THE INTRODUCION           -----------------------------------------------------------
      
      affichageintro();       // texte d'intro et cadre initiale
      Serial.println(" Phase d'introduction ");
      //delay(3000);

      // Test carte SD
      Serial.print("Initializing SD card...");   
        if (!SD.begin(5)) {                      // // see if the card is present and can be initialized, ajouter ici chipSelect ou 5 pour la pin 5 par default
        Serial.println("Card failed, or not present");
        errormessage();
        // don't do anything more:
        return;
      }
      Serial.println("card initialized.");   

      //Make the first line of datachain
      datachain += "lat" ; datachain += " ; "; datachain += "Lng" ; datachain += " ; "; 
      datachain += "Years"; datachain += " ; "; datachain += "Month" ; datachain += " ; "; datachain += "Day" ; datachain += " ; ";
      datachain += "Hour"; datachain += " ; "; datachain += "Minute"; datachain += " ; "; 
      datachain += "Bat %"; datachain += " ; "; datachain += "Bat mV"; datachain += " ; "; 
        for(int n=1; n<=nbrMes; n++){
          datachain += "Temp";
          datachain += " ; ";
        }
        for(int n=1; n<=nbrMes; n++){
          datachain += "EC";
          datachain += " ; ";
        }

     // afficher la datachain sur le port serie
      Serial.println(" Format de la chaine enregistrée ");
      Serial.println(datachain);

     // enregistrement de la datachain sur la carte SD
        File dataFile = SD.open("/datalog.txt", FILE_APPEND);
        if (dataFile) {                                        // if the file is available, write to it:
          dataFile.println(datachain);
          dataFile.close();
        }
        else {                                                 // if the file isn't open, pop up an error:
          Serial.println("error opening datalog.txt"); 
          errormessage();
          delay(3000);   
        }

      

      
      bootCount = bootCount+1;   // changement du numéro de compteur pour passer directement dans programm loop apres le reveil
      //Serial.println("Boot number: " + String(bootCount));  //affichage du numéro de boucle depuis que l'instrument est allumé
      
  }else
  {
      // ---------------        HERE IS THE MAIN PROGRAMM LOOP         ----------------------------------------------------------
      
//      digitalWrite(rtdpin, HIGH);   // temp  si allumage des pin
//      digitalWrite(ecpin, HIGH);   // ec
//      delay(2000);

      // Test carte SD
      Serial.print("Initializing SD card...");   
        if (!SD.begin(5)) {                      // // see if the card is present and can be initialized, ajouter ici chipSelect ou 5 pour la pin 5 par default
        Serial.println("Card failed, or not present");
        errormessage();
        // don't do anything more:
        return;
      }
      Serial.println("card initialized.");           

      // lecture de l'horloge rtc
      int second,minute,hour,date,month,year; 
      //second=Clock.getSecond();
      minute=Clock.getMinute();
      hour=Clock.getHour(h12, PM);
      date=Clock.getDate();
      month=Clock.getMonth(Century);
      year=Clock.getYear();
      
      // Read battery stats from the BQ27441-G1A
      unsigned int soc = lipo.soc();  // Read state-of-charge (%)
      unsigned int volts = lipo.voltage(); // Read battery voltage (mV)

      //Read the gps
      smartDelay(1000);  
      if (millis() > 5000 && gps.charsProcessed() < 10)
        Serial.println(F("No GPS data received: check wiring")); 

          
      // MESURE TEMPERATURE ET CONDUCTIVITE et construction de la chaine de donnée pour enregistrer sur la carte SD
      String datachain ="";

        // date heure GPS de debut de chaine
        datachain += gps.location.lat(); datachain += " ; " ;datachain += gps.location.lng(); datachain += " ; ";
        datachain += year; datachain += " ; ";datachain += month; datachain += " ; ";datachain += date; datachain += " ; ";
        datachain += hour; datachain += " ; ";datachain += minute; datachain += " ; ";
        datachain += soc; datachain += " ; "; datachain += volts ; datachain += " ; ";
                
                
        // température
        for(int n=1; n<=nbrMes; n++){
          mesureRTD(); 
          datachain += rtdData;
          datachain += " ; ";
        }
        
        // Salinité
        for(int n=1; n<=nbrMes; n++){
          mesureEC(); 
          datachain += ecData;
          datachain += " ; ";
        }

        // enregistrement sur la carte SD
          File dataFile = SD.open("/datalog.txt", FILE_APPEND);
          if (dataFile) {                                        // if the file is available, write to it:
            dataFile.println(datachain);
            dataFile.close();
          }
          else {                                                 // if the file isn't open, pop up an error:
            Serial.println("error opening datalog.txt");  
            errormessage();
            delay(3000);  
          }


      // Affichage ecran des datas
      Serial.print("datachain : "); Serial.println(datachain); //affichage de la chaine complete sur le port serie

          // calsal - fonction temporaire juste pour affichage d'une salinité plutot que conductivité
          float apar=0.0016, bpar=0.6318, cpar=0.5055; // pour un caclul pour une température de 22°
          //float apar=0.0002, bpar=0.7443, cpar=0.5697; // pour un caclul pour une température de 15°
          float xval=atof(ecData)/1000;
          float salfinal = apar*(xval*xval)+bpar*xval-cpar;  

      display.setRotation(3);
      display.fillScreen(GxEPD_WHITE);
      
      //data
      display.setTextColor(GxEPD_BLACK);
      display.setFont(f3);
      display.setCursor(10, 40); display.println(rtdData);
      display.setCursor(172,40); display.println(salfinal);
      display.setFont(f2);
      display.setCursor(40,70);display.print("deg C");
      display.setCursor(200,70);display.print("PSU");
    
      //cadre
      display.fillRect(147, 0, 2, 90, GxEPD_BLACK);
      display.fillRect(0, 90, 296, 2, GxEPD_BLACK); 
      
      //Last update
      display.setFont(f1);
      display.setCursor(0,108); display.print(date);display.print("/");display.print(month);display.print("/");display.print(year);  //date
      display.setCursor(123,108);display.print(hour);display.print("h");display.print(minute);                                       //heure
      display.setCursor(205,108);display.print("Bat:");display.print(soc);display.print("%");                                        //batterie
      display.setCursor(0, 125); display.print("Lat:");display.print(gps.location.lat(), 5);                                                     //lattitude
      display.setCursor(148,125); display.print("Lng:");display.print(gps.location.lng(), 4);                                                   //longitude
     
      display.update();


//      digitalWrite(rtdpin, LOW);   // temp
//      digitalWrite(ecpin, LOW);   // ec
             
  } // -------    fin Boucle exécution du programme (if pour l'intro, et else pour le programme principal)      --------------------
  

  // endormissement esp32
  Serial.println("Going to sleep");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}


void loop()
{
 //never use because of the sleeping of the princess esp32
 // peut être à utiliser pour la fonction wifi
}



/*
 * *****************************        LISTE DES FONCTION UTILISER DANS LE PROGRAMME            ***********************************
 */




void affichageintro(){                     // texte intro à l'allumage
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);
  
  //titre
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f4);
  display.setCursor(25, 55  );
  display.println("SENSOCEAN ");

  //version
  display.setFont(f1);
  display.setCursor(30, 90);
  display.print("Num Serie :");display.println(numserie);
  display.setCursor(30, 110);
  display.print("Ver Soft  :");display.print(versoft);
 
  display.update();
}

void errormessage(){                     // texte intro à l'allumage
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);
  
  //titre
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f4);
  display.setCursor(25, 55  );
  display.println("  ERROR ");

  //version
  display.setFont(f1);
  display.setCursor(30, 90);
  display.print("No SD Card ");
 
  display.update();
}


void mesureRTD(){
  Wire.beginTransmission(rtdAddress);    // Call the circuit by its ID number.  
  Wire.write('r');                       // Transmit the command that was sent through the serial port.
  Wire.endTransmission();                // End the I2C data transmission. 
  delay(rtdDelay);                       //wait the correct amount of time for the circuit to complete its instruction. 600ms by default.
  Wire.requestFrom(rtdAddress, 20, 1);   // Call the circuit and request 20 bytes (this may be more than we need)
  rtdCode=Wire.read();                   // The first byte is the response code, we read this separately.  
    switch (rtdCode){                    // Switch case based on what the response code is.  
      case 1:                            // Decimal 1.  
        Serial.println("RTD Success");   // Means the command was successful.
        break;                           // Exits the switch case.
      case 2:                            // Decimal 2. 
        Serial.println("RTD Failed");    // Means the command has failed.
        break;                           // Exits the switch case.
      case 254:                          // Decimal 254.
        Serial.println("RTD Pending");   // Means the command has not yet been finished calculating.
        break;                           // Exits the switch case.
      case 255:                          // Decimal 255.
        Serial.println("RTD No Data");   // Means there is no further data to send.
        break;                           // Exits the switch case.
    }
    byte i = 0;                          // Counter used for RTD_data array. 
    while(Wire.available()) {            // Are there bytes to receive.  
      rtdInChar = Wire.read();           // Receive a byte.
      rtdData[i]= rtdInChar;             // Load this byte into our array.
      i+=1;                              // Incur the counter for the array element. 
      if(rtdInChar == 0){                // If we see that we have been sent a null command. 
        i = 0;                           // Reset the counter i to 0.
        Wire.endTransmission();          // End the I2C data transmission.
        break;                           // Exit the while loop.
      }
    }
    Serial.println(rtdData);            //print the data.  
}



void mesureEC(){
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
    switch (ecCode) {                    // Switch case based on what the response code is.
      case 1:                            // Decimal 1.
        Serial.println("EC Success");    // Means the command was successful.
        break;                           // Exits the switch case.
      case 2:                            // Decimal 2.
        Serial.println("EC Failed");     // Means the command has failed.
        break;                           // Exits the switch case.
      case 254:                          // Decimal 254.
        Serial.println("EC Pending");    // Means the command has not yet been finished calculating.
        break;                           // Exits the switch case.
      case 255:                          // Decimal 255.
        Serial.println("EC No Data");    // Means there is no further data to send.
        break;                           // Exits the switch case.
    }
    ec = strtok(ecData, ","); 
    tds = strtok(NULL, ","); 
    sal = strtok(NULL, ","); 
    sg = strtok(NULL, ",");    // Let's pars the string at each comma.

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

void setupBQ27441(void)
{
  // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!lipo.begin()) // begin() will return true if communication is successful
  {
  // If communication fails, print an error message and loop forever.
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
    while (1) ;
  }
  Serial.println("Connected to BQ27441!");
  
  // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
  // of your battery.
  lipo.setCapacity(BATTERY_CAPACITY);
}

static void smartDelay(unsigned long ms)               
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}



//void affichageintro() {                 // pour ecran 4,2"
//    display.fillScreen(GxEPD_WHITE);
//
//  // Texte AE
//  display.setTextColor(GxEPD_BLACK);
//  display.setCursor(50, 80);
//  display.setFont(f2);
//  display.println("Astrolabe Expeditions");
//  
//  //cadre des data
//  display.fillRect(0, 140, display.width(), 50, GxEPD_BLACK);
////  display.fillRect(199, 150, 2, 30, GxEPD_WHITE); 
////  display.fillRect(199, 180, 2, 120, GxEPD_BLACK); 
//  display.setTextColor(GxEPD_WHITE); 
//  display.setCursor(140, 169);
//  display.println("SENSOCEAN");
//
//  //version 
//  display.setTextColor(GxEPD_BLACK);
//  display.setCursor(50, 250);
//  display.setFont(f1);
//  display.print("Num Serie :");display.print(numserie);
//  display.setCursor(50, 270);
//  display.print("Version :");display.print(versoft);
//  display.update();
//}




//void affichagebase() {                                     // pour affichage sur ecran 4,2"
//  display.fillScreen(GxEPD_WHITE);
//  // Read battery stats from the BQ27441-G1A
//  unsigned int soc = lipo.soc();  // Read state-of-charge (%)
////  unsigned int volts = lipo.voltage(); // Read battery voltage (mV)
////  int current = lipo.current(AVG); // Read average current (mA)
////  unsigned int fullCapacity = lipo.capacity(FULL); // Read full capacity (mAh)
////  unsigned int capacity = lipo.capacity(REMAIN); // Read remaining capacity (mAh)
////  int power = lipo.power(); // Read average power draw (mW)
////  int health = lipo.soh(); // Read state-of-health (%)
////
////  // Now print out those values:
////  String toPrint = String(soc) + "% | ";
////  toPrint += String(volts) + " mV | ";
////  toPrint += String(current) + " mA | ";
////  toPrint += String(capacity) + " / ";
////  toPrint += String(fullCapacity) + " mAh | ";
////  toPrint += String(power) + " mW | ";
////  toPrint += String(health) + "%";
////  Serial.println(toPrint);
//  
//  //Bandeau AE
//  display.fillRect(0, 0, display.width(), 40, GxEPD_BLACK);
//  display.setCursor(0, 0);
//  display.setTextColor(GxEPD_WHITE);
//  display.setFont(f1);
//  display.println();
//  display.println(" Astrolabe Expeditions     SensOcean");
//
//  // Last Update
//  display.setTextColor(GxEPD_BLACK);
//  display.setCursor(0, 60);
//  display.println("Last Update");
//  display.println(" ");
//  //display.print(date);display.print("/");display.print(month);display.print("/20");display.println(year);
//  display.println("Long :xx.3456789 - lat :yyy.3456789 ");
//  display.setCursor(258,60);display.print("Bat : "); display.print(soc);
//    
//  //cadre des data
//  display.fillRect(0, 150, display.width(), 30, GxEPD_BLACK);
//  display.fillRect(199, 150, 2, 30, GxEPD_WHITE); 
//  display.fillRect(199, 180, 2, 120, GxEPD_BLACK); 
//  display.setTextColor(GxEPD_WHITE); 
//  display.setCursor(35, 169);
//  display.println("TEMPERATURE");
//  display.setCursor(258, 169);
//  display.println("SALINITY");
//
//  // calsal - fonction temporaire juste pour affichae
// float apar=0.0016, bpar=0.6318, cpar=0.5055; // pour un caclul pour une température de 22°
// //float apar=0.0002, bpar=0.7443, cpar=0.5697; // pour un caclul pour une température de 15°
// float ecfloat=atof(ecData);
// float xval=ecfloat/1000;
// float salfinal = apar*(xval*xval)+bpar*xval-cpar;  
// Serial.print("Ecdata : ");Serial.print(ecData); Serial.print("Ecfloat : ");Serial.print(ecfloat); Serial.print("sal final : ");Serial.println(salfinal);
//  
//  //Data
//  display.setTextColor(GxEPD_BLACK);
//  display.setFont(f3);
//  display.setCursor(20,240);
//  display.print(rtdData);display.print(" C");
//  display.setCursor(210,240);
//  display.print(salfinal,2);display.print(" PSU");
//   
//  display.update();
//}
