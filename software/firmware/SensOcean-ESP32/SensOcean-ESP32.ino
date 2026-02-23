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
WaveShare display 4,2 inch on ESP32 Firebeetle
Branchement Ecran : BUSY -> 4, RST -> 25, DC -> 13, CS -> SS(5), CLK -> SCK(18), DIN -> MOSI(23), GND -> GND, 3.3V -> 3.3V,


Etat du programme : 
-------------------


Pour la prochain version : 
------------------------

***********************************************************/


// ---------------------   PARAMETRES MODIFIABLE DU PROGRAM    -----------------------------------
// Version et numero de serie
char numserie[] = "AESO18004";      // Numero de serie de la sonde
char versoft[] = "5.0";             // version du code

#define TIME_TO_SLEEP  10           // Durée d'endormissement entre 2 cycles complets de mesures (in seconds)
int nbrMes = 3;                     // nombre de mesure de salinité et température par cycle

// --------------------     FIN DES PARAMETRES MODIFIABLES     ------------------------------------


// Library
#include <GxEPD.h>
#include <GxGDEW042T2/GxGDEW042T2.h>  // 4.2" b/w
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include <Wire.h>                    //enable I2C.
#include <DS3231.h>                  // Pour horloge RTC
#include "SPI.h"                     // pour connection ecran bus SPI
#include <SD.h>                       // pour carte SD

//SPI pin definitions pour ecran epaper
GxIO_Class io(SPI, /*CS=5*/ 0, /*DC=*/ 13, /*RST=*/ 25); // arbitrary selection of 17, 16  //SS remplacé par 0
GxEPD_Class display(io, /*RST=*/ 25, /*BUSY=*/ 4); // arbitrary selection of (16), 4

// définition pour les font ecran
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

void setup()
{
  // ------------------        HERE IS NEEDED THE PARAMETER FOR THE ENTIER PROGRAMM   ------------------------------------------
  Serial.begin(115200);
  Wire.begin();  
  delay(1000); //Take some time to open up the Serial Monitor
  display.init(115200); // enable diagnostic output on Serial

  // Test carte SD
  Serial.print("Initializing SD card...");   
    if (!SD.begin(5)) {                      // // see if the card is present and can be initialized, ajouter ici chipSelect ou 5 pour la pin 5 par default
    Serial.println("Card failed, or not present");
    //delay(300);
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

//  pinMode(rtdpin, OUTPUT);            // pin temperature
//  pinMode(ecpin, OUTPUT);            // pin EC
//  digitalWrite(rtdpin, LOW);   // temp
//  digitalWrite(ecpin, LOW);   // ec

  if(bootCount == 0) //Run this only the first time
  {
      // --------------        HERE IS ONLY THE INTRODUCION           -----------------------------------------------------------
      
      affichageintro();       // texte d'intro et cadre initiale
      delay(2000);

      
      bootCount = bootCount+1;   // changement du numéro de compteur pour passer directement dans programm loop apres le reveil
      //Serial.println("Boot number: " + String(bootCount));  //affichage du numéro de boucle depuis que l'instrument est allumé
      
  }else
  {
      // ---------------        HERE IS THE MAIN PROGRAMM LOOP         ----------------------------------------------------------
      
//      digitalWrite(rtdpin, HIGH);   // temp
//      digitalWrite(ecpin, HIGH);   // ec
//      delay(2000);

      // lecture de l'horloge rtc
      int second,minute,hour,date,month,year; 
      //second=Clock.getSecond();
      minute=Clock.getMinute();
      hour=Clock.getHour(h12, PM);
      date=Clock.getDate();
      month=Clock.getMonth(Century);
      year=Clock.getYear();

      
      // Mesure de temp et conduc et construction de la chaine de donnée pour enregistrer sur la carte SD
      String datachain ="";

        // date heure GPS de debut de chaine
        datachain += year; datachain += " ; ";datachain += month; datachain += " ; ";datachain += date; datachain += " ; ";
        datachain += hour; datachain += " ; ";datachain += minute; datachain += " ; ";
                
        // température
        datachain += "RTD : ";
        for(int n=1; n<=nbrMes; n++){
          mesureRTD(); 
          datachain += rtdData;
          datachain += " ; ";
        }
        
        // Salinité
        datachain += "EC : ";
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
          }


      // a faire                                               // enregistrement de la chaine sur carte SD
      Serial.print("datachain : "); Serial.println(datachain); //affichage de la chaine complete sur le port serie
      affichagebase();                                         // Rafraichissement des DATA pour l'affichage pour l'affichage ecran

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
}



/*
 * *****************************        LISTE DES FONCTION UTILISER DANS LE PROGRAMME            ***********************************
 */


void affichagebase() {
  display.fillScreen(GxEPD_WHITE);
  
  //Bandeau AE
  display.fillRect(0, 0, display.width(), 40, GxEPD_BLACK);
  display.setCursor(0, 0);
  display.setTextColor(GxEPD_WHITE);
  display.setFont(f1);
  display.println();
  display.println(" Astrolabe Expeditions     SensOcean");

  // Last Update
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(0, 60);
  display.println("Last Update");
  display.println(" ");
  //display.print(date);display.print("/");display.print(month);display.print("/20");display.println(year);
  display.println("Long :xx.3456789 - lat :yyy.3456789 ");
  
  //cadre des data
  display.fillRect(0, 150, display.width(), 30, GxEPD_BLACK);
  display.fillRect(199, 150, 2, 30, GxEPD_WHITE); 
  display.fillRect(199, 180, 2, 120, GxEPD_BLACK); 
  display.setTextColor(GxEPD_WHITE); 
  display.setCursor(35, 169);
  display.println("TEMPERATURE");
  display.setCursor(258, 169);
  display.println("SALINITY");

  //Data
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f3);
  display.setCursor(20,240);
  display.print(rtdData);display.print(" C");
  display.setCursor(210,240);
  display.print(ecData);display.print(" PSU");
   
  display.update();
}



void affichageintro() {
    display.fillScreen(GxEPD_WHITE);

  // Texte AE
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(50, 80);
  display.setFont(f2);
  display.println("Astrolabe Expeditions");
  
  //cadre des data
  display.fillRect(0, 140, display.width(), 50, GxEPD_BLACK);
//  display.fillRect(199, 150, 2, 30, GxEPD_WHITE); 
//  display.fillRect(199, 180, 2, 120, GxEPD_BLACK); 
  display.setTextColor(GxEPD_WHITE); 
  display.setCursor(140, 169);
  display.println("SENSOCEAN");

  //version 
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(50, 250);
  display.setFont(f1);
  display.print("Num Serie :");display.print(numserie);
  display.setCursor(50, 270);
  display.print("Version :");display.print(versoft);
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
