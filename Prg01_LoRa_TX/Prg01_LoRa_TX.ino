/*!
 * \file      Prg01_LoRa_TX.ino
 *
 * \brief     Target board Heltec ESP32 LoRa (V2)
 *
 * \code
 *            UNIVERSITA' GUGLIELMO MARCONI 
 * 
 *
 * \endcode
 *
 * \author    Luca Paolinelli 
 *
 * \date      02 marzo 2022
 */


/*FORMATO DA TRASMETTERE
 * 
 * &0#0#NNNNN.00141,11402.3278$
 * 
 * 
 */


#include "heltec.h"
#include "images.h"
#include <TinyGPS++.h> 

#define BAND    868E6  //you can set band here directly,e.g. 868E6,915E6
#define BUF_LEN 2500   // lunghezza del buffer  
#define SF        7   // valore di SF

// The TinyGPS++ object
TinyGPSPlus gps;
static const int RXPin = 2, TXPin = 17;
static const uint32_t GPSBaud = 9600;

struct dataPackageModel {        
   int    id_dp;              // numero progressivo pacchetto
   String latitude;           // stringa messaggio contenente latitudine
   String longitude;          // stringa messaggio contenente longitudine
   int    n_tx;               // numero tentativi di trasmissione
};

dataPackageModel  dataPackage[BUF_LEN];

String startMessage = "&0#0#";
String endMessage   = "$";
String message = "";
String latitudeGps = "";
String longitudeGps = "";
String n_Sat= "";

unsigned long int counter =1;
unsigned long int tot_pack_TX =0;
unsigned long int sendTime=0, readTime =0;
unsigned int idx = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet ;


byte localAddress = 0x0A;     // address of this device
byte destination = 0x0B;      // destination to send to

volatile int interruptCounter;
 

void logo()
{
  Heltec.display->clear();
  Heltec.display->drawXbm(0,5,logo_width,logo_height,logo_bits);
  Heltec.display->display();
}

/*******************************************************
 * Name : setup()
 * auth.: luca p
 * date : 03/03/2022
 ******************************************************/
void setup()
{
  // GPS serial config 
  Serial2.begin(GPSBaud, SERIAL_8N1,RXPin,TXPin);
  // serial monitor
  Serial.begin(115200);
  delay(500); 
  
  Serial.println(F("...-GPS config-..."));
  Serial.print(F("TinyGPS++ library v. ")); 
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by luca p."));
  Serial.println();
   //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  LoRa.setSpreadingFactor(SF);           // ranges from 6-12,default 7 see API docs
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  
  Heltec.display->clear();
  
  Heltec.display->drawString(0, 0, "Heltec.LoRa Initial success!");
  Heltec.display->display();
  delay(1000);
}

void loop()
{
  
  
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0,  "TX n. : ");
  Heltec.display->drawString(60, 0, String(tot_pack_TX));
  Heltec.display->drawString(0, 15, "lat.  :");
  Heltec.display->drawString(40, 15, latitudeGps);
  Heltec.display->drawString(0, 30, "lon. :");
  Heltec.display->drawString(40, 30, longitudeGps);
  Heltec.display->drawString(0,45,  "sat. :");
  Heltec.display->drawString(40, 45, n_Sat);
  Heltec.display->display();
 
  // displays information every time a new sentence is correctly encoded.
  while (Serial2.available() > 0)
     gps.encode(Serial2.read());
      //displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10){
       
    Serial.println(F("No GPS detected: check wiring."));

    latitudeGps = "INVALID";
    longitudeGps = "INVALID";
    n_Sat = "INVALID";
    //while(true);
  }
  else{
    if (gps.satellites.isValid())  
       n_Sat = String(gps.satellites.value(),5);
    else
       n_Sat = "INVALID";
    if (gps.location.isValid()){
       latitudeGps = String(gps.location.lat(), 6);
       longitudeGps = String(gps.location.lng(), 6);
    }
    else{
       latitudeGps = "INVALID";
       longitudeGps = "INVALID";
    }
  }  
  
  
  if ((millis() - readTime) >= 1000 ){
    readTime = millis();
    if(latitudeGps != "INVALID" && longitudeGps != "INVALID"){
       counter++;
       if(idx >= BUF_LEN)  // quando super la lunghezza del buffer cancello i più vecchi
          idx = 0;
        dataPackage[idx].id_dp = counter;
        dataPackage[idx].latitude = latitudeGps;
        dataPackage[idx].longitude = longitudeGps;
        dataPackage[idx].n_tx = 0;
        idx++;
    }
  }


  if ((millis() - sendTime) >= 1150 ){  // send packet
     sendTime = millis();
     for(int i=0;i < BUF_LEN; i++) 
        if(dataPackage[i].n_tx == 0 && dataPackage[i].id_dp != 0 ){
           if ( sendMessage(i)){
              Serial.print(dataPackage[i].id_dp );
              Serial.print(",");
              Serial.print(dataPackage[i].latitude );
              Serial.print(",");
              Serial.println(dataPackage[i].longitude);
              tot_pack_TX +=1;
              
           }
           i = BUF_LEN;    // esco dal ciclo for    
        }
  }

}
/*******************************************************
 * Name : sendMessage(unsigned int n_index)
 * auth.: luca p
 * date : 03/03/2022
 ******************************************************/
bool sendMessage(unsigned int n_index)
{
  String count = String(dataPackage[n_index].id_dp);
  while(count.length() < 5)
    count = "0" + count;
  message = count + "," + dataPackage[n_index].latitude +
            "," +dataPackage[n_index].longitude;

  LoRa.beginPacket();                   // start packet
 /*
 * LoRa.setTxPower(txPower,RFOUT_pin);
 * txPower -- 0 ~ 20
 * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or 
 *                    RF_PACONFIG_PASELECT_RFO
 *   - RF_PACONFIG_PASELECT_PABOOST -- 
 *     LoRa single output via PABOOST, maximum output 20dBm
 *   - RF_PACONFIG_PASELECT_RFO     -- 
 *     LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
 */
  LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print(message);                 // add payload
  LoRa.endPacket();                    // finish packet and send it
  dataPackage[n_index].n_tx +=1;
  return true;
}

/*******************************************************
 * Name : displayInfo()
 * auth.: luca p
 * date : 03/03/2022
 ******************************************************/
void displayInfo()
{
  Serial.print(F("N. Sat. ")); 
  if (gps.satellites.isValid())
  {
    Serial.print(gps.satellites.value(), 5);
  }  
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
