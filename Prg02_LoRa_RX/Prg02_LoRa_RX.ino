/*!
 * \file      Prg02_LoRa_RX.ino
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


#include "heltec.h" 
#include "images.h"

#define BAND    868E6  //you can set band here directly,e.g. 868E6,915E6
#define SF      11   //you can set band here directly,e.g. 868E6,915E6
String rssi = "RSSI --";
String snr = "SNR --";
String packSize = "--";
String packet, StrDati ;

void logo(){
  Heltec.display->clear();
  Heltec.display->drawXbm(0,5,logo_width,logo_height,logo_bits);
  Heltec.display->display();
}

void LoRaData(){
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0 , 15 , "Received "+ packSize + " bytes");
  Heltec.display->drawStringMaxWidth(0 , 26 , 128, packet);
  Heltec.display->drawString(0, 0, rssi);  
  Heltec.display->display();
  StrDati = packet.substring(5,30);
  StrDati = StrDati + "," + rssi + "," + snr;
  Serial.println(StrDati);
}

void cbk(int packetSize) {
  packet ="";
  packSize = String(packetSize,DEC);
  for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
  rssi = "RSSI " + String(LoRa.packetRssi(), DEC) ;
  snr = "SNR " + String(LoRa.packetSnr(), DEC) ;
  LoRaData();
}

void setup() { 
   //WIFI Kit series V1 not support Vext control
  
  Serial.begin(115200);
  delay(500);
  
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  LoRa.setSpreadingFactor(SF);           // ranges from 6-12,default 7 see API docs
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  delay(500);
  Heltec.display->clear();
  
  Heltec.display->drawString(0, 0, "Heltec.LoRa Initial success!");
  Heltec.display->drawString(0, 10, "Wait for incoming data...");
  Heltec.display->display();
  delay(1000);
  LoRa.receive();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) { cbk(packetSize);  }
  delay(10);
}
