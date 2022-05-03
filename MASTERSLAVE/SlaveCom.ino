
/*!
 * \file      SlaveCom.ino
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


/*

* #00MANNNNNXX$ +CR                        Richiesta dati inizia con # poi 00 stazione richiedente MA   
*                                          (nodo master) poi NNNNNN numero record ricevuto l'ultima volta
*                                          poi XX cecksum pacchetto chiuso con $ + CR
* #00SLNNNNN,AA.AAAAAA,BB.BBBBBBXX$ +CR    invio dati inizia con # poi 00 stazione richiedente
*                                          SL   (nodo slave) poi NNNNNN numero record inviato 
*                                          AA.AAAAAA = latitudine 
*                                          BB.BBBBBB = longitudine 
*                                          poi XX cecksum pacchetto chiuso con $ + CR
*/

#include <ESP32_LoRaWAN.h>
#include "Arduino.h"
#include <TinyGPS++.h>

#define RF_FREQUENCY                                868000000 // Hz

#define TX_OUTPUT_POWER                             15        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                         1000
#define BUFFER_SIZE_TX                             40 // Define the payload size in Tx compreso CR
#define BUFFER_SIZE_RX                             14 // Define the payload size in Rx
#define BUFFER_SIZE_RESP                           40 // Define the payload size in response
#define BUFFER_SIZE_CSV                            30 // Define the payload size seriale CSV
#define BUFFER_LEN                               1000// Numero letture GPS memorizzate

//#define DEBUG

struct dataPackageModel {        
   int    id_dp;              // numero progressivo pacchetto
   String latitude;           // stringa messaggio contenente le coordinate
   String longitude;          // stringa messaggio contenente le coordinate
   int    n_tx;               // numero tentativi di trasmissione
};

dataPackageModel  dataPackage[BUFFER_LEN];


// The TinyGPS++ object
TinyGPSPlus gps;
static const int RXPin = 2, TXPin = 17;
static const uint32_t GPSBaud = 9600;

String latitudeGps = "";
String longitudeGps = "";
String n_Sat= "";
unsigned long int counter =0;
unsigned long int sendTime=0, readTime =0;
unsigned int idx = 0, n_record =0;


char txpacket[BUFFER_SIZE_TX] = "#00SL00000,00.000000,00.000000";
char rxpacket[BUFFER_SIZE_RX];
char Repacket[BUFFER_SIZE_RESP];
char serialoutCSV[BUFFER_SIZE_CSV ];                  
unsigned char byteCS = ' ';
unsigned char high4bit, low4bit;


static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRxTimeout(void);
void displayMcuInit();
void displaySendReceive();
unsigned char CalcChecksum(char *frame, int numchar); // calcolo checksum

typedef enum
{
    STATUS_LOWPOWER,
    STATUS_RX,
    STATUS_TX
}States_t;


States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;

//ESP32 4cd5fe7eb994  0xF790512A,0xEA9867A9,0x8B00F84F,0x950E661F Chengdu 2021/12/29 16:37:06
// licenza creata per utilizzo su CHIP_ID 4cd5fe7eb994
uint32_t  license[4] = {0xF790512A,0xEA9867A9,0x8B00F84F,0x950E661F};

void setup()
{
  displayMcuInit();
  
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

    while (!Serial);
    SPI.begin(SCK,MISO,MOSI,SS);
    Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);

    Rssi=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    
    // preparazione del primo pacchetto da inviare
    byteCS = CalcChecksum(txpacket, 30);
    high4bit = byteCS >> 4;
    low4bit  = byteCS & 0b00001111;
    txpacket[30]=high4bit +(high4bit>9?55:48);
    txpacket[31]=low4bit  +(low4bit >9?55:48);
    txpacket[32]='$';
    txpacket[33]='\n';
    
    state=STATUS_TX;
}


void loop()
{
  /* lettura del modulo GPS*/
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read())){;}
  if (millis() > 5000 && gps.charsProcessed() < 10){
    Serial.println(F("No GPS detected: check wiring."));
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
  
  /**archiviazione dati nel buffer**/
  if ((millis() - readTime) >= 1000 ){
    readTime = millis();
    if(latitudeGps != "INVALID" && longitudeGps != "INVALID"){
       counter++;
       for(int y = 0; y < BUFFER_LEN; y++){
          if(dataPackage[y].n_tx >= 1 || dataPackage[y].id_dp == 0 ){ 
            // sovrascrivo il dato se è stato trasmesso
            dataPackage[y].id_dp = counter;
            dataPackage[y].latitude = latitudeGps;
            dataPackage[y].longitude = longitudeGps;
            dataPackage[y].n_tx = 0;
            Serial.print(dataPackage[y].id_dp );
            Serial.print(",");
            Serial.print(dataPackage[y].latitude );
            Serial.print(",");
            Serial.println(dataPackage[y].longitude);
            y = BUFFER_LEN;
          }
          if(y == (BUFFER_LEN -1)){
            #ifdef DEBUG
              Serial.println(" Buffer Overflow !");
            #endif   
          }
       }
     }
  }

  //conteggio del numero di record da trasmettere
  idx =0;
  for(int y = 0; y < BUFFER_LEN; y++){
    if(dataPackage[y].n_tx == 0 && dataPackage[y].id_dp != 0){
      idx +=1; 
    }
  }
  n_record = idx;
  #ifdef DEBUG
    Serial.print("N. Record buffer: ");
    Serial.println(n_record);
  #endif   


  switch(state)
  {
    case STATUS_TX:
        delay(300); //1000
        #ifdef DEBUG
          Serial.println(txpacket);
        #endif   
        Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
        state=STATUS_LOWPOWER;
        break;
    case STATUS_RX:
        #ifdef DEBUG  
          Serial.println("into RX mode");
        #endif 
        Radio.Rx( 2000 ); //5000
        state=STATUS_LOWPOWER;
        break;
    case STATUS_LOWPOWER:
        LoRaWAN.sleep(CLASS_C,0);
        break;
    default:
        break;
  }
}

void OnTxDone( void )
{
  #ifdef DEBUG
    Serial.print("TX done......");
  #endif
  sprintf(Repacket,"%s","");
  displaySendReceive();
  state=STATUS_RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    #ifdef DEBUG
      Serial.print("TX Timeout......");
    #endif
    sprintf(Repacket,"%s","TX Timeout,Retransmission");
    displaySendReceive();
    state=STATUS_TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    char numcount[6];
    int  n_count =0;
    Rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    Radio.Sleep( );
    byteCS = CalcChecksum(rxpacket, 10);
    high4bit = byteCS >> 4;
    low4bit  = byteCS & 0b00001111;
    if( (rxpacket[10] == high4bit +(high4bit>9?55:48)) && (rxpacket[11] == low4bit  +(low4bit >9?55:48))){
      for(int i =0; i < 5; i++)  // rilevo il numero del pacchetto ricevuto dal master
        numcount[i] = rxpacket[5+i];
      numcount[5] ='\0';
      n_count = atoi(numcount);
      for(int i =0; i < BUFFER_LEN; i++) // incremento in numero tx nel record trasmesso correttamente
        if((dataPackage[i].id_dp == n_count) && (dataPackage[i].id_dp != 0 )){
           dataPackage[i].n_tx += 1;/************************/
        }   
    }
    else
       #ifdef DEBUG
       Serial.println("RX error!");
       #endif
    #ifdef DEBUG
      Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
      Serial.println("wait to send next packet");
    #endif
    sprintf(Repacket,"%s","");
    for(int i=0;i < BUFFER_LEN; i++){ 
      if(dataPackage[i].n_tx == 0 && dataPackage[i].id_dp != 0 ){
        String count = String(dataPackage[i].id_dp);
        while(count.length() < 5)
          count = "0" + count;
        for(int x=0;x < count.length(); x++)       
          txpacket[x+5] = count.charAt(x); //"#00SL00000,00.000000,00.000000";
        txpacket[10] = ',';                //"#00SL00000,00.000000,00.000000";
        for(int x=0;x < dataPackage[i].latitude.length(); x++)       
          txpacket[x+11] = dataPackage[i].latitude.charAt(x); //"#00SL00000,00.000000,00.000000";      
        txpacket[20] = ',';
        for(int x=0;x < dataPackage[i].longitude.length(); x++)       
          txpacket[x+21] = dataPackage[i].longitude.charAt(x); //"#00SL00000,00.000000,00.000000";
        byteCS = CalcChecksum(txpacket, 30);
        high4bit = byteCS >> 4;
        low4bit  = byteCS & 0b00001111;
        txpacket[30]=high4bit +(high4bit>9?55:48); // converto i 4 bit superiori in esadecimale
        txpacket[31]=low4bit  +(low4bit >9?55:48); // converto i 4 bit inferiori in esadecimale 
        txpacket[32]='$';
        txpacket[33]='\n'; 
        i = BUFFER_LEN;    // esco dal ciclo for    
      }
    }  
    displaySendReceive();
    state=STATUS_TX;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    #ifdef DEBUG  
      Serial.print("RX Timeout......");
    #endif
    sprintf(Repacket,"%s","RX Timeout,Retransmission");
    displaySendReceive();
    state=STATUS_TX;
}

unsigned char CalcChecksum(char *frame, int numchar)
{ unsigned char res=0;
    for(int i=0;i<numchar;i++){
       res ^= *(frame+i);
    }
  return res;
}

void displayMcuInit()
{
  Display.wakeup();
  Display.init();
  delay(100);
  Display.flipScreenVertically();
  Display.setFont(ArialMT_Plain_10);
  Display.setTextAlignment(TEXT_ALIGN_LEFT);
  Display.clear();
  Display.drawString(18, 11, "SLAVE TX_RX");
  Display.drawString(40, 31, "VER. 0.0.1");

  Display.display();
  delay(1000);
  Display.clear();
}

void displaySendReceive()
{
  Display.drawString(0,0,"TX packet:   BUF:");
  Display.drawString(100,0,(String)n_record);
  Display.drawString(0,11,(String)txpacket);
  Display.drawString(0,25,(String)Repacket);
  Display.drawString(0,40,"RX packet:" );
  Display.drawString(0,51,(String)rxpacket);
  Display.display();
  delay(100);
  Display.clear();
}
