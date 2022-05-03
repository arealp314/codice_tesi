/*!
 * \file      MasterCom.ino
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

 * #00MANNNNNXX$ +CR    Richiesta dati inizia con # poi 00 stazione 
 *                      richiedente MA (nodo master) poi NNNNNN numero 
 *                      record ricevuto l'ultima volta poi XX cecksum 
 *                      pacchetto chiuso con $ + CR
*/

#include <ESP32_LoRaWAN.h>
#include "Arduino.h"

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
#define LORA_PREAMBLE_LENGTH                        8         // Tx = Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE_TX                                 14 // Define the payload size in Tx compreso CR
#define BUFFER_SIZE_RX                                 40 // Define the payload size in Rx
#define BUFFER_SIZE_RESP                               40 // Define the payload size in response
#define BUFFER_SIZE_CSV                                30 // Define the payload size seriale CSV

//#define DEBUG

char txpacket[BUFFER_SIZE_TX] = "#00MA00000";
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
void ReadPack( void );

typedef enum
{
    STATUS_LOWPOWER,
    STATUS_RX,
    STATUS_TX
}States_t;


int16_t txNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;


//ESP32 DC2F14F7C630  0x293276B4,0x8B5A30C0,0x3E41C1FD,0xBF554C23 Chengdu 2022/03/22 16:28:22

// licenza creata per utilizzo su CHIP_ID DC2F14F7C630
uint32_t  license[4] = {0x293276B4,0x8B5A30C0,0x3E41C1FD,0xBF554C23};

void setup()
{
  displayMcuInit();
  
  Serial.begin(115200);
  while (!Serial);
  SPI.begin(SCK,MISO,MOSI,SS);
  Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);


    txNumber=0;
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
    byteCS = CalcChecksum(txpacket, 10);
    high4bit = byteCS >> 4;
    low4bit  = byteCS & 0b00001111;
    txpacket[10]=high4bit +(high4bit>9?55:48); // converto i 4 bit superiori in esadecimale
    txpacket[11]=low4bit  +(low4bit >9?55:48); // converto i 4 bit inferiori in esadecimale 
    txpacket[12]='$';
    txpacket[13]='\n';
    
    state=STATUS_TX;
}


void loop()
{
  switch(state)
  {
    case STATUS_TX:
        delay(300);
        #ifdef DEBUG
          Serial.println("Modo TX");
          Serial.println(tx_packet);
        #endif  
        Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
        state=STATUS_LOWPOWER;
        break;
    case STATUS_RX:
        #ifdef DEBUG
          Serial.println("Modo RX");
        #endif  
        Radio.Rx( 2000 );
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
  sprintf(Repacket,"%s","TxOk");
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
    Rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    Radio.Sleep();
    ReadPack();
    #ifdef DEBUG
      Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
      Serial.println("wait to send next packet");
    #endif 
    sprintf(Repacket,"%s","RxOk");
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

void ReadPack( void ){
  
  byteCS = CalcChecksum(rxpacket, 30);
  high4bit = byteCS >> 4;
  low4bit  = byteCS & 0b00001111;
  if(rxpacket[30] == high4bit +(high4bit>9?55:48) && rxpacket[31] == low4bit  +(low4bit >9?55:48)){
    for(int i =0; i < 10; i++)  // creo il pacchetto di richiesta dati con il codice dell'ultimo ricevuto
      txpacket[i] = rxpacket[i];
    txpacket[3] = 'M';
    txpacket[4] = 'A';
    byteCS = CalcChecksum(txpacket, 10); // calcolo checksum del pacchetto da trasmettere
    high4bit = byteCS >> 4;
    low4bit  = byteCS & 0b00001111;
    txpacket[10]=high4bit +(high4bit>9?55:48); // converto i 4 bit superiori in esadecimale
    txpacket[11]=low4bit  +(low4bit >9?55:48); // converto i 4 bit inferiori in esadecimale 
    txpacket[12]='$';
    txpacket[13]='\0';
    for(int x =5; x < 30; x++) //invio il pacchetto alla seriale   "NNNNN,AA.AAAAAA,BB.BBBBBB" 
      serialoutCSV[x-5] = rxpacket[x];
    serialoutCSV[25] = '\0';
    Serial.println(serialoutCSV);
  }
  else{
    #ifdef DEBUG
       Serial.println("RX error!");
    #endif 
  }
}


unsigned char CalcChecksum(char *frame, int numchar)
{ 
  unsigned char res=0;
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
  Display.drawString(18, 11, "MASTER TX_RX");
  Display.drawString(40, 31, "VER. 0.0.1");
  Display.display();
  delay(1000);
  Display.clear();
}

void displaySendReceive()
{
    Display.drawString(0,0,"TX packet:");
    Display.drawString(0,11,(String)txpacket);
    Display.drawString(0,25,(String)Repacket);
    Display.drawString(0,40,"RX packet:" );
    Display.drawString(0,51,(String)rxpacket);
    Display.display();
    delay(100);
    Display.clear();
}
