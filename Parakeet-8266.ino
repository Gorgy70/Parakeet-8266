//#define DEBUG
#define INT_BLINK_LED
//#define EXT_BLINK_LED

#include <SPI.h>
//#include <EEPROM.h>
#include <ESP8266WiFi.h>
//#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>

#include "cc2500_REG.h"

#define GDO0_PIN D1            // Цифровой канал, к которму подключен контакт GD0 платы CC2500


#define NUM_CHANNELS (4)      // Кол-во проверяемых каналов
#define FIVE_MINUTE 300000    // 5 минут

#define RADIO_BUFFER_LEN 200 // Размер буфера для приема данных от GSM модема

#define my_webservice_url    "http://parakeet.esen.ru/receiver.cgi"
#define my_webservice_reply  "!ACK"
#define my_user_agent        "parakeet-8266"
#define my_password_code     "12543"

#define my_wifi_ssid         "ssid"
#define my_wifi_pwd          "password"

//ESP8266WiFiMulti WiFiMulti;

unsigned long dex_tx_id;
char transmitter_id[] = "6E853";

unsigned long packet_received = 0;

byte fOffset[NUM_CHANNELS] = { 0xE4, 0xE3, 0xE2, 0xE2 };
byte nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
unsigned long waitTimes[NUM_CHANNELS] = { 0, 600, 600, 600 };

byte sequential_missed_packets = 0;
byte wait_after_time = 100;
unsigned long next_time = 0; // Время ожидания следующего пакета на канале 0
unsigned long catch_time = 0; // Время последнего пойманного пакета (приведенное к пакету на канале 0)

byte misses_until_failure = 2;                                                   //
// after how many missed packets should we just start a nonstop scan?                               //
// a high value is better for conserving batter life if you go out of wixel range a lot             //
// but it could also mean missing packets for MUCH longer periods of time                           //
// a value of zero is best if you dont care at all about battery life                               //

byte wifi_wait_tyme = 100; // Время ожидания соединения WiFi в секундах

char radio_buff[RADIO_BUFFER_LEN]; // Буффер для чтения данных и прочих нужд

// Коды ошибок мигают лампочкой в двоичной системе
// 1 (0001) - Нет модключения к WiFi
// 2 (0010) - Облачная служба не отвечает
// 3 (0011) - Облачная служба возвращает ошибку

typedef struct _Dexcom_packet
{
  byte len;
  unsigned long dest_addr;
  unsigned long src_addr;
  byte port;
  byte device_info;
  byte txId;
  unsigned int raw;
  unsigned int filtered;
  byte battery;
  byte unknown;
  byte checksum;
  byte RSSI;
  byte LQI2;
} Dexcom_packet;

Dexcom_packet Pkt;

char SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
                          '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
                          'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
                          'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y'
                        };


void dexcom_src_to_ascii(unsigned long src, char addr[6]) {
  addr[0] = SrcNameTable[(src >> 20) & 0x1F];
  addr[1] = SrcNameTable[(src >> 15) & 0x1F];
  addr[2] = SrcNameTable[(src >> 10) & 0x1F];
  addr[3] = SrcNameTable[(src >> 5) & 0x1F];
  addr[4] = SrcNameTable[(src >> 0) & 0x1F];
  addr[5] = 0;
}


unsigned long getSrcValue(char srcVal) {
  byte i = 0;
  for (i = 0; i < 32; i++) {
    if (SrcNameTable[i] == srcVal) break;
  }
  return i & 0xFF;
}

unsigned long asciiToDexcomSrc(char addr[6]) {
  unsigned long src = 0;
  src |= (getSrcValue(addr[0]) << 20);
  src |= (getSrcValue(addr[1]) << 15);
  src |= (getSrcValue(addr[2]) << 10);
  src |= (getSrcValue(addr[3]) << 5);
  src |= getSrcValue(addr[4]);
  return src;
}

byte bit_reverse_byte (byte in)
{
    byte bRet = 0;
    if (in & 0x01)
        bRet |= 0x80;
    if (in & 0x02)
        bRet |= 0x40;
    if (in & 0x04)
        bRet |= 0x20;
    if (in & 0x08)
        bRet |= 0x10;
    if (in & 0x10)
        bRet |= 0x08;
    if (in & 0x20)
        bRet |= 0x04;
    if (in & 0x40)
        bRet |= 0x02;
    if (in & 0x80)
        bRet |= 0x01;
    return bRet;
}

void bit_reverse_bytes (byte * buf, byte nLen)
{
    byte i = 0;
    for (; i < nLen; i++)
    {
        buf[i] = bit_reverse_byte (buf[i]);
    }
}

unsigned long dex_num_decoder (unsigned int usShortFloat)
{
    unsigned int usReversed = usShortFloat;
    byte usExponent = 0;
    unsigned long usMantissa = 0;
    bit_reverse_bytes ((byte *) & usReversed, 2);
    usExponent = ((usReversed & 0xE000) >> 13);
    usMantissa = (usReversed & 0x1FFF);
    return usMantissa << usExponent;
}

#ifdef EXT_BLINK_LED
void blink_sequence_red(const char *sequence) {
  byte i;

  digitalWrite(YELLOW_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  delay(500); 
  for (i = 0; i < strlen(sequence); i++) {
    digitalWrite(RED_LED_PIN, HIGH);
    switch (sequence[i]) {
      case '0': 
        delay(500);
        break;
      case '1': 
        delay(1000);
        break;
      default:
        delay(2000);
        break;
    }
    digitalWrite(RED_LED_PIN, LOW);
    delay(500); 
  }  
  digitalWrite(YELLOW_LED_PIN, LOW);
}
#endif

#ifdef INT_BLINK_LED
void blink_sequence(const char *sequence) {
  byte i;

  digitalWrite(LED_BUILTIN, LOW);
  delay(500); 
  for (i = 0; i < strlen(sequence); i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    switch (sequence[i]) {
      case '0': 
        delay(500);
        break;
      case '1': 
        delay(1000);
        break;
      default:
        delay(2000);
        break;
    }
    digitalWrite(LED_BUILTIN, LOW);
    delay(500); 
  }  
}

void blink_builtin_led_quarter() {  // Blink quarter seconds
  if ((millis() / 250) % 2) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void blink_builtin_led_half() {  // Blink half seconds
  if ((millis() / 500) % 2) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
#endif

#ifdef EXT_BLINK_LED
void blink_yellow_led_quarter() {  // Blink quarter seconds
  if ((millis() / 250) % 2) {
    digitalWrite(YELLOW_LED_PIN, HIGH);
  } else
  {
    digitalWrite(YELLOW_LED_PIN, LOW);
  }
}

void blink_yellow_led_half() {  // Blink half seconds
  if ((millis() / 500) % 2) {
    digitalWrite(YELLOW_LED_PIN, HIGH);
  } else
  {
    digitalWrite(YELLOW_LED_PIN, LOW);
  }
}

void blink_red_led_quarter() {  // Blink quarter seconds
  if ((millis() / 250) % 2) {
    digitalWrite(RED_LED_PIN, HIGH);
  } else
  {
    digitalWrite(RED_LED_PIN, LOW);
  }
}

void blink_red_led_quarter2() {  // Blink quarter seconds
  if ((millis() / 250) % 2) {
    digitalWrite(RED_LED_PIN, LOW);
  } else
  {
    digitalWrite(RED_LED_PIN, HIGH);
  }
}

void blink_red_led_half() {  // Blink half seconds
  if ((millis() / 500) % 2) {
    digitalWrite(RED_LED_PIN, HIGH);
  } else
  {
    digitalWrite(RED_LED_PIN, LOW);
  }
}

void blink_red_led_half2() {  // Blink half seconds
  if ((millis() / 500) % 2) {
    digitalWrite(RED_LED_PIN, LOW);
  } else
  {
    digitalWrite(RED_LED_PIN, HIGH);
  }
}
#endif

void WriteReg(char addr, char value) {
  digitalWrite(SS, LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(SS, HIGH);
  //  delay(10);
}

char SendStrobe(char strobe)
{
  digitalWrite(SS, LOW);

  while (digitalRead(MISO) == HIGH) {
  };

  char result =  SPI.transfer(strobe);
  digitalWrite(SS, HIGH);
  //  delay(10);
  return result;
}

void init_CC2500() {
//FSCTRL1 and MDMCFG4 have the biggest impact on sensitivity...
   
   WriteReg(PATABLE, 0x00);
//   WriteReg(IOCFG0, 0x01);
   WriteReg(IOCFG0, 0x06);
   WriteReg(PKTLEN, 0xff);
   WriteReg(PKTCTRL1, 0x0C); // CRC_AUTOFLUSH = 1 & APPEND_STATUS = 1
//   WriteReg(PKTCTRL1, 0x04);
   WriteReg(PKTCTRL0, 0x05);
   WriteReg(ADDR, 0x00);
   WriteReg(CHANNR, 0x00);

   WriteReg(FSCTRL1, 0x0f); 
   WriteReg(FSCTRL0, 0x00);  
  
   WriteReg(FREQ2, 0x5d);
   WriteReg(FREQ1, 0x44);
   WriteReg(FREQ0, 0xeb);
   
   WriteReg(FREND1, 0xb6);  
   WriteReg(FREND0, 0x10);  

   // Bandwidth
   //0x4a = 406 khz
   //0x5a = 325 khz
   // 300 khz is supposedly what dex uses...
   //0x6a = 271 khz
   //0x7a = 232 khz
   WriteReg(MDMCFG4, 0x7a); //appear to get better sensitivity
   WriteReg(MDMCFG3, 0xf8);
   WriteReg(MDMCFG2, 0x73);
   WriteReg(MDMCFG1, 0x23);
   WriteReg(MDMCFG0, 0x3b);
   
   WriteReg(DEVIATN, 0x40);

   WriteReg(MCSM2, 0x07);
   WriteReg(MCSM1, 0x30);
   WriteReg(MCSM0, 0x18);  
   WriteReg(FOCCFG, 0x16); //36
   WriteReg(FSCAL3, 0xa9);
   WriteReg(FSCAL2, 0x0a);
   WriteReg(FSCAL1, 0x00);
   WriteReg(FSCAL0, 0x11);
  
   WriteReg(AGCCTRL2, 0x03);  
   WriteReg(AGCCTRL1, 0x00);
   WriteReg(AGCCTRL0, 0x91);
   //
   WriteReg(TEST2, 0x81);
   WriteReg(TEST1, 0x35); 
   WriteReg(TEST0, 0x0b);  
   
   WriteReg(FOCCFG, 0x0A);    // allow range of +/1 FChan/4 = 375000/4 = 93750.  No CS GATE
   WriteReg(BSCFG, 0x6C);
 
}

char ReadReg(char addr) {
  addr = addr + 0x80;
  digitalWrite(SS, LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  char y = SPI.transfer(0);
  digitalWrite(SS, HIGH);
  //  delay(10);
  return y;
}

char ReadStatus(char addr) {
  addr = addr + 0xC0;
  digitalWrite(SS, LOW);
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  char y = SPI.transfer(0);
  digitalWrite(SS, HIGH);
  //  delay(10);
  return y;
}

void setup() {
#ifdef DEBUG
  byte b1;
#endif

#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif
  // initialize digital pin LED_BUILTIN as an output.
#ifdef INT_BLINK_LED
  pinMode(LED_BUILTIN, OUTPUT);
#endif
#ifdef EXT_BLINK_LED
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
#endif

  dex_tx_id = asciiToDexcomSrc (transmitter_id);
#ifdef DEBUG
  Serial.print("Dexcom ID: ");
  Serial.println(dex_tx_id);
#endif

  SPI.begin();
  //  SPI.setClockDivider(SPI_CLOCK_DIV2);  // max SPI speed, 1/2 F_CLOCK
  digitalWrite(SS, HIGH);

  init_CC2500();  // initialise CC2500 registers
#ifdef DEBUG
  Serial.print("CC2500 PARTNUM=");
  b1 = ReadStatus(PARTNUM);
  Serial.println(b1,HEX);
  Serial.print("CC2500 VERSION=");
  b1 = ReadStatus(VERSION);
  Serial.println(b1,HEX);
#endif
 ESP.wdtDisable();
 ESP.wdtEnable(WDTO_8S);
}

void swap_channel(unsigned long channel, byte newFSCTRL0) {

  SendStrobe(SIDLE);
  SendStrobe(SFRX);
//  WriteReg(FSCTRL0,newFSCTRL0);
  WriteReg(CHANNR, channel);
  SendStrobe(SRX);  //RX
  while (ReadStatus(MARCSTATE) != 0x0d) {
    // Подождем пока включится режим приема
  }
}

byte ReadRadioBuffer() {
  byte len;
  byte i;
  byte rxbytes;

  memset (&radio_buff, 0, sizeof (Dexcom_packet));
  len = ReadStatus(RXBYTES);
#ifdef DEBUG
  Serial.print("Bytes in buffer: ");
  Serial.println(len);
#endif
  if (len > 0 && len < 65) {
    for (i = 0; i < len; i++) {
      if (i < sizeof (Dexcom_packet)) {
        radio_buff[i] = ReadReg(RXFIFO);
#ifdef DEBUG
        Serial.print(radio_buff[i],HEX);
        Serial.print("\t");
#endif
      }
    }
    Serial.println();
  }
//  memcpy(&Pkt, &radio_buff[0], sizeof (Dexcom_packet));
/*
  unsigned long dest_addr;
  unsigned long src_addr;
  byte port;
  byte device_info;
  byte txId;
  unsigned int raw;
  unsigned int filtered;
  byte battery;
  byte unknown;
  byte checksum;
  byte RSSI;
  byte LQI2;
*/
  Pkt.len = radio_buff[0];
  memcpy(&Pkt.dest_addr, &radio_buff[1], 4);
  memcpy(&Pkt.src_addr, &radio_buff[5], 4);
  Pkt.port = radio_buff[9];
  Pkt.device_info = radio_buff[10];
  Pkt.txId = radio_buff[11];
  memcpy(&Pkt.raw, &radio_buff[12], 2);
  memcpy(&Pkt.filtered, &radio_buff[14], 2);
  Pkt.battery = radio_buff[16];
  Pkt.unknown = radio_buff[17];
  Pkt.checksum = radio_buff[18];
  Pkt.RSSI = radio_buff[19];
  Pkt.LQI2 = radio_buff[20];
#ifdef DEBUG
  Serial.print("Dexcom ID: ");
  Serial.println(Pkt.src_addr);
#endif
  return len;
}

boolean WaitForPacket(unsigned int milliseconds_wait, byte channel_index)
{
  unsigned long start_time;
  unsigned long current_time;
  boolean nRet = false;
  boolean packet_on_board;
  byte packet_len;

  start_time = millis();
  swap_channel(nChannels[channel_index], fOffset[channel_index]);

#ifdef DEBUG
  Serial.print("Chanel = ");
  Serial.print(nChannels[channel_index]);
  Serial.print(" Time = ");
  Serial.print(start_time);
  Serial.print(" Next Time = ");
  Serial.println(next_time);
#endif
  current_time = 0;
  while (true) {
    ESP.wdtFeed();
    current_time = millis();
    if (milliseconds_wait != 0 && current_time - start_time > milliseconds_wait) {
      break; // Если превысыли время ожидания на канале - выход
    }
    if (channel_index == 0 && next_time != 0 && current_time > (next_time + wait_after_time)) {
      break; // Если превысыли время следующего пакета на канале 0 - выход
    }
#ifdef INT_BLINK_LED
    blink_builtin_led_quarter();
#endif
    packet_on_board = false;
    while (digitalRead(GDO0_PIN) == HIGH) {
      packet_on_board = true;
      // Идет прием пакета
    }
    if (packet_on_board) {
      packet_len = ReadRadioBuffer();
      if (Pkt.src_addr == dex_tx_id) {
#ifdef DEBUG
        Serial.print("Catched.Ch=");
        Serial.print(nChannels[channel_index]);
        Serial.print(" Int=");
        if (catch_time != 0) {
          Serial.println(current_time - 500 * channel_index - catch_time);
        }
        else {
          Serial.println("unkn");
        }
#endif
        fOffset[channel_index] += ReadStatus(FREQEST);
        catch_time = current_time - 500 * channel_index; // Приводим к каналу 0
        nRet = true;
      } 
//      if (next_time != 0 && !nRet && channel_index == 0 && current_time < next_time && next_time-current_time < 2000) {
      if (next_time != 0 && !nRet && packet_len != 0) {
#ifdef DEBUG
        Serial.print("Try.Ch=");
        Serial.print(nChannels[channel_index]);
        Serial.print(" Time=");
        Serial.println(current_time);
#endif
        swap_channel(nChannels[channel_index], fOffset[channel_index]);
      }
      else {
        break;
      }
    }
  }

#ifdef INT_BLINK_LED
  digitalWrite(LED_BUILTIN, HIGH);
#endif
#ifdef EXT_BLINK_LED
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
#endif
  return nRet;
}

boolean get_packet (void) {
  byte nChannel;
  boolean nRet;

  nRet = false;
  for (nChannel = 0; nChannel < NUM_CHANNELS; nChannel++)
  {
    if (WaitForPacket (waitTimes[nChannel], nChannel)) {
      nRet = true;
      break;
    }
  }
  if (!nRet) {
    sequential_missed_packets++;
#ifdef DEBUG
    Serial.print("Missed-");
    Serial.println(sequential_missed_packets);
#endif
    if (sequential_missed_packets > misses_until_failure) { // Кол-во непойманных пакетов превысило заданное кол-во. Будем ловить пакеты непрерывно
      next_time = 0;
      sequential_missed_packets = 0; // Сбрасываем счетчик непойманных пакетов
    }  
  }
  else {
    next_time = catch_time; 
  }

  if (next_time != 0) {
    next_time += FIVE_MINUTE;
  }
  SendStrobe(SIDLE);
  SendStrobe(SFRX);

  return nRet;
}

void print_packet() {
  
  HTTPClient http;
  int httpCode;
  String response;
  byte i;
  String request;
  unsigned long ts;
  
#ifdef DEBUG
  Serial.print(Pkt.len, HEX);
  Serial.print("\t");
  Serial.print(Pkt.dest_addr, HEX);
  Serial.print("\t");
  Serial.print(Pkt.src_addr, HEX);
  Serial.print("\t");
  Serial.print(Pkt.port, HEX);
  Serial.print("\t");
  Serial.print(Pkt.device_info, HEX);
  Serial.print("\t");
  Serial.print(Pkt.txId, HEX);
  Serial.print("\t");
  Serial.print(dex_num_decoder(Pkt.raw));
  Serial.print("\t");
  Serial.print(dex_num_decoder(Pkt.filtered)*2);
  Serial.print("\t");
  Serial.print(Pkt.battery, HEX);
  Serial.print("\t");
  Serial.print(Pkt.unknown, HEX);
  Serial.print("\t");
  Serial.print(Pkt.checksum, HEX);
  Serial.print("\t");
  Serial.print(Pkt.RSSI, HEX);
  Serial.print("\t");
  Serial.print(Pkt.LQI2, HEX);
  Serial.println(" OK");
#endif
#ifdef INT_BLINK_LED    
  digitalWrite(LED_BUILTIN, LOW);
#endif
    // wait for WiFi connection
  i = 0;  
  WiFi.begin(my_wifi_ssid,my_wifi_pwd);
#ifdef DEBUG
    Serial.print("Connecting WiFi: ");
#endif
  while (WiFi.status() != WL_CONNECTED) {
#ifdef DEBUG
    Serial.print(".");
#endif
    delay(500);
    i++;
    if (i == wifi_wait_tyme*2) break;
  }
#ifdef DEBUG
  Serial.println();
#endif
  if((WiFi.status() == WL_CONNECTED)) {
/*    
    sprintf(radio_buff,"%s?rr=%lu&zi=%lu&pc=%s&lv=%lu&lf=%lu&db=%hhu&ts=%lu&bp=%d&bm=%d&ct=%d&gl=%s\" ",my_webservice_url,millis(),dex_tx_id,my_password_code,
                                                                                                        dex_num_decoder(Pkt.raw),dex_num_decoder(Pkt.filtered)*2,
                                                                                                        Pkt.battery,millis()-catch_time,0, 0, 37, "");         
                                                                                                        
    sprintf(radio_buff,"%s?rr=%lu&zi=%lu&pc=%s&lv=%lu&lf=%lu&db=%hhu&ts=%lu&bp=%d&bm=%d&ct=%d",my_webservice_url,millis(),dex_tx_id,my_password_code,
                                                                                                        dex_num_decoder(Pkt.raw),dex_num_decoder(Pkt.filtered)*2,
                                                                                                        Pkt.battery,millis()-catch_time,0, 0, 37);         
*/                                                                                                        
    ts = millis()-catch_time;  
    request = my_webservice_url;                                                                                                      
    request = request + "?rr=" + millis() + "&zi=" + dex_tx_id + "&pc=" + my_password_code +
              "&lv=" + dex_num_decoder(Pkt.raw) + "&lf=" + dex_num_decoder(Pkt.filtered)*2 + "&db=" + Pkt.battery +
              "&ts=" + ts + "&bp=0&bm=0&ct=37"; 
    http.begin(request); //HTTP
#ifdef DEBUG
    Serial.println(request);
#endif
    httpCode = http.GET();
    if(httpCode > 0) {
#ifdef DEBUG
      Serial.print("HTTPCODE = ");
      Serial.println(httpCode);
#endif
      if(httpCode == HTTP_CODE_OK) {
        response = http.getString();
#ifdef DEBUG
        Serial.print("RESPONSE = ");
        Serial.println(response);
#endif
      } else
      {
#ifdef INT_BLINK_LED    
        blink_sequence("0011");
#endif        
      }
    } else
    {
#ifdef INT_BLINK_LED    
      blink_sequence("0010");
#endif        
    }
    WiFi.disconnect(true);
  }
  else {
#ifdef INT_BLINK_LED    
    blink_sequence("0001");
#endif   
#ifdef DEBUG
    Serial.print("WiFi CONNECT ERROR = ");
    Serial.println(WiFi.status());
#endif   
  }
#ifdef INT_BLINK_LED    
  digitalWrite(LED_BUILTIN, HIGH);
#endif
}

void loop() {
  unsigned long current_time;
  
  if (next_time != 0) {
#ifdef DEBUG
    Serial.print("next tmime = ");
    Serial.print(next_time);
    Serial.print(" current time = ");
    Serial.print(millis());
    Serial.print(" interval = ");
    Serial.println(next_time - millis() - 3000);
#endif
    current_time = millis();
    if  (next_time > current_time && (next_time - current_time) < FIVE_MINUTE)  {

#ifdef DEBUG
      Serial.println("Delay");
#endif
      delay(next_time - current_time - 2000); // Можно спать до следующего пакета. С режимом сна будем разбираться позже
//      ESP.deepSleep((next_time - current_time - 2000)*1000,RF_DISABLED);
      
#ifdef DEBUG
      Serial.println("WakeUp");
#endif
    }
    else {
#ifdef DEBUG
      Serial.println("Timer overflow");
#endif
      next_time = 0;
    }
  }
  if (get_packet ())
  {
    print_packet ();
  }

}

