#define DEBUG
#define INT_BLINK_LED
//#define EXT_BLINK_LED
#define BLUETOOTH

#include <SPI.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
//#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#ifdef BLUETOOTH
  #include <SoftwareSerial.h>
#endif


extern "C" {
#include "user_interface.h"
}

#include "cc2500_REG.h"
#include "webform.h"

#define GDO0_PIN   D1            // Цифровой канал, к которму подключен контакт GD0 платы CC2500
#define LEN_PIN    D2            // Цифровой канал, к которму подключен контакт LEN (усилитель слабого сигнала) платы CC2500

#ifdef BLUETOOTH
  #define TX_PIN   D3            // Tx контакт для последовательного порта
  #define RX_PIN   D4            // Rx контакт для последовательного порта
#endif


#define NUM_CHANNELS (4)      // Кол-во проверяемых каналов
#define FIVE_MINUTE  300000    // 5 минут
#define THREE_MINUTE 180000    // 3 минуты
#define TWO_MINUTE   120000    // 2 минуты

#define RADIO_BUFFER_LEN 200 // Размер буфера для приема данных от GSM модема
#define SERIAL_BUFFER_LEN 100 // Размер буфера для приема данных от порта BT

#define my_webservice_url    "http://parakeet.esen.ru/receiver.cgi"
#define my_webservice_reply  "!ACK"
#define my_user_agent        "parakeet-8266"
#define my_password_code     "12543"

#define my_wifi_ssid         "ssid"
#define my_wifi_pwd          "password"

//ESP8266WiFiMulti WiFiMulti;

unsigned long dex_tx_id;
char transmitter_id[] = "6E853";

IPAddress local_IP(192,168,70,1);
IPAddress gateway(192,168,70,1);
IPAddress subnet(255,255,255,0);

ESP8266WebServer server(80);
unsigned long web_server_start_time;

#ifdef BLUETOOTH
  SoftwareSerial mySerial(RX_PIN, TX_PIN,false, 256); // RX, TX
#endif

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
byte default_bt_format = 0; // Формат обмена по протколу BlueTooth 0 - None 1 - xDrip, 2 - xBridge
byte old_bt_format;

char radio_buff[RADIO_BUFFER_LEN]; // Буффер для чтения данных и прочих нужд
char serial_buff[SERIAL_BUFFER_LEN]; // Буффер для чтения данных и прочих нужд
volatile boolean wake_up_flag;

// defines the xBridge protocol functional level.  Sent in each packet as the last byte.
#define DEXBRIDGE_PROTO_LEVEL (0x01)

// Коды ошибок мигают лампочкой в двоичной системе
// 1 (0001) - Нет модключения к WiFi
// 2 (0010) - Облачная служба не отвечает
// 3 (0011) - Облачная служба возвращает ошибку
// 4 (0100) - Неверный CRC в сохраненных настройках. Берем настройки по умолчанию


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

typedef struct _RawRecord
{
  byte size; //size of the packet.
  byte cmd_code; // code for this data packet.  Always 00 for a Dexcom data packet.
  unsigned long  raw;  //"raw" BGL value.
  unsigned long  filtered; //"filtered" BGL value 
  byte dex_battery;  //battery value
  byte my_battery; //xBridge battery value
  unsigned long  dex_src_id;   //raw TXID of the Dexcom Transmitter
  //int8  RSSI; //RSSI level of the transmitter, used to determine if it is in range.
  //uint8 txid; //ID of this transmission.  Essentially a sequence from 0-63
  byte function; // Byte representing the xBridge code funcitonality.  01 = this level.
} RawRecord;

typedef struct _parakeet_settings
{
  unsigned long dex_tx_id;     //4 bytes
  char http_url[56];
  char password_code[6];
  char wifi_ssid[17];
  char wifi_pwd[18];
  byte bt_format;
  unsigned long checksum; // needs to be aligned

} parakeet_settings;

parakeet_settings settings;

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

void clearSettings()
{
  memset (&settings, 0, sizeof (settings));
  settings.dex_tx_id = asciiToDexcomSrc (transmitter_id);
  dex_tx_id = settings.dex_tx_id;
  sprintf(settings.http_url, my_webservice_url);
  sprintf(settings.password_code, my_password_code);
  sprintf(settings.wifi_ssid, my_wifi_ssid);
  sprintf(settings.wifi_pwd, my_wifi_pwd);
  settings.bt_format = default_bt_format;
  settings.checksum = 0;
}

unsigned long checksum_settings()
{
  char* flash_pointer;
  unsigned long chk = 0x12345678;
  byte i;
  //   flash_pointer = (char*)settings;
  flash_pointer = (char*)&settings;
  for (i = 0; i < sizeof(parakeet_settings) - 4; i++)
  {
    chk += (flash_pointer[i] * (i + 1));
    chk++;
  }
  return chk;
}

void saveSettingsToFlash()
{
  char* flash_pointer;
  byte i;

  EEPROM.begin(sizeof(parakeet_settings));
  
  settings.checksum = checksum_settings();
  flash_pointer = (char*)&settings;
  for (i = 0; i < sizeof(parakeet_settings); i++)
  {
    EEPROM.write(i,flash_pointer[i]);
  }
  EEPROM.commit();
//  EEPROM.put(0, settings);
}

void loadSettingsFromFlash()
{
  char* flash_pointer;
  byte i;

  EEPROM.begin(sizeof(parakeet_settings));
  flash_pointer = (char*)&settings;
  for (i = 0; i < sizeof(parakeet_settings); i++)
  {
    flash_pointer[i] = EEPROM.read(i);
  }
  
//  EEPROM.get(0, settings);
  dex_tx_id = settings.dex_tx_id;
  if (settings.checksum != checksum_settings()) {
    clearSettings();
#ifdef INT_BLINK_LED
    blink_sequence("0100");
#endif
#ifdef EXT_BLINK_LED
    blink_sequence_red("0100");
#endif
  }
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

void handleRoot() {
  char current_id[6];
  char temp[1400];
  char chk1[8];
  char chk2[8];
  char chk3[8];
  dexcom_src_to_ascii(settings.dex_tx_id,current_id);
  switch (settings.bt_format) {
    case 0:
      sprintf(chk1,"%s","checked");
      chk2[0] = '\0';
      chk3[0] = '\0';
      break;
    case 1:
      chk1[0] = '\0';
      sprintf(chk2,"%s","checked");
      chk3[0] = '\0';
      break;
    case 2:
      chk1[0] = '\0';
      chk2[0] = '\0';
      sprintf(chk3,"%s","checked");
      break;
    default:  
      chk1[0] = '\0';
      chk2[0] = '\0';
      chk3[0] = '\0';
      break;
  } 
  sprintf(temp,edit_form,current_id,settings.password_code,settings.http_url,settings.wifi_ssid,settings.wifi_pwd,chk1,chk2,chk3);
  server.send(200, "text/html", temp);
}

void handleNotFound() {
  server.send ( 404, "text/plain", "not found!" );
}

void handleSave() {
  char new_id[6]; 
  String arg1;
  char temp[1400];
  char bt_frmt[8];

  arg1 = server.arg("DexcomID");
  arg1.toCharArray(new_id,6);
  settings.dex_tx_id = asciiToDexcomSrc (new_id);
  dex_tx_id = settings.dex_tx_id;
  arg1 = server.arg("PasswordCode");
  arg1.toCharArray(settings.password_code,6);
  arg1 = server.arg("WebService");
  arg1.toCharArray(settings.http_url,56);
  arg1 = server.arg("WiFiSSID");
  arg1.toCharArray(settings.wifi_ssid,16);
  arg1 = server.arg("WiFiPwd");
  arg1.toCharArray(settings.wifi_pwd,16);
  arg1 = server.arg("BtFormat");
  if (arg1 == "0") {
    settings.bt_format = 0;
    sprintf(bt_frmt,"%s","None");
  }
  else if (arg1 == "1") {   
    settings.bt_format = 1;
    sprintf(bt_frmt,"%s","xDrip");
  } else if (arg1 == "2") {    
    settings.bt_format = 2;
    sprintf(bt_frmt,"%s","xBridge");
  }
  saveSettingsToFlash();
  
  sprintf(temp, "Configuration saved!<br>DexcomID = %s<br>Password Code = %s<br>URL = %s<br>WiFi SSID = %s<br>WiFi Password = %s<br> BlueTooth format: %s<br>",
                new_id,settings.password_code,settings.http_url,settings.wifi_ssid,settings.wifi_pwd,bt_frmt);
  server.send ( 200, "text/html",temp );
//  server.send ( 200, "text/plain","Configuration saved!" );
#ifdef DEBUG
  Serial.println("Configuration saved!");
#endif      
}

void PrepareWebServer() {
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP("Parakeet");
  server.on("/", handleRoot);
  server.on("/save", handleSave);
  server.onNotFound ( handleNotFound );
  server.begin(); 
  web_server_start_time = millis();   
}

#ifdef BLUETOOTH
boolean bt_command(const char *command, const char *response, int timeout) {
  return bt_command(command,strlen(command),response,timeout);
}

boolean bt_command(const char *command, byte len, const char *response, int timeout) {
  boolean ret;
  unsigned long timeout_time; 
  int len_cmd = strlen (command);
  int len_resp = strlen (response);
  int loop = 0;
  byte i;


  if (response == "") {
    ret = true;
  } 
  else {
    ret = false;
  }  

  memset (&serial_buff[0],0,sizeof(serial_buff));

  if (len > 0) {
    len_cmd = len;
  }
  for (i = 0; i < len_cmd; i++) {
    mySerial.write(command[i]);
#ifdef DEBUG
    Serial.print(command[i],HEX);
#endif
  }
#ifdef DEBUG
  Serial.println();
#endif
  timeout_time = timeout;
  timeout_time = millis() + (timeout_time * 1000);
  while (millis() < timeout_time)
  {
    ESP.wdtFeed();
    if (mySerial.available()) {
      delayMicroseconds(100);
      serial_buff[loop] = mySerial.read();
      loop++;
      if (loop == SERIAL_BUFFER_LEN) loop = 0; // Контролируем переполнение буфера
      if (loop >= len_resp) {
        if (strncmp(response,&serial_buff[loop-len_resp],len_resp) == 0) {
//        if (memcmp(response,&serial_buff[loop-len_resp],len_resp) == 0) {
          ret = true;
          delay(100);
        }
      }  
    } 
    else {
      if (ret) {
        delayMicroseconds(100);
        break;
      }
    }
  }
#ifdef DEBUG
  Serial.print("Cmd = ");
  Serial.println(command);
  Serial.print("Cmd Len = ");
  Serial.println(len_cmd);
  Serial.print("Exp.resp = ");
  Serial.println(response);
  Serial.print("Exp.resp Len = ");
  Serial.println(len_resp);
  Serial.print("Resp = ");
  Serial.println(serial_buff);
  Serial.print("Res = ");
  Serial.println(ret);
#endif

  if (settings.bt_format == 2 && strlen(radio_buff) > 0) {
    if (radio_buff[0]== 0x06 && radio_buff[1] == 0x01) {
#ifdef DEBUG
      Serial.println("Processing TXID packet");
#endif      
    }
    if (radio_buff[0]== 0x02 && radio_buff[1] == 0xF0) {
#ifdef DEBUG
      Serial.println("Processing ACK packet");
#endif      
      ret = true;
    }
  }
  return ret;
}

void sendBeacon()
{
  //char array to store the response in.
  char cmd_response[8];
  //return if we don't have a connection or if we have already sent a beacon
  cmd_response[0] = 0x07;
  cmd_response[1] = 0xF1;
  memcpy(&cmd_response[2], &settings.dex_tx_id, sizeof(settings.dex_tx_id));
  cmd_response[6] = DEXBRIDGE_PROTO_LEVEL;
  cmd_response[7] = '\0';
  bt_command(cmd_response,7,"OK",2);
}

void PrepareBlueTooth() {

   if (settings.bt_format == 0) return;
   mySerial.begin(9600);
   delay(500);
   bt_command("AT","OK",2);
   delay(500);
   if (settings.bt_format == 1) {
     bt_command("AT+NAMExDrip","Set:",2);
   } 
   else if (settings.bt_format == 2) {   
     bt_command("AT+NAMExBridge","Set:",2);
   }
   delay(500);
   bt_command("AT+RESET","RESET",2);
}
#endif

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
  pinMode(LEN_PIN, OUTPUT);
  pinMode(GDO0_PIN, INPUT);
  // initialize digital pin LED_BUILTIN as an output.
#ifdef INT_BLINK_LED
  pinMode(LED_BUILTIN, OUTPUT);
#endif
#ifdef EXT_BLINK_LED
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
#endif

  loadSettingsFromFlash();
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
  PrepareWebServer();
 
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);
#ifdef BLUETOOTH
  PrepareBlueTooth();
  old_bt_format = settings.bt_format;
#endif
#ifdef DEBUG
  Serial.println("Wait two minutes or configure device!");
#endif
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
  digitalWrite(LEN_PIN, HIGH); // Включаем усилитель слабого сигнала
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
  digitalWrite(LEN_PIN, LOW); // Выключаем усилитель слабого сигнала

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

  if (strlen(settings.wifi_ssid) == 0) {
#ifdef DEBUG
    Serial.println("WiFi not configred!");
#endif
    return;
  }
#ifdef INT_BLINK_LED    
  digitalWrite(LED_BUILTIN, LOW);
#endif
    // wait for WiFi connection
  i = 0;  
  WiFi.begin(settings.wifi_ssid,settings.wifi_pwd);
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

#ifdef BLUETOOTH
void print_bt_packet() {
  RawRecord msg;  

  if (settings.bt_format == 0) {
    return;
  }
//  sprintf(dex_data,"%lu %d %d",275584,battery,3900);
#ifdef INT_BLINK_LED    
  digitalWrite(LED_BUILTIN, LOW);
#endif
  if (settings.bt_format == 1) {
    sprintf(radio_buff,"%lu %d",dex_num_decoder(Pkt.raw),Pkt.battery);
    bt_command(radio_buff,0,"OK",2);
  }  
  else if (settings.bt_format == 2) { 
    msg.cmd_code = 0x00;
    msg.raw = dex_num_decoder(Pkt.raw);
    msg.filtered = dex_num_decoder(Pkt.filtered)*2;
    msg.dex_battery = Pkt.battery;
//    msg.my_battery = battery_capacity;
    msg.my_battery = 0;
    msg.dex_src_id = Pkt.src_addr;
//    msg.size = sizeof(msg);
    msg.size = 17;
    msg.function = DEXBRIDGE_PROTO_LEVEL; // basic functionality, data packet (with ack), TXID packet, beacon packet (also TXID ack).
//    memcpy(&radio_buff, &msg, sizeof(msg));

    radio_buff[0] = msg.size;
    radio_buff[1] = msg.cmd_code;
    memcpy(&radio_buff[2],&msg.raw , 4);
    memcpy(&radio_buff[6],&msg.filtered , 4);
    radio_buff[10] = msg.dex_battery;
    radio_buff[11] = msg.my_battery;
    memcpy(&radio_buff[12],&msg.dex_src_id , 4);
    radio_buff[16] = msg.function;
    radio_buff[sizeof(msg)] = '\0';
    bt_command(radio_buff,msg.size,"OK",2);
  }
#ifdef INT_BLINK_LED    
  digitalWrite(LED_BUILTIN, HIGH);
#endif
}
#endif

void my_wakeup_cb() {
  wake_up_flag = true;
#ifdef DEBUG
  Serial.println("WakeUp callback");
#endif 
}
bool light_sleep(unsigned long time_ms) {

  wake_up_flag = false;
  wifi_station_disconnect();
  wifi_set_opmode(NULL_MODE);  
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  wifi_fpm_open();
//    wifi_fpm_set_sleep_type(MODEM_SLEEP_T);
  wifi_fpm_set_wakeup_cb(my_wakeup_cb);
  wifi_fpm_do_sleep(time_ms*1000) ; 
//  sleep_time = time_ms*960;
//  wifi_fpm_do_sleep(sleep_time) ; 
  delay(time_ms);
/*  
  while (!wake_up_flag) {
    delay(500);
  }
*/  
  wifi_fpm_do_wakeup();
  wifi_fpm_close();
  wifi_set_opmode(STATION_MODE);         // set station mode
}

void loop() {
  unsigned long current_time;

// Первые две минуты работает WebServer на адресе 192.168.70.1 для конфигурации устройства
  if (web_server_start_time > 0) {
    server.handleClient();
    if ((millis() - web_server_start_time) > TWO_MINUTE && !server.client()) {
      server.stop();
      WiFi.softAPdisconnect(true);
      web_server_start_time = 0;  
#ifdef DEBUG
      Serial.println("Configuration mode is done!");
#endif
#ifdef BLUETOOTH
      if (old_bt_format != settings.bt_format) {
        PrepareBlueTooth();
        delay(500);
      }
      if (settings.bt_format == 2) {
        sendBeacon();
      }   
#endif
    }
    return;
  }
  
// После пяти минут ожидания ждем сигнал с декскома
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
      Serial.println("GoTo sleep");
      delay(500);
      current_time = millis();
#endif
      if ((next_time - current_time) > THREE_MINUTE) {       
        light_sleep(THREE_MINUTE);
#ifdef DEBUG
        current_time = millis();
        Serial.print("Time = ");
        Serial.println(current_time);
#endif
        delay(500);
        current_time = millis();
      }
      light_sleep(next_time - current_time - 2000);
//      delay(next_time - current_time - 2000); // Можно спать до следующего пакета. С режимом сна будем разбираться позже
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
#ifdef BLUETOOTH
    print_bt_packet();
#endif
    print_packet ();
  }

}

