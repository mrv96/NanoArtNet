//http://artisticlicence.com/WebSiteMaster/User%20Guides/art-net.pdf
//https://art-net.org.uk/structure/streaming-packets/artdmx-packet-definition/
//https://art-net.org.uk/structure/universe-addressing/
//https://art-net.org.uk/structure/discovery-packets/artpollreply/

//#include <EtherCard.h>  //If you need to reduce total used memory delete all Serial call in the library.
#include "EtherCard.h" //previous comment DONE!
#include <DmxSimple.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SoftReset.h>
#include <TimerOne.h>

#define HARD_RESET_MEM_PAGE 0
#define RESET_WAIT_TIME 5
#define RESET_TIMES 3
#define DMX_DIR 4
#define RESET_BUTTON 9

//EtherCardConfig
#define DST_PORT  6454  // standard Art-Net port
#define SRC_PORT  6454
#define TIMEOUT   30000 // after this ms use static config and not DHCP

// ID of the settings block
#define CONFIG_VERSION "LM1"
// Tell it where to store your config data in EEPROM
#define CONFIG_START 32

typedef struct
{
  uint8_t mymac[6];
  uint8_t myip[4];
  uint8_t net[4];
  uint8_t gwip[4];
  uint16_t universe;
  boolean dhcp=false;
} sysConf;

sysConf configuration;
uint8_t Ethernet::buffer[700];  //700 should be enought for UDP ArtNet DMX packet (probably even less)

//Art-Net Config
#define OEM 0x00FF
#define MAX_UNIVERSE 32767 //given by ArtDMX standard

//Misc Config
#define DEBUG 0 // set for debug information via serial monitor, Baud rate 115200
#define PAYLOAD_SIZE 239

#define HIGH_BYTE(x) ((x >> 8) & 0xFF)
#define LOW_BYTE(x) (x & 0xFF)
#define ARTNET_HEADER 'A','r','t','-','N','e','t','\0'  // Art-Net header for identifying Art-Net packets
char ArtNetHeader[8] = "Art-Net\0"; // Art-Net header for identifying Art-Net packets

//Node Settings
#define OPCODE 0x2100 // sets the OpCode "ArtPollReply" (don't change!)
#define ARTNET_VERSION 0x000E // sets the Art-Net protocol version (don't change!)
#define UBEA_VERSION 0x00 // sets the firmware version of the User BIOS Extension Area (don't change!)
#define STATUS1 B11000000 // sets the status of the node, part 1
#define ESTA_MAN_CODE 0x7FFF // sets the Entertainment Service and Technology Association manufacturer code (don't change!)
#define SHORT_NAME 'L','E','A','M',' ','A','r','t','-','N','e','t',' ','N','o','d','e',0 // sets the short (max 17 char + \0) name for the node
// sets the long (max 63 char + \0) name for the node
#define LONG_NAME 'L','E','A','M',' ','A','r','t','-','N','e','t',' ','N','o','d','e',' ','2','0','1','9',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
// sets debug information string (max 63 char + \0), not used
#define NODE_REPORT '#','0','0','0','1',' ','[','0','0','0','0',']',' ','L','E','A','M',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
#define NUM_PORTS 1  // sets the number of max input or output ports (0-4)
#define PORT_TYPES B10000000,0,0,0  // sets the port types (input/output, type of data)
#define GOOD_INPUT  0,0,0,0 // sets the input port status (don't change!)
#define GOOD_OUTPUT 0,0,0,0 // sets the output port status and configuration (don't change!)
#define SW_IN 0,0,0,0 // sets the universe for each input port
#define SW_OUT 0,0,0,0 // sets the univers for each output port
#define STYLE 0x00 // sets the style code for the node, deprecated
#define BIND_INDEX 0x00 // sets the distance to the root device, 0 or 1 means this is the root device (don't change!)
#define STATUS2 0x0f  // sets the status of the node, part 2

void setup () {
//  defaultConfig();
//  saveConfig();
  pinMode(DMX_DIR, OUTPUT);
  digitalWrite(DMX_DIR, HIGH);

  pinMode(RESET_BUTTON, INPUT);
  digitalWrite(RESET_BUTTON, HIGH);
  if(digitalRead(RESET_BUTTON)==LOW) {
    delay(100);
    if(digitalRead(RESET_BUTTON)==LOW) {  
      if(EEPROM.read(HARD_RESET_MEM_PAGE)!=0) EEPROM.write(HARD_RESET_MEM_PAGE, 0);
      defaultConfig();
      saveConfig();
    }
  }

  if(EEPROM.read(HARD_RESET_MEM_PAGE)>=RESET_TIMES) {
    EEPROM.write(HARD_RESET_MEM_PAGE, 0);
    defaultConfig();
    saveConfig();
  } else {
    Timer1.initialize(1000000);
    Timer1.attachInterrupt(handleHardReset);
    Timer1.start();
  }
  
#if DEBUG
  Serial.begin(115200); //initialises serial monitor
#endif

  if(loadConfig()==-1) {
    defaultConfig();
    saveConfig();
  }

  if(!ether.begin(sizeof Ethernet::buffer, configuration.mymac)){ // initialises ethernet controller
#if DEBUG
    Serial.println(F("Failed to access Ethernet controller!"));
#endif
  }
  
  if(configuration.dhcp==false) {
#if DEBUG
    Serial.print(F("Setting IP statically."));
#endif
    ether.staticSetup(configuration.myip, configuration.gwip, 0, configuration.net); // intialises ethernet controller for static IP
  } else {
#if DEBUG
    Serial.print(F("Setting IP via DHCP."));
#endif
    unsigned long ms = millis();
    while(!ether.dhcpSetup()){ // intialises ethernet controller for dynamic IP
#if DEBUG
      Serial.println(F("DHCP failed!"));
#endif
      if(millis()-ms>=TIMEOUT) {
        //defaultConfig();
        ether.staticSetup(configuration.myip, configuration.gwip, 0, configuration.net); // intialises ethernet controller for static IP
        break;
      }
    }
  }
  ether.enableBroadcast(); // enables braodcasting UDP packets
  ether.udpServerListenOnPort(&processArtNetPacket, DST_PORT);

  //init DMX - not needed because DmxSimple initialize the library automatically in write method
//  DmxSimple.usePin(3);          // DMX output is pin 3
  DmxSimple.maxChannel(512);    //should be 512
}

const PROGMEM char payload[PAYLOAD_SIZE] = 
                                  {ARTNET_HEADER, LOW_BYTE(OPCODE), HIGH_BYTE(OPCODE), 0, 0, 0, 0, LOW_BYTE(DST_PORT), HIGH_BYTE(DST_PORT), HIGH_BYTE(ARTNET_VERSION), LOW_BYTE(ARTNET_VERSION),
                                   0, 0, HIGH_BYTE(OEM), LOW_BYTE(OEM), UBEA_VERSION, STATUS1, LOW_BYTE(ESTA_MAN_CODE), HIGH_BYTE(ESTA_MAN_CODE), SHORT_NAME, LONG_NAME, NODE_REPORT,
                                   HIGH_BYTE(NUM_PORTS), LOW_BYTE(NUM_PORTS), PORT_TYPES, GOOD_INPUT, GOOD_OUTPUT, SW_IN, SW_OUT, 0, 0, 0, 0, 0, 0, STYLE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   BIND_INDEX, STATUS2};

void processArtNetPacket(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, const char *data, uint16_t len){
#if DEBUG
  Serial.print(F("UDP packet with length "));
  Serial.print(len);
  Serial.println(F(" received."));
#endif
  if((len > 13) && (len <= 576)){ // checks if packet has the correct length
    //bool isArtNet = true;
    for(uint8_t i = 0;i < 8;i++){ // checks if the packet is an Art-Net packet
      if(data[i] != ArtNetHeader[i]){
        //isArtNet = false;
        return;
      }
    }
    //if(isArtNet){
      uint16_t OpCode = ((data[9] << 8) & 0xFF00) + (data[8] & 0xFF);
#if DEBUG
      Serial.print(F("Opcode: "));
      Serial.println(OpCode,HEX);
#endif
      if(OpCode == 0x5000){ // checks if Art-Net packet is an ArtDMX packet
#if DEBUG
        Serial.println(F("ArtDMX received."));
#endif
        if((data[14] == (const char)LOW_BYTE(configuration.universe)) && (data[15] == (const char)HIGH_BYTE(configuration.universe))){ // checks if Art-Net packets is addressed to this node
          uint16_t dmxDataLength = (data[16] << 8) | data[17];
          for(uint16_t i = 1; i <= dmxDataLength; i++){
            DmxSimple.write(i,(uint8_t)data[i+17]); //Ethernet::buffer[59+i] should be the same thing
          }
#if DEBUG
          Serial.println(F("Dmx channels sent."));
#endif
        }
      }
      else if(OpCode == 0x2000){ // checks if Art-Net packet is an ArtPoll packet
#if DEBUG
        Serial.println(F("ArtPoll received."));
#endif
        sendArtPollReply();
#if DEBUG
        Serial.println(F("ArtPollReply sent."));
#endif
      }
    //}
  }
}

void sendArtPollReply(){
#if DEBUG
  Serial.println(F("Sending ArtPollReply..."));
#endif
  //memorizzare in ether.buffer (partendo da ether.buffer + UDP_DATA_P) (NOTA: UDP_DATA_P = 0x2a)
  for(int i=0; i<PAYLOAD_SIZE; i++){
    if(i>=10 && i<=13)
      ether.buffer[i+UDP_DATA_P] = ether.myip[i-10];
    else if(i==18) // sets the Net part of the port address (0x00-0x7F)      
      ether.buffer[i+UDP_DATA_P] = HIGH_BYTE(configuration.universe);
    else if(i==19) // sets the Sub-Net part of the port address (0x00-0x0F)
      ether.buffer[i+UDP_DATA_P] = LOW_BYTE(configuration.universe)>>4;
    else if(i>=201 && i<=206)
      ether.buffer[i+UDP_DATA_P] = ether.mymac[i-201];
    else if(i>=207 && i<=210)
      ether.buffer[i+UDP_DATA_P] = ether.myip[i-207];
    else
      ether.buffer[i+UDP_DATA_P] = pgm_read_word_near(payload + i);
  }
  ether.sendUdp(ether.buffer + UDP_DATA_P, /*sizeof(payload)*/PAYLOAD_SIZE, SRC_PORT, ether.broadcastip, DST_PORT); // sends UDP packet with Art-Net (ArtPoll) data
}

void loop () {
  word pos = ether.packetLoop(ether.packetReceive()); // waits for incoming UDP packets and assign pos
  if(pos) {
    if (strncmp_P((char *)Ethernet::buffer + pos, PSTR("GET / "),  6) == 0) {
      // Page emmited
      sendStatusPage();
    }
    else if (strncmp_P((char *)Ethernet::buffer + pos, PSTR("GET /ajax_"), 10) == 0) { //ci andrebbe ajax_inputs ma ho bisogno di risparmiare caratteri
      ether.httpServerReply(0);
      if (strncmp_P((char *)Ethernet::buffer + pos + 10, PSTR("dft"), 3) == 0) {
        defaultConfig();
        saveConfig();
        soft_restart();
      } else if(strncmp_P((char *)Ethernet::buffer + pos + 10, PSTR("dhcp"), 4) == 0) {
        configuration.dhcp = !configuration.dhcp;
        saveConfig();
        soft_restart();
      } else saveAndReboot((char *)Ethernet::buffer + pos + 10);
    }
    else ether.httpServerReply(0);
  }
}

const char statusPage1[] PROGMEM =
"HTTP/1.0 200 OK\r\n"
"Content-Type: text/html\r\n"
"\r\n"
"<html>"
  "<head><meta charset=\"utf-8\"><title>"
    "ArtNet Node Configuration"
  "</title>"
  "<link rel=\"icon\" href=\"data:;base64,iVBORw0KGgo=\">"
  "<script>"
        "var strText;"        
        "function SendText(n)"
        "{"
            "var request=new XMLHttpRequest();"
            
            "if(n===0){"
              "if(document.getElementById('DHCP').checked===true)strText=\"dhcp\";";

const char statusPage2[] PROGMEM =
              "else if(n===0)strText=document.getElementById('MAC').value+document.getElementById('IP').value+\".\"+"
                           "document.getElementById('NET').value+\".\"+document.getElementById('GW').value+\".\"+document.getElementById('UNI').value+\".\";"
            "}else if(n===1)strText=\"dft\";"

            "document.getElementById('done').innerHTML=\"DONE, PLEASE REFRESH!\";"
            
            "request.open(\"GET\", \"ajax_\" + strText, true);"
            "request.send(null);"
        "}"
        "</script>"
  "</head>";

const char statusPage3[] PROGMEM =
  "<body>"
    "<h2>Network Configuration</h2>"
    "<p>MAC Address:&emsp;<input type=\"text\" id=\"MAC\" value=\"%02X:%02X:%02X:%02X:%02X:%02X\" maxlength=\"17\" size=\"14\"/></p>"
    "<p>IP Address:&thinsp;&emsp;&emsp;&nbsp;<input type=\"text\" id=\"IP\" value=\"%d.%d.%d.%d\" maxlength=\"15\" size=\"11\"/></p>"
    "<p>Net Mask:&thinsp;&emsp;&emsp;&ensp;&nbsp;<input type=\"text\" id=\"NET\" value=\"%d.%d.%d.%d\" maxlength=\"15\" size=\"11\"/></p>"
    "<p>Gateway:&emsp;&emsp;&emsp;&nbsp;<input type=\"text\" id=\"GW\" value=\"%d.%d.%d.%d\" maxlength=\"15\" size=\"11\"/></p>";

const char statusPage4[] PROGMEM =
    "<p>Universe:&thinsp;&emsp;&emsp;&ensp;&ensp;<input type=\"number\" id=\"UNI\" value=\"%d\" min=\"0\" max=\"32767\"/> (max. 32767)</p>"
    "<p><input type=\"checkbox\" id=\"DHCP\" name=\"dhcp\"%s/>USE DHCP</p>"
    "<p><input type=\"submit\" value=\"SAVE & REBOOT\" onclick=\"SendText(0)\"/></p>"
    "<p><input type=\"submit\" value=\"RESTORE DEFAULT\" onclick=\"SendText(1)\"/></p>"
    "<p id=\"done\"></p>"
  "</body>"
"</html>";

void sendStatusPage() {  
  ether.httpServerReplyAck(); // send ack to the request
  //memcpy_P(ether.tcpOffset(), pageA, sizeof pageA); // send first packet and not send the terminate flag
  uint16_t len = sprintf_P((char*)ether.tcpOffset(), statusPage1);
  ether.httpServerReply_with_flags(len, TCP_FLAGS_ACK_V);

  len = sprintf_P((char*)ether.tcpOffset(), statusPage2);
  ether.httpServerReply_with_flags(len, TCP_FLAGS_ACK_V);

  len = sprintf_P((char*)ether.tcpOffset(), statusPage3,
      ether.mymac[0], ether.mymac[1], ether.mymac[2], ether.mymac[3], ether.mymac[4], ether.mymac[5],
      ether.myip[0], ether.myip[1], ether.myip[2], ether.myip[3],
      ether.netmask[0], ether.netmask[1], ether.netmask[2], ether.netmask[3],      
      ether.gwip[0], ether.gwip[1], ether.gwip[2], ether.gwip[3]);
  ether.httpServerReply_with_flags(len, TCP_FLAGS_ACK_V);

  if(configuration.dhcp==false){
    len = sprintf_P((char*)ether.tcpOffset(), statusPage4, configuration.universe, "");
  }
  else{
    char str[9];
    strcpy_P(str, PSTR(" checked"));
    len = sprintf_P((char*)ether.tcpOffset(), statusPage4, configuration.universe, str);
  }
  ether.httpServerReply_with_flags(len, TCP_FLAGS_ACK_V|TCP_FLAGS_FIN_V);
}

void saveAndReboot(char *dataIn) {
  char str[6];
  uint8_t *ptr = &configuration.mymac[0];
  uint8_t i;

  //MAC UPDATE
  for(i = 0; i < 6*3; i=i+3) {
    strncpy(str, &dataIn[i], 2);
    str[2]=0;
    *ptr++ = strtol(str, NULL, 16);
  }

  //UPDATE THE REMAINING ADDRESSES
  uint8_t cnt = 0, i2 = i - 1;
  i = 0;
  do{
    if((str[i] = dataIn[i2++]) == '.') {
      str[i] = 0;
      i = 0;
      cnt++;
      *ptr++ = atoi(str);
    }
    else i++;
  }while(cnt<12);

  //UPDATE UNIVERSE
  while((str[i++] = dataIn[i2++]) != '.');
  str[i-1] = 0;
  configuration.universe = atoi(str)%(MAX_UNIVERSE+1);

  saveConfig();
  soft_restart();
}

int8_t loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
  {
    for (unsigned int t=0; t<sizeof(configuration); t++)
      *((char*)&configuration + t) = EEPROM.read(CONFIG_START+3 + t);
    return 0;
  } else return -1;
}

void saveConfig() {
  EEPROM.write(CONFIG_START + 0, CONFIG_VERSION[0]);
  EEPROM.write(CONFIG_START + 1, CONFIG_VERSION[1]);
  EEPROM.write(CONFIG_START + 2, CONFIG_VERSION[2]);
  for (unsigned int t=0; t<sizeof(configuration); t++)
    EEPROM.write(CONFIG_START+3 + t, *((char*)&configuration + t));
}

void defaultConfig() {
  //MAC ADDRESS must be different for each device in the same network
  configuration.mymac[0] = 0xFA;
  configuration.mymac[1] = 0xFA;
  configuration.mymac[2] = 0xFA;
  configuration.mymac[3] = 0x01;
  configuration.mymac[4] = 0x01;
  configuration.mymac[5] = 0x01;
  //calculates default static IP address as per Art-Net protocol, with previous MAC->2.0.1.1
  configuration.myip[0] = 2;
  configuration.myip[1] = configuration.mymac[3] + OEM; // 
  configuration.myip[2] = configuration.mymac[4];       // 
  configuration.myip[3] = configuration.mymac[5];       // 
  configuration.gwip[0] = 2; // calculates default gateway address as per Art-Net protocol
  configuration.gwip[1] = 0; //
  configuration.gwip[2] = 0; //
  configuration.gwip[3] = 1; //
  configuration.net[0] = 255; // calculates default subnet mask as per Art-Net protocol
  configuration.net[1] = 0;   //
  configuration.net[2] = 0;   //
  configuration.net[3] = 0;   //
  configuration.universe = 0;
  configuration.dhcp=false;
}

void handleHardReset() {
  static uint8_t sec = 1;
  if(sec==RESET_WAIT_TIME-1) EEPROM.write(HARD_RESET_MEM_PAGE, EEPROM.read(HARD_RESET_MEM_PAGE)+1);
  else if(sec==RESET_WAIT_TIME+1) {
    if(EEPROM.read(HARD_RESET_MEM_PAGE)!=0) EEPROM.write(HARD_RESET_MEM_PAGE, 0);
    Timer1.detachInterrupt();
    Timer1.stop();
  }
  sec++;
}
