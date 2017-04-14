/*
  WiFi UDP Send and Receive String

  This sketch wait an UDP packet on localPort using a WiFi shield.
  When a packet is received an Acknowledge packet is sent to the client on port remotePort

  Circuit:
   WiFi shield attached

  created 30 December 2012
  by dlf (Metodo2 srl)

*/

#include "masonbot.h"
#include <stdio.h>
#include <stdlib.h>
#include <TimerOne.h>

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

const int TEST = 88;   //Battery test

int battery = 6;
float batterylevel = 0;

int count1 = 0;
int count2 = 0;
int count3 = 0;

int status = WL_IDLE_STATUS;
//char ssid[] = "Verizon-SM-G930V-6155"; //  your network SSID (name) // IP address 192.168.43.95(may change)
//char pass[] = "jasonwifi";

char ssid[] = "QCHJB"; //  your network SSID (name) // IP address 192.168.43.95(may change)
char pass[] = "robotsarecool2!";

int keyIndex = 1;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet

char  ReplyBuffer[] = "ACK0";       // a string to send back
int microseconds = 50000;
//**********************Added for 3 digit velocity
char vel[2]; //buffer for pwm input
int velocity = 0;
//************************************

WiFiUDP Udp;

void setup() {
  MasonBot();
  MasonBot().moveStop();
  //WiFi.setPins(41, 45, 47, 43);
  pinMode(A1, INPUT);
  pinMode(43, OUTPUT);

  // DO NOT CHANGE THIS!!!!
  digitalWrite(43, HIGH);
  WiFi.setPins(41, 45, 47);
  /*DDRC = 0xFF;
  PORTC = (TCCR3A&0xC0) >> 5;
  delay(2000);
  PORTC = (TCCR3A&0x30) >> 3;
  delay(2000);
  PORTC = (TCCR3A&0x0C) >> 1;
  delay(2000);
   
   PORTC = (TCCR4A&0xC0) >> 5;
  delay(2000);
  PORTC = (TCCR4A&0x30) >> 3;
  delay(2000);
  PORTC = (TCCR3A&0x0C) >> 1;
  delay(2000);*/
  /*
  PORTC = (TCCR3C&7) <<2;
  delay(1000);
  PORTC = (TCCR4A&7) <<2;
  delay(1000);
  PORTC = (TCCR4B&7) <<2;
  delay(1000);
  PORTC = (TCCR4C&7) <<2;
  delay(1000);*/

  

  //Initialize serial and wait for port to open:
  //Serial.begin(9600);


  //  attachInterrupt(digitalPinToInterrupt(_ir_en_1), counter1, FALLING);
  //  attachInterrupt(digitalPinToInterrupt(_ir_en_2), counter2, FALLING);
  //  attachInterrupt(digitalPinToInterrupt(_ir_en_3), counter3, FALLING);
  //  Timer1.initialize(microseconds);
  //  Timer1.attachInterrupt(checkspeed);
  //while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  //}

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    DDRC = 0xFF;
    PORTC = 0xFF;
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    DDRC = 0xFF;
    PORTC = 0x03;
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(5000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
  /*  TCCR3B = (TCCR3B&(~0x07))|0x02;
  TCCR4B = (TCCR4B&(~0x07))|0x02;*/
}

void loop() {

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {

    IPAddress remoteIp = Udp.remoteIP();

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;


    //****************************ADDED for 3 digit velocity
    char *vel = packetBuffer + 1;
    velocity = strtoul(vel, NULL, 0);
    //*********************************************
    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
  //***************************send velocity to move commands  
  if(packetBuffer[0] == 'a'){
        MasonBot().moveRotateCW(velocity);
        packetBuffer[0] = ' ';
  }else if(packetBuffer[0] == 'A'){
        MasonBot().moveRotateCCW(velocity);
         packetBuffer[0] = ' ';
  }else if(packetBuffer[0] == 'f'){
        MasonBot().moveForward(velocity);
         packetBuffer[0] = ' ';         
  }else if(packetBuffer[0] == 's'){
        MasonBot().moveStop();
         packetBuffer[0] = ' ';
  }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

////Counters for encoders
//void counter1() {
//  count1 += 1;
//  Serial.println(count1);
//}
//void counter2() {
//  count2 += 1;
//  Serial.println(count2);
//}
//void counter3() {
//  count3 += 1;
//  Serial.println(count3);
//}

//void checkspeed(){
//  //Add code to counter compared to expected(based on pwm value)
//  // Compare count with wifioutpwm  ie count = 100, and wifipwm = 200 means count should be 75 lower outpwm
//  // one revolution is 8 counts,
//  // if count is < expected
//  //      outpwm = outpwm - 10  use a variable based on the difference of actual to expected.
//  //else if count > expected
//  //      outpwm = outpwm + 10
//
//  count1 = 0;
//  count2 = 0;
//  count3 = 0;
//  Serial.println(count1);
//  Serial.println(count2);
//  Serial.println(count3);
//}


