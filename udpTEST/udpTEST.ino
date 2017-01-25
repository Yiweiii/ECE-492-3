/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using a WiFi shield.
 When a packet is received an Acknowledge packet is sent to the client on port remotePort

 Circuit:
 * WiFi shield attached

 created 30 December 2012
 by dlf (Metodo2 srl)

 */

#include <stdio.h>
#include <stdlib.h>

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

int status = WL_IDLE_STATUS;
char ssid[] = "XSABIN"; //  your network SSID (name) //ip address 10.0.0.39
char pass[] = "A8970aacef768421bd8797";    // your network password (use for WPA, or use as key for WEP)
//char ssid[] = "Verizon-SM-G930V-6155"; //  your network SSID (name) // IP address 192.168.43.95(may change)
//char pass[] = "yhay051/";  

int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
int outpwm1 = 0;
char pwm1[2]; //buffer for pwm input
int outpwm2 = 0;
char pwm2[2]; //buffer for pwm input
int outpwm3 = 0;
char pwm3[2]; //buffer for pwm input
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;

void setup() {
  WiFi.setPins(8,7,4);
  pinMode(3, OUTPUT);      // set the LED pin mode
  pinMode(5, OUTPUT);      // powers motor
  pinMode(6, OUTPUT);      // powers motor
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void loop() {

  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;

    
    char *pwm1 = packetBuffer + 1;
    char *pwm2 = packetBuffer + 6;
    char *pwm3 = packetBuffer + 11;
    outpwm1 = strtoul(pwm1, NULL, 0);
    outpwm2 = strtoul(pwm2, NULL, 0);
    outpwm3 = strtoul(pwm3, NULL, 0);
  
    
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    Serial.println(outpwm1);
    Serial.println(outpwm2);
    Serial.println(outpwm3);
    
    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
  if(packetBuffer[0] == 'A'){
//      analogWrite(5, LOW);
//      analogWrite(6, outpwm1);
      analogWrite(3, outpwm1);               // GET /H turns the LED on
  }else if(packetBuffer[0] == 'a'){
//      analogWrite(6, LOW);
//      analogWrite(5, outpwm1);
      analogWrite(3, outpwm1);               // GET /H turns the LED on
  }else{
//      analogWrite(6, LOW);
//      analogWrite(5, LOW);
      analogWrite(3, LOW);               // GET /H turns the LED on
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




