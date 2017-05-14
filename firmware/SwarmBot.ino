/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using a WiFi shield.
 When a packet is received an Acknowledge packet is sent to the client on port remotePort

 Circuit:
 * WiFi shield attached

 created 30 December 2012
 by dlf (Metodo2 srl)

 */

#include <masonbot.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <TimerOne.h>

#include <SPI.h>
#include <WiFi101_MasonBot.h>
#include <WiFiUdp.h>

const int TEST = 88;   //Battery test

int battery = 6;
float batterylevel = 0;
char mac[6];

int count1 =0;
int count2 =0;
int count3 =0;

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
char Xlocch[2]; //buffer for pwm input
char Ylocch[2]; //buffer for pwm input
char Thetalocch[2]; //buffer for pwm input
char Xexpectedch[2]; //buffer for pwm input
char Yexpectedch[2]; //buffer for pwm input
char Thetaexpectedch[2]; //buffer for pwm input

float Xloc; //buffer for pwm input
float Yloc; //buffer for pwm input
float Thetaloc; //buffer for pwm input
float Xexpected; //buffer for pwm input
float Yexpected; //buffer for pwm input
float Thetaexpected; //buffer for pwm input

int velocity = 0;
int theta = 0;
//************************************

WiFiUDP Udp;

void setup() {
    MasonBot();
    MasonBot().moveStop();
    
WiFi.setPins(41,45,47,43); //47 CS, 45 en, 43 irq, 41 rst
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  pinMode(_ir_read_1, INPUT_PULLUP);
  pinMode(_ir_read_2, INPUT_PULLUP);
  pinMode(_ir_read_3, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(_ir_read_1 ), counter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(_ir_read_2 ), counter2, FALLING);
  attachInterrupt(digitalPinToInterrupt(_ir_read_3 ), counter3, FALLING);

  pinMode(_ir_en_1, OUTPUT);
  pinMode(_ir_en_2, OUTPUT);
  pinMode(_ir_en_3, OUTPUT);
  digitalWrite(_ir_en_1, HIGH);
  digitalWrite(_ir_en_2, HIGH);
  digitalWrite(_ir_en_3, HIGH);
//  Timer1.initialize(microseconds);
//  Timer1.attachInterrupt(checkspeed);
  while (!Serial) {
   ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_MODULE) {
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
    delay(5000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

  // Obtain the MAC address of the Wi-Fi module.
  WiFi.macAddress(mac);
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


    //****************************ADDED for 3 digit velocity
    char *vel = packetBuffer + 1;
    velocity = strtoul(vel, NULL, 0);

    // Controlled move,  where capital is current and lower is wanted
    //C X.XX,Y.YY,ZZZ,x.xx,y.yy,zzz
    
    char *Xlocch = packetBuffer + 2;
    char *Ylocch = packetBuffer + 7;
    char *Thetalocch = packetBuffer + 12;
    char *Xexpectedch = packetBuffer + 16;
    char *Yexpectedch = packetBuffer + 21;
    char *Thetaexpectedch = packetBuffer + 26;

    Xloc = atof(Xlocch);
    Yloc = atof(Ylocch);
    Thetaloc = atof(Thetalocch);
    Xexpected = atof(Xexpectedch);
    Yexpected = atof(Yexpectedch);
    Thetaexpected = atof(Thetaexpectedch); 
    
    theta = vel;

    
    //*********************************************
    Serial.println("Contents:");
    Serial.println(packetBuffer);

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
  }else if(packetBuffer[0] == 'r'){
        MasonBot().runForward(&count1);
        packetBuffer[0] = ' ';
  }else if(packetBuffer[0] == 'D'){
        MasonBot().feedbackRun( Xloc, Yloc, Thetaloc, Xexpected, Yexpected, Thetaexpected);
        packetBuffer[0] = ' ';
  }else if(packetBuffer[0] == 'c'){
        MasonBot().fbRunarc( Xloc, Yloc, Thetaloc, Xexpected, Yexpected, Thetaexpected);
        packetBuffer[0] = ' ';
  }else if (packetBuffer[0] == 'C'){
        MasonBot().controlRun(&count1, Xloc, Yloc, Thetaloc, Xexpected, Yexpected, Thetaexpected);
        packetBuffer[0] = ' ';
  }else if (packetBuffer[0] == 'T'){
        MasonBot().angle_control(int theta){
  }else if(packetBuffer[0] == 'm'){
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(mac, 6);
    Udp.endPacket();
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
void counter1() {  
  count1 += 1;
  //Serial.println(count1);
}

void counter2() {
  count2 += 1;
//  Serial.println("C2 ");
}

void counter3() {
  count3 += 1;
//  Serial.println("C3 ");
}

//void checkspeed(){
//  Add code to counter compared to expected(based on pwm value)
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

