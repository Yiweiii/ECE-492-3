/*
    Swarm robot basic controls
    Used UDPSEND/RECIEVE for setup
    Controls three wheels of the holonomic robot using 3 seperate pwm's
 */

#include <stdio.h>
#include <stdlib.h>

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <TimerOne.h>


const int HALT1 = 22;
const int HALT2 = 23;
const int HALT3 = 24;
const int DIR1 = 25;
const int DIR2 = 26;
const int DIR3 = 27;

const int MOTOR1 = 2;
const int MOTOR2 = 3;
const int MOTOR3 = 5;

const int ENCODERE1 = 30;
const int ENCODERE2 = 31;
const int ENCODERE3 = 32;
const int ENCODERCOUNT1 = 19;
const int ENCODERCOUNT2 = 20;
const int ENCODERCOUNT3 = 21;

const int TEST = 40;   //Battery test




int battery = 6;
float batterylevel = 0;

int count1 =0;
int count2 =0;
int count3 =0;
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
char  ReplyBuffer[] = "ACK0";       // a string to send back
int microseconds = 50000;

WiFiUDP Udp;

void setup() {
  WiFi.setPins(8,7,4);

  
  pinMode(MOTOR1, OUTPUT);      // powers motor1
  pinMode(MOTOR2, OUTPUT);      // powers motor2
  pinMode(MOTOR3, OUTPUT);      // powers motor3
  pinMode(HALT1, OUTPUT);     // motor halt1
  pinMode(HALT2, OUTPUT);     // motor halt2
  pinMode(HALT3, OUTPUT);     // motor halt3
  pinMode(DIR1, OUTPUT);     // motor dir1
  pinMode(DIR2, OUTPUT);     // motor dir2
  pinMode(DIR3, OUTPUT);     // motor dir3

  pinMode(ENCODERE1, OUTPUT);     // encoder enable1
  pinMode(ENCODERE2, OUTPUT);     // encoder enable2
  pinMode(ENCODERE3, OUTPUT);     // encoder enable3
  pinMode(ENCODERCOUNT1, INPUT_PULLUP);  //interrupts for encoder1
  pinMode(ENCODERCOUNT2, INPUT_PULLUP);  //interrupts for encoder2
  pinMode(ENCODERCOUNT3, INPUT_PULLUP);  //interrupts for encoder3

   pinMode(TEST, OUTPUT);     // Powertest enable
   pinMode(A0, INPUT);
  
  
  
  attachInterrupt(digitalPinToInterrupt(19), counter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(20), counter2, FALLING);
  attachInterrupt(digitalPinToInterrupt(21), counter3, FALLING);
  Timer1.initialize(microseconds);
  Timer1.attachInterrupt(checkspeed);
  
  
  
  
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
  
    if(packetBuffer[0] == 'A'){ //Wheel one forward
        digitalWrite(DIR1, HIGH);
        digitalWrite(HALT1, HIGH);
        digitalWrite(ENCODERE1, HIGH);
        analogWrite(MOTOR1, outpwm1);
        
    }else if(packetBuffer[0] == 'a'){ //Wheel one reverse
        digitalWrite(DIR1, LOW);
        digitalWrite(HALT1, HIGH);
        digitalWrite(ENCODERE1, HIGH);
        analogWrite(MOTOR1, outpwm1);
    }else{
        digitalWrite(HALT1, LOW); //stop wheel one
        digitalWrite(ENCODERE1, LOW);
        analogWrite(MOTOR1, LOW);
    }
    if(packetBuffer[5] == 'B'){ //Wheel TWO forward
        digitalWrite(DIR2, HIGH);
        digitalWrite(HALT2, HIGH);
        digitalWrite(ENCODERE2, HIGH);
        analogWrite(MOTOR2, outpwm2);
    }else if(packetBuffer[5] == 'b'){ //Wheel TWO reverse
        digitalWrite(DIR2, LOW);
        digitalWrite(HALT2, HIGH);
        digitalWrite(ENCODERE2, HIGH);
        analogWrite(MOTOR2, outpwm2);
    }else{                               //stop wheel TWO
        digitalWrite(HALT2, LOW); 
        digitalWrite(ENCODERE2, LOW);
        analogWrite(MOTOR2, LOW);
    }
        if(packetBuffer[10] == 'C'){ //Wheel three forward
        digitalWrite(DIR3, HIGH);
        digitalWrite(HALT3, HIGH);
        digitalWrite(ENCODERE3, HIGH);
        analogWrite(MOTOR3, outpwm3);
    }else if(packetBuffer[10] == 'c'){ //Wheel three reverse
        digitalWrite(DIR3, LOW);
        digitalWrite(HALT3, HIGH);
        digitalWrite(ENCODERE3, HIGH);
        analogWrite(MOTOR3, outpwm3);
    }else{
        digitalWrite(HALT3, LOW); //stop wheel three
        digitalWrite(ENCODERE3, LOW);
        analogWrite(MOTOR3, LOW);
    }
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

//Counters for encoders
void counter1() {  
  count1 += 1;
  Serial.println(count1);
}
void counter2() {
  count2 += 1;
  Serial.println(count2);
}
void counter3() {
  count3 += 1;
  Serial.println(count3);
}

void checkspeed(){
  //Add code to counter compared to expected(based on pwm value)
  
  count1 = 0;
  count2 = 0;
  count3 = 0;
  Serial.println(count1);
  Serial.println(count2);
  Serial.println(count3);

  digitalWrite(TEST, HIGH);
  battery = analogRead(A0);
  batterylevel = battery * (5/1024.0);
  Serial.println(batterylevel);
  if(batterylevel < 2.5)
    ReplyBuffer[-1] = '1';
  else
    ReplyBuffer[-1] = '0';
  
  
}






