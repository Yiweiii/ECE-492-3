/*
  MasonBot movement program
  2017 George Mason University
*/

#include "masonbot.h"
#include <stdio.h>
#include <stdlib.h>
#include <TimerOne.h>

#include <SPI.h>
#include <WiFi101_MasonBot.h>
#include <WiFiUdp.h>

#define PACKET_BUFFER_SIZE 255

// ***
// *** Global Variables
// ***

// UDP Wi-Fi class
WiFiUDP Udp;

int count1 = 0;

// Network-related variables
int status = WL_IDLE_STATUS;
char ssid[] = "QCHJB";
char pass[] = "robotsarecool2!";
char mac[6];
char packetBuffer[PACKET_BUFFER_SIZE];
const char ackMessage[] = "ACK0";
unsigned int localPort = 2390;

// 3-digit velocity stuff
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

// ***
// *** Arduino Setup
// ***
void setup() {
	// Setup MasonBot class
	MasonBot();
	MasonBot().moveStop();

	//WiFi.setPins(41, 45, 47, 43);
	pinMode(A1, INPUT);
	pinMode(43, OUTPUT);

	// DO NOT CHANGE THIS!!!!
	digitalWrite(43, HIGH);
	WiFi.setPins(41, 45, 47);

	// Enable PORTC as outputs
	// PORTC0..4 are LEDs
	DDRC = 0xFF;

	// Try to find Wi-Fi module
	if (WiFi.status() == WL_NO_MODULE) {
		// Turn on all LEDs if the Wi-Fi shield is not present.
		// Do nothing afterward.
		PORTC = L_A|L_B|L_C|L_PWR|L_WIFI;
		while (true);
	}

	// Try to connect to wireless network
	while (status != WL_CONNECTED) {
		PORTC = L_PWR|L_WIFI;
		status = WiFi.begin(ssid, pass);
		delay(5000);
	}

	// Start UDP connection.
	Udp.begin(localPort);

	// Obtain the MAC address of the Wi-Fi module.
	WiFi.macAddress(mac);
}

// ***
// *** Arduino Loop
// ***
void loop() {
	int packetSize = Udp.parsePacket();
	if (packetSize) {
		IPAddress remoteIp = Udp.remoteIP();

		int len = Udp.read(packetBuffer, PACKET_BUFFER_SIZE);
		if (len > 0) 
			packetBuffer[len] = ' ';
			//packetBuffer[len] = 0;

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
		//*********************************************
		
		// send a reply, to the IP address and port that sent us the packet we received
		Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
		Udp.write(ackMessage);
		Udp.endPacket();
	}
	// Move commands.
	// TODO remove redundant packet buffer clear.
	// Move this if-block inside of packetSize if-block.
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
  }else if(packetBuffer[0] == 'c'){
    MasonBot().feedbackRun( Xloc, Yloc, Thetaloc, Xexpected, Yexpected, Thetaexpected);
    packetBuffer[0] = ' ';
  }else if (packetBuffer[0] == 'C'){
        MasonBot().controlRun(&count1, Xloc, Yloc, Thetaloc, Xexpected, Yexpected, Thetaexpected);
        packetBuffer[0] = ' ';
  }else if(packetBuffer[0] == 'm'){
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(mac, 6);
    Udp.endPacket();
    packetBuffer[0] = ' ';
  }
}
