from socket import *
import sys

HOST = ''            
PORT = 21567         
BUFSIZE = 1024             
ADDR = (HOST,PORT)


udpCliSock = socket(AF_INET, SOCK_DGRAM)
udpCliSock.bind(ADDR)       


while True:
	print "Ready to Receive from Server..."
	data, addr = udpCliSock.recvfrom(BUFSIZE)
	
	if not data:
		udpCliSock.sendto("NAK", addr)
		print "empty socket"
		
	elif data:
		print "Received data...", data
		
		#MESSAGE PROCESSING DONE BY ARDUINO
		#...
		#GET MESSAGE BACK: MESSAGE = "ACK0"
		MESSAGE = "ACK1"
		udpCliSock.sendto(MESSAGE, addr)
		
	
	print "Received from Server address: ", addr
         
udpCliSock.close()