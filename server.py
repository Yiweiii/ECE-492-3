from socket import *
import sys

HOST = 'localhost'
PORT = 21567
BUFSIZE = 1024
FLAG = 0

ADDR = (HOST, PORT)

udpSerSock = socket(AF_INET, SOCK_DGRAM)


while True:
    MESSAGE = raw_input('>')
    print "send message: ", MESSAGE

    udpSerSock.sendto(MESSAGE, ADDR
        data, ADDR = udpSerSock.recvfrom(BUFSIZE)
    except:
        data = "FAIL"

    if data == "ACK0":
        print "ACK received"
        print "batterylow"
    elif data == "ACK1":
        print "batteryhigh"
    elif data == "FAIL":
        print "Robot not connected"
    else:
        print "Message not recieved by robot, re-send"

udpSerSock.close()

