package main

import (
	"fmt"
	"github.com/nsf/termbox-go"
	"net"
	"strconv"
	"strings"
	"time"
)

func errHandle(err error) {
	if err != nil {
		panic(err)
	}
}

func makeConn(i int) *net.UDPConn {
	// Address and port of server to connect to.
	addrUDP, err := net.ResolveUDPAddr("udp", "192.168.1."+strconv.Itoa(i)+":2390")
	errHandle(err)

	// Do not use 127.x.x.x loopback!
	// lUDP, err := net.ResolveUDPAddr("udp", "192.168.1.3:0")
	conn, err := net.DialUDP("udp", nil, addrUDP)
	errHandle(err)
	conn.SetReadDeadline(time.Now().Add(time.Millisecond * 300))
	return conn
}

var robots map[[6]byte]int = map[[6]byte]int{
	[6]byte{0xf8, 0xf0, 0x05, 0xf1, 0xd6, 0x1c}: 1,
	[6]byte{0xf8, 0xf0, 0x05, 0xf7, 0xff, 0xf9}: 2,
	[6]byte{0xf8, 0xf0, 0x05, 0xf7, 0xff, 0xf1}: 3,
	[6]byte{0xf8, 0xf0, 0x05, 0xf7, 0xff, 0xf2}: 4,
}

func searchForMAC() map[int]string {
	buf := make([]byte, 1)   // Transmitting buffer.
	buf2 := make([]byte, 10) // Receiving buffer.
	macbuf := [6]byte{}
	rList := make(map[int]string)

	// Send ASCII "m"
	buf[0] = 109
	for i := 0; i < 10; i++ {
		conn := makeConn(i)
		//fmt.Println("Sending:", buf)
		conn.Write(buf)
		// Also handle time-out here to prevent throwing garbage MAC address inside table.
		if conn.Read(buf2); strings.Compare(string(buf2[0:4]), "ACK0") == 0 {
			IP := "192.168.1." + strconv.Itoa(i)
			fmt.Print(IP + ": ")
			conn.Read(buf2)
			for i, _ := range buf2[0:6] {
				fmt.Printf("%02X", buf2[5-i])
				macbuf[i] = buf2[5-i]
			}
			fmt.Println(": Robot", robots[macbuf])
			rList[robots[macbuf]] = IP
		}
		// Clear the buffer
		for n := range buf2 {
			buf2[n] = 0
		}
		conn.Close()
	}
	return rList
}

func main() {
	robot := searchForMAC()
	errHandle(termbox.Init())
	defer termbox.Close()
	termbox.SetInputMode(termbox.InputEsc)
	errHandle(termbox.Clear(termbox.ColorDefault, termbox.ColorDefault))
	errHandle(termbox.Flush())

	robotAddr, err := net.ResolveUDPAddr("udp", "192.168.1.6:2390")
	errHandle(err)

	conn, err := net.DialUDP("udp", nil, robotAddr)
	errHandle(err)

myLoop:
	for {
		switch event := termbox.PollEvent(); event.Type {
		case termbox.EventKey:
			switch event.Key {
			case termbox.KeyEsc:
				break myLoop
			case termbox.KeyArrowLeft:
				fmt.Fprintf(conn, "A128")
			case termbox.KeyArrowRight:
				fmt.Fprintf(conn, "a128")
			case termbox.KeyArrowUp:
				fmt.Fprintf(conn, "f255")
			case termbox.KeyArrowDown:
				fmt.Fprintf(conn, "s")
			default:
				if event.Ch != 0 {
					num := int(event.Ch - 48)
					if num >= 1 && num <= 4 {
						conn.Close()
						robotAddr, err := net.ResolveUDPAddr("udp", robot[num]+":2390")
						errHandle(err)

						conn, err = net.DialUDP("udp", nil, robotAddr)
						errHandle(err)
					}
				}
			}
		}
	}
	conn.Close()
}
