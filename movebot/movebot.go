// Robot move program
// Manually controls robots
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

func searchForMAC(status int) map[int]string {
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
			if status == 0 {
				fmt.Print(IP + ": ")
			}
			conn.Read(buf2)
			for i, _ := range buf2[0:6] {
				if status == 0 {
					fmt.Printf("%02X", buf2[5-i])
				}
				macbuf[i] = buf2[5-i]
			}
			if status == 0 {
				fmt.Println(": Robot", robots[macbuf])
			}
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

func drawText(x, y int, s string, fg, bg termbox.Attribute) {
	for i, v := range s {
		termbox.SetCell(x+i, y, v, fg, bg)
	}
	errHandle(termbox.Flush())
}

func drawList(rList map[int]string, sel, status, nbots int) {
	voff := 2
	hbg := termbox.ColorBlue
	sbg := termbox.ColorRed
	cbg := termbox.ColorDefault
	for k := 0; k < nbots; k++ {
		if k == sel {
			if status == 1 {
				cbg = sbg
			} else {
				cbg = hbg
			}
		} else {
			cbg = termbox.ColorDefault
		}
		if r, exist := rList[k]; exist == true {
			drawText(10, k+voff, r, termbox.ColorWhite, cbg)
			drawText(0, k+voff, "Robot "+strconv.Itoa(k)+":", termbox.ColorWhite, cbg)
		} else {
			drawText(10, k+voff, "-----------", termbox.ColorWhite, cbg)
			drawText(0, k+voff, "        ", termbox.ColorWhite, cbg)
		}
	}
}

func main() {
	robot := searchForMAC(0)
	errHandle(termbox.Init())
	nbots := 10
	width, height := termbox.Size()
	defer termbox.Close()
	termbox.SetInputMode(termbox.InputEsc)
	errHandle(termbox.Clear(termbox.ColorDefault, termbox.ColorDefault))
	errHandle(termbox.Flush())

	robotAddr, err := net.ResolveUDPAddr("udp", "192.168.1.6:2390")
	errHandle(err)

	conn, err := net.DialUDP("udp", nil, robotAddr)
	errHandle(err)
	status := 0
	sel := 0
	drawList(robot, sel, status, nbots)
	drawText(0, 0, "Robot Controller. Press 'Enter' to select robot. Arrow keys control robot.", termbox.ColorWhite, termbox.ColorDefault)
	drawText(0, 1, "Press 's' is stop all robots. Press 'r' to search network.", termbox.ColorWhite, termbox.ColorDefault)

myLoop:
	for {
		switch event := termbox.PollEvent(); event.Type {
		case termbox.EventKey:
			switch event.Key {
			case termbox.KeyEsc:
				if status == 0 {
					break myLoop
				} else {
					status--
				}
			case termbox.KeyArrowLeft:
				if status == 1 {
					fmt.Fprintf(conn, "A128")
				}
			case termbox.KeyArrowRight:
				if status == 1 {
					fmt.Fprintf(conn, "a128")
				}
			case termbox.KeyArrowUp:
				if status == 1 {
					fmt.Fprintf(conn, "f255")
				} else {
					if sel > 0 {
						sel--
					}
				}
			case termbox.KeyArrowDown:
				if status == 1 {
					fmt.Fprintf(conn, "s")
				} else {
					if sel < nbots-1 {
						sel++
					}
				}
			case termbox.KeyEnter:
				if status == 0 {
					if rs, exist := robot[sel]; exist == true {
						status++
						conn.Close()
						robotAddr, err := net.ResolveUDPAddr("udp", rs+":2390")
						errHandle(err)

						conn, err = net.DialUDP("udp", nil, robotAddr)
						errHandle(err)
					}
				}
			default:
				if event.Ch != 0 {
					if event.Ch == 's' {
						for _, v := range robot {
							conn.Close()
							robotAddr, err := net.ResolveUDPAddr("udp", v+":2390")
							errHandle(err)
							conn, err = net.DialUDP("udp", nil, robotAddr)
							errHandle(err)
							fmt.Fprintf(conn, "s")
						}
						status = 0
					} else if event.Ch == 'r' {
						status = 0
						drawText((width/2)-5, height/2, "Searching...", termbox.ColorWhite, termbox.ColorBlue)
						robot = searchForMAC(1)
						drawText((width/2)-5, height/2, "            ", termbox.ColorWhite, termbox.ColorDefault)
					}
				}
			}
			drawList(robot, sel, status, nbots)
		}
	}
	conn.Close()
}
