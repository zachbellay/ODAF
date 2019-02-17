import serial
import io
import curses

ser = serial.Serial('/dev/ttyUSB0', 115200)

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0,10,"Hit 'q' to quit")
stdscr.refresh()

key = ''
while key != ord('q'):
	key = stdscr.getch()
	stdscr.addch(20,25,key)
	stdscr.refresh()
	if key == ord('s'):
		stdscr.clear()
		stdscr.addstr(2, 20, "Stop")
		ser.write("0,0,0\n")
	if key == ord('a'):
		stdscr.clear()
		stdscr.addstr(2, 20, "Rotate Left")
		ser.write("0,0,-1\n")
	if key == ord('d'):
		stdscr.clear()
		stdscr.addstr(2, 20, "Rotate Right")
		ser.write("0,0,1\n")
	elif key == curses.KEY_UP: 
		stdscr.clear()
		stdscr.addstr(2, 20, "Up")
		ser.write("0,1,0\n")
	elif key == curses.KEY_DOWN: 
		stdscr.clear()
		stdscr.addstr(2, 20, "Down")
		ser.write("0,-1,0\n")
	elif key == curses.KEY_LEFT:
		stdscr.clear()
		stdscr.addstr(2, 20, "Left")
		ser.write("-1,0,0\n")
	elif key == curses.KEY_RIGHT:
		stdscr.clear()
		stdscr.addstr(2, 20, "Right")
		ser.write("1,0,0\n")
	

curses.endwin()