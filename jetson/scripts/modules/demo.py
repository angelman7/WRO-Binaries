import tty, sys, termios

filedescriptors = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)
x = 0
while 1:
  x=sys.stdin.read(1)[0]
  print("You pressed", x)
  if x == "r":
    print("If condition is met")
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)