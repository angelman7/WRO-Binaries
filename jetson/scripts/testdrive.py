import sys, tty, termios

try:
    filedescriptors = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    while True:
        key = sys.stdin.read(1)[0]
        print(key)

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, filedescriptors)
    key = sys.stdin.read(1)[0]