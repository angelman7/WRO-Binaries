try:
  import usocket as socket
except:
  import socket
#from car import Car
from machine import Pin
import network

import esp
esp.osdebug(None)

# eliminates useless data (gc = garbage collector)
import gc
gc.collect()

ssid = 'Angelman iPhone 12'
password = 'esp32-server'

steering = 0
throttle = 0
station = network.WLAN(network.STA_IF)

station.active(True)
station.connect(ssid, password)

while station.isconnected() == False:
  pass

print('Connection successful')
print(station.ifconfig())

led = Pin(2, Pin.OUT)

def web_page():
  if led.value() == 1:
    gpio_state="ON"
  else:
    gpio_state="OFF"
  
  html ="""<html><head> <title>ESP Web Server</title> <meta name="viewport" content="width=device-width, initial-scale=1">
      <link rel="icon" href="data:,"> <style>html{font-family: Helvetica; display:inline-block; margin: 0px auto; text-align: center;}
      h1{color: #0F3376; padding: 2vh;}p{font-size: 1.5rem;}.button{display: inline-block; background-color: #e7bd3b; border: none; 
      border-radius: 4px; color: white; padding: 16px 40px; text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
      .button2{background-color: #4286f4;}</style></head>

      <body> 
      <h1>ESP Web Server</h1> 

          <p>GPIO state: <strong>A</strong></p>
          <p><a href="/?throttle=up"><button class="button">UP</button></a></p>
          <p><a href="/?steering=left"><button class="button">LEFT</button></a>
          <a href="/?steering=right"><button class="button">RIGHT</button></a></p>
          <p><a href="/?throttle=down"><button class="button">DOWN</button></a></p>
          </p>
      </body></html>"""
  
  return html

def check_request(request):
    global throttle, steering
    
    th_up = request.find("/?throttle=up")
    th_down = request.find("/?throttle=down")
    st_left = request.find("/?steering=left")
    st_right = request.find("/?steering=right")
    if th_up == 6:
        throttle = min(100, throttle+5)
    if th_down == 6:
        throttle = max(100, throttle-5)
    if st_left == 6:
        throttle = min(100, steering-5)
    if st_right == 6:
        throttle = max(100, throttle+5)
    print("throttle: ", throttle)
    print("steering: ", steering)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 80))
s.listen(5)
led_state = False

while True:
  conn, addr = s.accept()
  print('Got a connection from ' + str(addr))
  request = conn.recv(1024)
  request = str(request)
  print('Content = ' + request)
  
  check_request(request)

  response = web_page()
  conn.send('HTTP/1.1 200 OK\n')
  conn.send('Content-Type: text/html\n')
  conn.send('Connection: close\n\n')
  conn.sendall(response)
  conn.close()