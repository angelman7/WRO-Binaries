from datetime import datetime
import logging

'''
The program that will run on the Jetson Nano 2GB during the 1st phase of the competition.
Exists for possible future development and utilization.
'''

logging.basicConfig(filename="./jetson.log", level=logging.DEBUG, encoding='utf-8')

try:
    while True:
        pass
except Exception as e:
    logging.error(str(datetime.now())[0:18] + str(Exception))