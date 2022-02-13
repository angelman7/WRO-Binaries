from datetime import datetime
import logging

logging.basicConfig(filename="./jetson.log", level=logging.DEBUG, encoding='utf-8')

"""
The program that will run for the preparation of the 1st system, in the conditions
of the 1st part of the competition. With the current design, the file exists
for possible future development and utilization
"""


while True:
    try:
        while True:
            pass
    except KeyboardInterrupt as exc:
        logging.info("[" + str(datetime.now())[0:18] + "]" + "Application closed due to: KeyboardInterrupt")
        break
    except Exception as exc:
        logging.error("[" + str(datetime.now())[0:18] + "]" + str(exc))