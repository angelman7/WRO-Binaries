from modules.camera import Camera

from datetime import datetime
from pathlib import Path
import numpy as np
import argparse
import logging
import cv2
import csv

logging.basicConfig(filename="./jetson.log", level=logging.DEBUG, encoding='utf-8')

"""
The program that will run for the preparation of the 1st system, in the
conditions of the 2nd part of the competition. It would be useful for 
an application to display the output given as a parameter by the
user (eg default display, romote display etc) what the camera sees, and
the user can (also remotely):
 - Create a snapshot of the camera
 - Select an area in the snapshot with the mouse (cropping)
 - Repeat the above two steps as many times as desired
 - After selecting the user, display in a new window the areas selected in the previous steps, with a suitable tool for selecting color components in HSV color space (track bars), for the manual configuration of lower and upper HSV levels
 - Save the lower & upper components to a file that will be read by run2.py for the 2nd mission (with an appropriate argument in the command line)
"""

win_name = "HSV Configuration"

parser = argparse.ArgumentParser(description="Pillars Calibration")
parser.add_argument('color', type=str, default='red', help="The Pillar's name (red, blue etc)")
parser.add_argument('filename', type=str, default='pillars_values.csv', help="The output csv file name")

args = parser.parse_args()

logging.basicConfig(filename="./jetson.log", level=logging.DEBUG, encoding='utf-8')

def create_csv_file(data_file):
    with open(data_file, 'w') as f:
        writer = csv.writer(f)
        header = ("color", "Lower", "Upper")
        writer.writerow(header)

def add_csv_file(data_file, data):
    with open(data_file, 'a') as f:
        writer = csv.writer(f)
        writer.writerow(data)

try:
    while True:
        pass
except Exception as e:
    logging.error(str(datetime.now())[0:18] + str(Exception))
