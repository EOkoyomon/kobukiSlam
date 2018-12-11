from duck_detect import *
from send_instruction import FOLDER
import time

while True:
    fname = get_latest(FOLDER)
    duck_center(FOLDER + fname, display=True)
    time.sleep(0.5)
