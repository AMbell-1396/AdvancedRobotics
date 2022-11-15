import rospy
import os
import sys
import time

from std_msgs.msg import String #String message 
from std_msgs.msg import Int8

from curtsies import Input

FILEPATH = os.path.dirname(os.path.abspath(__file__)) + '/latest_key.txt'
PUBLISHTOPIC = 'elfin/reactive_controller/key'

def keys():
    rospy.init_node('keyparser')
    pub = rospy.Publisher(PUBLISHTOPIC, String,queue_size=10) # "key" is the publisher name
    key = ''
    bufferString = ""

    #https://github.com/thomasballinger/curtsies/blob/master/examples/curses_keys.py
    with Input(keynames='curses') as input_generator:
        print("use 'ESC' to exit input (force-quit breaks this terminal instance)")
        for key in input_generator:
            if (key == '\x1b'):
                # handle ESC
                break

            elif (key == '\n'):
                #handle ENTER
                pub.publish(bufferString)
                print(repr(bufferString) + " published to " + PUBLISHTOPIC)
                bufferString = ""

            elif (key == '\x7f'):
                #handle backspace
                bufferString = bufferString[:len(bufferString)-1]

            else:
                bufferString += key

            # save key to filepath (legacy functionality)
            writeLastKey(key)
            sys.stdout.write('\033[K' + repr(bufferString) + '\r')
            
            #reset key just in case
            key = ''

def writeLastKey(string):
    # write last pressed key to latest_key.txt (unused)
    file = open(FILEPATH, 'w')
    file.write(string)

if __name__=='__main__':
    keys()
