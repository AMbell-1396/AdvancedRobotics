import rospy
import os

from std_msgs.msg import String #String message 
from std_msgs.msg import Int8

from curtsies import Input

FILEPATH = os.path.dirname(os.path.abspath(__file__)) + '/latest_key.txt'

def keys():
    pub = rospy.Publisher('elfin/reactive_controller/key',String,queue_size=10) # "key" is the publisher name
    rospy.init_node('keypress')
    k = ''
    #https://github.com/thomasballinger/curtsies/blob/master/examples/curses_keys.py
    with Input(keynames='curses') as input_generator:
        for k in input_generator:
            if (k == '\x1b'):
                break
            print(repr(k))
            pub.publish(k)
            writeBurger(k)
            k = ''

def writeBurger(k):
    f = open(FILEPATH, 'w')
    f.write(k)

if __name__=='__main__':
    keys()
