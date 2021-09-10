
#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys, select, os

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class BayesLoc:

    def __init__(self, P0, colourCodes, colourMap, transProbBack, transProbForward):
        self.colour_sub = rospy.Subscriber('camera_rgb', String, self.colour_callback)
        self.line_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        self.cmd_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.probability = P0 ## initial state probability is equal for all states
        self.colourCodes = colourCodes
        self.colourMap = colourMap
        self.transProbBack = transProbBack
        self.transProbForward = transProbForward
        self.numStates = len(P0)
        self.statePrediction = np.zeros(np.shape(P0))

        self.CurColour = None ##most recent measured colour

 
    def colour_callback(self, msg):
        '''
        callback function that receives the most recent colour measurement from the camera.
        '''
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))
        self.CurColour = np.array([r,g,b])

    def line_callback(self, msg):
        '''
        TODO: Complete this with your line callback function from lab 3.
        '''
        return
    

    def waitforcolour(self):
        while(1):
            if self.CurColour is not None:
                break

    def measurement_model(self):
        if self.CurColour is None:
            self.waitforcolour()
        prob=np.zeros(len(colourCodes))
        '''
        Measurement model p(z_k | x_k = colour) - given the pixel intensity, what's the probability that  
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.CurColour) 
            and the reference RGB values of each color (self.ColourCodes).
        '''
        return prob

    def statePredict(self,forward):
        rospy.loginfo('predicting state')
        '''
        TODO: Complete the state prediction function
        '''

    def stateUpdate(self):
        rospy.loginfo('updating state')
        '''
        TODO: Complete the state update function
        '''      




if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # 0: Green, 1: Purple, 2: Orange, 3: Yellow, 4: Line   
    color_maps = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1] ## current map starting at cell#2 and ending at cell#12
    color_codes = [[72, 255, 72], #green
                    [255, 144, 0], #orange
                    [145,145,255], #purple
                    [255, 255, 0], #yellow 
                    [133,133,133]] #line

    trans_prob_fwd = [0.1,0.9]
    trans_prob_back = [0.2,0.8]
                 
    rospy.init_node('final_project')
    bayesian=BayesLoc([1.0/len(color_maps)]*len(color_maps), color_codes, color_maps, trans_prob_back,trans_prob_fwd)
    prob = []
    rospy.sleep(0.5)    
    state_count = 0
    
    prev_state=None
    try:
        
        while (1):
            key = getKey()
            if (key == '\x03'): 
                rospy.loginfo('Finished!')
                rospy.loginfo(prob)
                break
            
            rospy.loginfo("TODO: complete this main loop by calling functions from BayesLoc, and adding your own high level and low level planning + control logic")
                
    except Exception as e:
        print("comm failed:{}".format(e))

    finally:
        rospy.loginfo(bayesian.probability)
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        cmd_publisher.publish(twist)


