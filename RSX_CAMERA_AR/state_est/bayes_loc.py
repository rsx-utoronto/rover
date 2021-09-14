#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys, select, os
from scipy.special import softmax
import matplotlib.pyplot as plt

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

saved_prob = []
saved_step = []
count = 0

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

    def __init__(self, P0, colourCodes, colourMap, transProbBack, transProbForward, delivery_list):
        self.colour_sub = rospy.Subscriber('camera_rgb', String, self.colour_callback)
        self.line_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        self.cmd_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.delivery_list = delivery_list

        self.probability = P0 ## initial state probability is equal for all states
        self.colourCodes = colourCodes
        self.colourMap = colourMap
        self.transProbBack = transProbBack
        self.transProbForward = transProbForward
        self.numStates = len(P0)
        self.statePrediction = np.full(len(colourMap),P0)
    
        self.CurColour = None ##most recent measured colour

        self.actual = 0 # measured value for line following 
        self.desired = 320 # desired centre value 

        # PID Control Coefficients
        self.kp = 0.004
        self.ki = 0.00001
        self.kd = 0.01
        
        # PID Control Placeholders  
        self.integral = 0
        self.derivative = 0
        self.lasterror = 0
 
    def colour_callback(self, msg):
        #callback function that receives the most recent colour measurement from the camera.
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))
        self.CurColour = np.array([r,g,b])

    def line_callback(self, msg):
        # Get line value 
        self.actual = int(msg.data)
        
    def follow_the_line(self):
        # Initialize twist msg 
        twist = Twist() 
        twist.linear.x = 0.1
        
        # Initialize for proportional control 
        error = self.desired - self.actual
        
        #Integral Control
        self.integral = self.integral + error
        if self.integral >= 0.1: # cap on integral 
            self.integral = 0 

        #Derivative Control
        self.derivative = error - self.lasterror
        if abs(self.derivative) > 50:
            twist.linear.x = 0.06
        
        # Apply control 
        correction = self.kp*error + self.ki*self.integral + self.kd*self.derivative
        twist.angular.z = correction
        self.lasterror = error
        self.cmd_pub.publish(twist)

    def deliver(self):
        # Once office has been delivered to, remove from list
        stop = self.getCurrentLocation()
        self.delivery_list.remove(stop)
        
        # Initialize message
        twist = Twist() 
        twist.linear.x = 0.1 # Move forward for 2 seconds 
        self.cmd_pub.publish(twist)
        rospy.sleep(2)
        twist.linear.x = 0
        twist.angular.z = math.pi/8 #takes 4 seconds

        t = rospy.get_rostime() # Turn 90 degrees
        while (rospy.get_rostime() - t) < rospy.Duration(4) and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
        twist.angular.z = -math.pi/8
        
        t = rospy.get_rostime() # Turn back 90 degrees 
        while (rospy.get_rostime() - t) < rospy.Duration(4) and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
        
        # Set angular velocity back to 0 
        twist.angular.z = 0 #just in case
        self.cmd_pub.publish(twist) #just in case
        rospy.sleep(2.3)

    def waitforcolour(self):
        while(1):
            if self.CurColour is not None:
                break

    def measurement_model(self):
        # Wait until a colour is detected 
        if self.CurColour is None:
            self.waitforcolour()
        
        # Initialize list of probabilities 
        prob=np.zeros(len(self.colourCodes))
        codes = self.colourCodes
        # Find closest colour to the currently detected colour 
        for i, code in enumerate(codes):
            norm_colour = np.linalg.norm(code)
            col = codes[i]/norm_colour 
            norm_curr = np.linalg.norm(bayesian.CurColour)
            curr = bayesian.CurColour/norm_curr
            project = np.dot(col,curr)
            prob[i] = project
        prob_colour = softmax(prob[:4]) # apply softmax to get differentiated colours 
        prob = softmax(prob)
        rospy.loginfo("PROB OF EACH COLOUR: ")
        rospy.loginfo(prob)
        return [prob_colour,prob]

    def statePredict(self,forward):
        rospy.loginfo('Predicting state...')
        state_model = [[self.transProbBack[1], 0, self.transProbForward[0]], [0, 1, 0], [self.transProbBack[0], 0, self.transProbForward[1]]]
        current_probs = self.statePrediction
        uk = forward + 1 #only need to look at the probability of moving forward
        self.statePrediction = [state_model[1][uk]*current_probs[0]+state_model[0][uk]*current_probs[1]+state_model[2][uk]*current_probs[10], state_model[2][uk]*current_probs[0]+state_model[1][uk]*current_probs[1]+ state_model[0][uk]*current_probs[2], \
            state_model[2][uk]*current_probs[1]+ state_model[1][uk]*current_probs[2]+ state_model[0][uk]*current_probs[3], state_model[2][uk]*current_probs[2]+ state_model[1][uk]*current_probs[3]+ state_model[0][uk]*current_probs[4],  \
            state_model[2][uk]*current_probs[3]+state_model[1][uk]*current_probs[4]+ state_model[0][uk]*current_probs[5], state_model[2][uk]*current_probs[4]+ state_model[1][uk]*current_probs[5]+ state_model[0][uk]*current_probs[6],   \
            state_model[2][uk]*current_probs[5]+state_model[1][uk]*current_probs[6]+state_model[0][uk]*current_probs[7], state_model[2][uk]*current_probs[6]+state_model[1][uk]*current_probs[7]+state_model[0][uk]*current_probs[8],   \
            state_model[2][uk]*current_probs[7]+state_model[1][uk]*current_probs[8]+state_model[0][uk]*current_probs[9], state_model[2][uk]*current_probs[8]+state_model[1][uk]*current_probs[9]+state_model[0][uk]*current_probs[10], \
            state_model[2][uk]*current_probs[9]+state_model[1][uk]*current_probs[10]+state_model[0][uk]*current_probs[0] ]


    def stateUpdate(self):
        measurement_model = self.measurement_model()[0]
        color_maps = self.colourMap
        next_probs = self.statePrediction 

        rospy.loginfo('Updating state...')
        probsum = 0
        for j in range(0, 11):
            probsum = probsum + measurement_model[color_maps[j]]*next_probs[j]
        current_probs = \
        [measurement_model[color_maps[0]]*next_probs[0]/probsum, measurement_model[color_maps[1]]*next_probs[1]/probsum, measurement_model[color_maps[2]]*next_probs[2]/probsum, measurement_model[color_maps[3]]*next_probs[3]/probsum, measurement_model[color_maps[4]]*next_probs[4]/probsum, measurement_model[color_maps[5]]*next_probs[5]/probsum, measurement_model[color_maps[6]]*next_probs[6]/probsum, measurement_model[color_maps[7]]*next_probs[7]/probsum, measurement_model[color_maps[8]]*next_probs[8]/probsum, measurement_model[color_maps[9]]*next_probs[9]/probsum, measurement_model[color_maps[10]]*next_probs[10]/probsum]
        self.statePrediction = current_probs
    
    def getCurrentLocation(self):
        max_prob = 0
        index = 0
        maxdex = 0
        probs = self.statePrediction
        rospy.loginfo(probs)
        for prob in probs: 
            if (prob > max_prob):
                max_prob = prob
                maxdex = index
            index = index + 1
        global count
        saved_prob.append(max_prob)
        saved_step.append(count)
        count +=1
        return maxdex





if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # 0: Green, 1: Purple, 2: Orange, 3: Yellow, 4: Line   
    #color_maps = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1] ## original map, indexed 0-10
    color_maps = [3, 2, 0, 3, 2, 1, 0, 1, 2, 0, 3] # test day map 
    color_codes = [[107, 255, 102], #green
                    [206,206,255], #purple
                    [255, 205, 0], #orange
                    [255, 255, 0], #yellow 
                    [223,223,223]] #line
    
    delivery_list = [0, 7, 9] # using offices 0-10 
    delivered = 0
    moved_step = False

    trans_prob_fwd = [0.05,0.95]
    trans_prob_back = [0.05,0.95]
                 
    rospy.init_node('final_project')
    bayesian=BayesLoc([1.0/len(color_maps)], color_codes, color_maps, trans_prob_back,trans_prob_fwd,delivery_list)
    prob = []
    rospy.sleep(0.5)    
    state_count = 0

    located_self = False
    
    prev_state=None
    try:
        
        while (1):
            key = getKey()
            rospy.loginfo(bayesian.CurColour.astype(int))
            codes = bayesian.colourCodes
            prob_colours = bayesian.measurement_model()[1]
            colour = prob_colours.argmax()
            rospy.loginfo("CURRENT COLOUR: ")
            rospy.loginfo(colour)
            if colour == 4:
                moved_step = True
                bayesian.follow_the_line()
            elif (moved_step == True):
                bayesian.statePredict(1) # assume going forwards always
                bayesian.stateUpdate()
                moved_step = False
                current_probs = bayesian.statePrediction
                rospy.loginfo("THIS IS CURRENT LOCATION: ")
                rospy.loginfo(bayesian.getCurrentLocation())
                for prob in current_probs:
                    if (prob > 0.17):
                        located_self = True
                        rospy.loginfo("located self!")
                if (located_self):
                    for place in delivery_list:
                        if (place == bayesian.getCurrentLocation()):
                            bayesian.deliver()
                            delivered = delivered + 1
                twist = Twist()
                twist.angular.z = 0
                twist.linear.x = 0.1
                bayesian.cmd_pub.publish(twist)
                rospy.sleep(2)

            if (key == '\x03' or bayesian.delivery_list == []): 
                fig, ax = plt.subplots()
                ax.plot(saved_step, saved_prob, label='probability')

                ax.set(xlabel='time step ', ylabel='probability',
                title='Max Probaility over time')
                fig.savefig("probs_location.png")
                rospy.loginfo('Finished!')
                rospy.loginfo(prob)
                break
            
            #rospy.loginfo("TODO: complete this main loop by calling functions from BayesLoc, and adding your own high level and low level planning + control logic")
                
    except Exception as e:
        # plotting
        fig, ax = plt.subplots()
        ax.plot(saved_step, saved_prob, label='probability')

        ax.set(xlabel='time step ', ylabel='probability',
            title='Max Probaility over time')
        fig.savefig("probs.png")
        print("comm failed:{}".format(e))

    finally:
        rospy.loginfo(bayesian.probability)
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        cmd_publisher.publish(twist)





