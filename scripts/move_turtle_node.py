#!/usr/bin/env python
import rospy
import math
import threading
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from ros_first_steps.srv import *
from std_srvs.srv import Empty
from turtlesim.srv import *
from ros_first_steps.cfg import paramConfig
from dynamic_reconfigure.server import Server

"""****Declaration of global variables******"""
x = 0
y = 0
angle = 0
play = True
speed_param = 5

"""Let's generate three locks, one for the access to play/pause variable 
,other to access to the position variables and other one to acces to the speed"""
lock_play = threading.Lock() 
lock_pose = threading.Lock()
lock_speed = threading.Lock()
"""********Declaration of functions/handlers********"""

def speedCallback(config,level):
    """Callback for the dynamic reconfigure"""
    global lock_speed
    global speed_param
    rospy.loginfo("""Reconfigure Request: {turtle_speed}""".format(**config))
    with lock_speed:
        speed_param = config.turtle_speed
    return config
    



def srvCallback(req):
    """Handler of the play/pause service"""
    global play
    global lock_play
    rospy.loginfo("Service for Play/Pause Called")
    with lock_play:
        if(req.request == 'play'):
            play = True
            rospy.loginfo("Resuming movement")
            return 'OK. Resuming movement'
        elif(req.request == 'pause'):
            play = False
            rospy.loginfo("Pausing movement")
            return 'OK. Pausing movement'
            
        else:
            rospy.logwarn("Argument not well passed")
            return "Error. You should use 'play' or 'pause' as an argument. Try again."
            

    
def poseCallback (pose_msg):
    """Callback function for the suscript topic"""
    global x
    global y,angle
    global lock_pose

    with lock_pose:
        x = pose_msg.x
        y = pose_msg.y
        angle = pose_msg.theta

def move_straight(distance):
    """Move the turtle a distance, which was passed by argument"""
    global vel_pub
    global x,y
    global play
    global lock_pose
    global lock_play
    global lock_speed
    global speed_param
    lineal_k = 0.3
    vel_msg = Twist()
    difference = 0.0

    with lock_pose:
        x0=x
        y0=y

    rate = rospy.Rate(62)

      
    while True:
        
        with lock_speed:
            speed_escalator = speed_param
        with lock_pose:
            distance_moved = math.sqrt(((x-x0)** 2) + (y-y0)**2 )
        with lock_play:
            vel_msg.linear.x = play*lineal_k * speed_escalator * (abs(distance - distance_moved))

        vel_pub.publish(vel_msg)
        last_difference = difference
        difference = abs(distance - distance_moved)
        if  ((difference < 0.01 or difference > last_difference )and last_difference != 0 ):
            break
        rate.sleep()

    vel_msg.linear.x = 0
    vel_pub.publish(vel_msg)


def rotate(relative_angle, clockwise): #In radian
    """Rotate one desired angle clockwise or counterclockwise """
    global play
    global lock_pose
    global lock_pose
    global lock_speed
    global angle
    global speed_param
    angular_k = 0.3
    difference = 0.0
    last_difference = 0.0
    vel_msg = Twist()


    #angle depends of pose.theta, which is positive or negative depending
    #if the movement before was clockwise or not
    #I am going to convert always to positive 0-2pi
    with lock_pose:        
        theta0 = angle
    if theta0<0:
        theta0 = math.pi*2 + theta0

    #If the angle is bigger than 360 degrees, module is apply
    if relative_angle >= math.pi*2:
        relative_angle = relative_angle % (math.pi*2)

    rate = rospy.Rate(62)

    while True:

        with lock_speed:
            angular_speed_escalator = speed_param
        
        #angle depends of pose.theta, wich variable is positive or negative depending
        #if the movement before was clockwise or not
        #I am going to convert always to positive 0-2pi
        with lock_pose:
            if angle < 0:
                angle_positive = math.pi*2 + angle   #angle is negative in this case 
            else:
                angle_positive = angle
        #check the angle moved
        if(clockwise):
            angle_moved = theta0 - angle_positive
        else:
            angle_moved = angle_positive - theta0
        #if the angle_moved is negative means that it cross the 0 degree/radian
        if angle_moved < 0:# here angle_moved is negative
            angle_moved = math.pi*2 + angle_moved 
            #Now the value is absolute

        with lock_play:
            vel_msg.angular.z = abs(play*angular_k*angular_speed_escalator*(relative_angle - angle_moved))

        if(clockwise):#If it must rotate cloclwise change the sign
            vel_msg.angular.z = -vel_msg.angular.z 
        
        vel_pub.publish(vel_msg)
          
        last_difference = difference
        difference = abs(relative_angle-angle_moved)
        if(( difference < 0.01 or last_difference < difference) and last_difference !=0):
            break
        rate.sleep()
    #Stop the turtle
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)

def go_to(x_end, y_end):
    """ Go to an specific coordinate. First rotate and 
        then go forward to the final destination"""
    global x
    global y,angle
    global lock_pose

    with lock_pose:
        distance = math.sqrt((x_end-x)**2 + (y_end - y)**2)
        desired_angle = math.atan2(y_end - y,x_end - x) #atan2 returns betwen a range[-pi,+pi]

    if(desired_angle < 0):
        desired_angle = desired_angle + math.pi*2
    compass(desired_angle)
    move_straight( distance)


def compass(desired_angle):
    """Rotate the turtle until it reach an aboslute angle"""
    global lock_pose
    #convert angle to positive 0-2pi
    with lock_pose:
        if angle < 0:
            angle_positive = math.pi*2 + angle   #angle is negative in this case 
        else:
            angle_positive = angle

    delta_angle = desired_angle - angle_positive
    ##calculate which is the shortest way to rotate
    if delta_angle < 0:
        #if delta_angle is negative let's check if its biger than 180degrees
        if((abs(delta_angle) / math.pi) > 1.0):
            clockwise = False
            delta_angle = math.pi*2 + delta_angle
        else:
            clockwise = True

    else:#delta angle > 0
        #check if it is bigger than 180 degrees
        if(delta_angle / math.pi > 1.0):
            clockwise = True
            delta_angle = math.pi*2 - delta_angle 
        else:
            clockwise = False
             
    rotate(abs(delta_angle),clockwise)


def draw_from_files():
    """Draw a figure from the coordinates located in a file"""
    rospy.loginfo("Draw a figure from the coordinates located in a file")
    if(rospy.has_param("/points")):
        coordinates = rospy.get_param("/points")  #it returns a list of a dict of coordinates
        for p in coordinates:
            go_to(float(p['x']),float(p['y']))    
    else:
        rospy.logerr("Error. There is not figure for drawing")

    
def change_pen(): 
    """Call the turtlesim service to change pen"""
    rospy.loginfo("Call the turtlesim service to change pen")     
    rospy.wait_for_service('/turtle1/set_pen') 
    setpen_srv = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    setpen_srv(255,255,255,4,0)  #/R G B WIDTH HIDE()


def change_background(r,g,b):   
    """Call the turtlesim service to change background color"""
    rospy.loginfo("Call the turtlesim service to change background color")   
    rospy.set_param('background_r',r)
    rospy.set_param('background_g',g)
    rospy.set_param('background_b',b) 
    clear_traces()   

def clear_traces():
    """Call the turtlesim service to clear traces"""
    rospy.loginfo("Call the turtlesim service to clear traces")  
    rospy.wait_for_service('/clear')
    clear_srv = rospy.ServiceProxy('/clear', Empty)
    clear_srv()


if __name__ == '__main__':
    try:
        #initialize node
        rospy.init_node('move_turtle_node' , anonymous = True)
        
        #Publishe to topic
        vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)

        #Suscribe to topic
        pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)

        #Declare a service
        srv = rospy.Service('playpause', PlayOrPause,srvCallback)
        
        #Declare server for rqt_reconfigure
        rqt = Server(paramConfig,speedCallback)

        rospy.sleep(1)
        
        clear_traces()
        draw_from_files()




    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
