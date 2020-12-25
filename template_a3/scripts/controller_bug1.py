#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from quat2euler import quat2euler

import numpy
import time
import math
from math import sin, cos, atan2, pi, fabs
import matplotlib.pyplot as plt

## Define global variables
pose = [0,0,0]
range_data = [0,0,0]
front=0
inp=[0,0]
max_dist=1.0
xData=[]
yData=[]

def callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
    
def laser_callback(msg):
    global range_data, front
    data=msg.ranges    
    range_data=[min(data[480:719]),min(data[240:479]),min(data[0:239])]
    # range_data=[min(data[480:719]),min(data[310:479]),min(data[0:309])]
    front = min(data[300:420])

def Waypoints(t):
    x  = 5
    y  = 0
    return [x,y] 

def eulerdist(a,b):
    return ((a[0]-b[0])**2+(a[1]-b[1])**2)**0.5    


### Map angle to within [-pi, pi]
def map_angle(theta):
    if(theta > pi):
        theta = -(2*pi-theta)
    elif(theta < -pi):
        theta = (2*pi+theta)
    return theta

### Saturate control to reasonable values
def sat(x, Th):
    if x<-Th:
        return -Th
    if x > Th:
        return Th
    return x

def go_to_point(theta_err,dist_error):
    global pose, range_data,inp
    K1=0.4  ##not aggressive
    K2=2.0  ## 4 is aggresive even 3 

    if math.fabs(theta_err)<=15*pi/180:
        linV = sat(K1*dist_error*cos(theta_err), 0.20)
    else:
        linV = 0   
    angV = sat(K2*theta_err, 0.5)
    inp=[linV,angV]

def wall_follow(theta_err,dist_error):
    global pose,range_data,inp,front,max_dist
    K2=2
    K1=0.1


        #follow_wall
    a=1 if range_data[0]<max_dist else 0
    b=1 if range_data[1]<max_dist else 0 
    c=1 if range_data[2]<max_dist else 0            

    wall_dir=0
    #personal note currently: pi/6 correspo to pi/3 and pi/3 corresponds to pi/2
    if a==0 and b==0 and c==1:
        # print('go straight')
        wall_dir=0    
        # print(a,b,c)
    elif a==0 and b==1 and c==1:
        # print('go pi/3')
        wall_dir=pi/6 
    elif a==1 and b==1 and c==1:
        # print('go 2pi/3')
        wall_dir=pi/3
    elif a==1 and b==0 and c==1:
        wall_dir=0
        # print('go straight')    
    elif a==0 and b==1 and c==0:
        wall_dir=pi/6
        # print('go left')
    elif a==1 and b==1 and c==0:
        wall_dir=pi/3
        # print('go 2pi/3 1')
    elif a==1 and b==0 and c==0:
    # else :
        wall_dir=pi/3   
        # print('go 2pi/3 2')    
    elif a==0 and b==0 and c==0:
        wall_dir=-pi/6   
        # print('go -pi/3 or turning right')        
    else:
        wall_dir=-1
        print("stop the bot")  
        print(a,b,c)                       
    

    # if front>max_dist and wall_dir==pi/6:
    #     wall_dir=0
    #     print("going front ahead")    
    # if wall_dir==0 or wall_dir==pi/3 :

    angV = K2*wall_dir            
    linV = K1*cos(wall_dir)
    if wall_dir==2*pi/3 or wall_dir==pi/3:            
        linV = 0
    elif wall_dir==-1:
        angV=0
        linV=0
        print('stopped the bot') 

    inp=[linV,angV]    



    pass        

## This is where we will calculate error and apply the proportional control to 
##  compute linear velocity and angular velocity for the turtlebot 
def control_loop():
    global pose, range_data,front,inp,max_dist,xData,yData
    hitPt=[]
    startPt=[-5, 0]
    leavePt=[] #this will be the closest point
    gap=0.5   #distance from leave point and hit point
    gapAngle=2#degree error to transition back to point follow
    gapInit=0.8 #to switch from circumventing to go to closest point
    distMin=30
    print("minimum distance currently ", distMin)
    
    rospy.init_node('differential_bot_con')
    pub = rospy.Publisher('/bot_0/cmd_vel', Twist, queue_size=10)
    pubw = rospy.Publisher('/bot_0/waypoint', String, queue_size=10)
    rospy.Subscriber('/bot_0/odom', Odometry, callback)
    sub=rospy.Subscriber('/bot_0/laser/scan',LaserScan,laser_callback)
    
    ## Setting the rate for loop execution
    rate = rospy.Rate(10) 
    
    ## Twist values to move the robot
    velocity_msg = Twist()

    i = 0
    wp= Waypoints(i)
    dist_error = 2  # some random value
    state_index=0  # go to point=0 : ; circumvent the wall=1:; circumvent and go to closest point=2
 ### Apply the proportional control
    count=0       
    theta_des=atan2(wp[1]-startPt[1],wp[0]-startPt[0])
    # leavePt=startPt
    # print(hitPt)
    while not rospy.is_shutdown():


        ### Compute errors
        x_err = wp[0]-pose[0]
        y_err = wp[1]-pose[1]
        theta_ref = atan2(y_err, x_err)  
        dist_error = (x_err**2 + y_err**2)**0.5   
        theta_err = map_angle(theta_ref - pose[2])

        velocity_msg = Twist()
        velocity_msg.linear.x=0
        velocity_msg.angular.z=0

        if state_index==0:
            # print(pose[0:2])
            # if eulerdist(leavePt,pose[0:2])>gap and count>30:
            if count>20:  #count not very important
                if math.fabs(theta_err)<pi/6 and range_data[1]<max_dist:
                    state_index=1
                    hitPt=pose[0:2]
                    print("CHANGING TO: wall circumvent: centre obstacle: follow wall ",hitPt )
                    distMin=30
                    count=0
                elif theta_err>pi/6 and theta_err<pi/2 and range_data[0]<max_dist:
                    state_index=1
                    hitPt=pose[0:2]
                    count=0
                    distMin=30
                    print("CHANGING TO: wall circumvent: left obstacle : follow wall ",hitPt)
                elif theta_err<-pi/6 and theta_err>-pi/2 and range_data[2]<max_dist:
                    state_index=1  
                    hitPt=pose[0:2]
                    count=0
                    distMin=30
                    print("CHANGING TO: wall circumvent: right obstacle: follow wall ",hitPt)
                else:
                    # print("go to point")  
                    go_to_point(theta_err,dist_error)

            else:  
                # print("go to point")   
                go_to_point(theta_err,dist_error)

                 
                # pub.publish(velocity_msg)
        elif state_index==1:

            if eulerdist(hitPt,pose[0:2])<gapInit and count>150:  #count and gap very important
                # print(leavePt,pose[0:2])               

                state_index=2  
                print("time passed ", count/10)
                count=0
                print("CHANGING TO: go to closest point along the wall ",leavePt)
                # rospy.signal_shutdown('debugging mode')

            else: 
                # print("Follow the wall")
                wall_follow(theta_err,dist_error)
                if dist_error<distMin:
                    distMin=dist_error
                    leavePt=pose[0:2]
                    print("current leave point ", leavePt, "minimum distance ",distMin )

        elif state_index==2:

            if eulerdist(leavePt,pose[0:2])<gapInit and count>80:  #count and gap very important
                # print(hitPt,pose[0:2])               
                # leavePt=pose[0:2]
                state_index=0  
                count=0
                print("CHANGING TO: go to point")
            else:
                # print("Follow the wall")
                wall_follow(theta_err,dist_error)
             
        elif state_index==-1:
            print('i dont know how we reached here') 


        velocity_msg.angular.z = inp[1]
        velocity_msg.linear.x = inp[0] 
        pub.publish(velocity_msg)  
        # print( "linear velocity is", velocity_msg.linear.x, " angular velocity " ,velocity_msg.angular.z," state ", state_index )    
        count=count+1    
        xData.append(pose[0])
        yData.append(pose[1])    
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Waiting for gazebo to start")
        time.sleep(5)
        print("Starting the control loop")
        control_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        plt.plot(xData,yData)
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.title('Robot Trajectory:BUG 1')

        # plt.hold()
        rectangle = plt.Rectangle((-0.5, -2), 1, 4, fc='r')
        plt.gca().add_patch(rectangle)
        plt.show()
        print("Done")       