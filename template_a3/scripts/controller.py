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
    global range_data
    data=msg.ranges    
    range_data=[min(data[480:719]),min(data[240:479]),min(data[0:239])]
    # range_data=[min(data[480:719]),min(data[310:479]),min(data[0:309])]

def Waypoints(t):
    x  = 5
    y  = 0
    return [x,y] 


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

## This is where we will calculate error and apply the proportional control to 
##  compute linear velocity and angular velocity for the turtlebot 
def control_loop():
    global pose, range_data,xData,yData
    
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
    state_index=0  # state_index=0 : go to point; =1: follow the wall
    
    while not rospy.is_shutdown():
    

        ### Compute errors
        x_err = wp[0]-pose[0]
        y_err = wp[1]-pose[1]
        theta_ref = atan2(y_err, x_err)  

        dist_error = (x_err**2 + y_err**2)**0.5
        
        theta_err = map_angle(theta_ref - pose[2])

        # ### Debug string 
        # print("\n heading:{:0.5f},\tref:{:0.5f},\terror:{:0.5f}".format(pose[2], theta_ref, theta_err))
        
        ### Apply the proportional control
        K1=0.4  ##not aggressive
        K2=3.0  ## aggressive        

        #check condition
        print("\n heading:{:0.5f},\tref:{:0.5f},\terror:{:0.5f}".format(range_data[0], range_data[1], range_data[2]))
        if fabs(theta_err)>pi/2:
            if min(range_data[0],min(range_data[1],range_data[2]))>max_dist:
                state_index=0
                print("state_index=0")
            else:
                state_index=1
                print("state_index=1 a")
        else:
            if fabs(theta_err)<pi/6:
                if range_data[1]>max_dist:
                    state_index=0  
                else:
                    state_index=1
                    print("state_index=1 b")
            elif theta_err>pi/6 and theta_err<pi/2:
                if range_data[0]>max_dist:
                    state_index=0  
                else:
                    state_index=1
                    print("state_index=1 c")
            elif theta_err<-pi/6 and theta_err>-pi/2:
                if range_data[2]>max_dist:
                    state_index=0  
                else:
                    state_index=1         
                    print("state_index=1 d")    
            else :
                print('check me state_index end')  
                state_index=-1 #this means error                                          



        velocity_msg = Twist()
        if state_index==0: 
            print("go to point")   
            if math.fabs(theta_err)<=25*pi/180:
                K1=0.4
                velocity_msg.linear.x = sat(K1*dist_error*cos(theta_err), 0.20)
            else:
                velocity_msg.linear.x = 0   
            velocity_msg.angular.z = sat(K2*theta_err, 0.5)
        elif state_index==1:
            print("Follow the wall")
            # pass
            #follow_wall
            a=1 if range_data[0]<max_dist else 0
            b=1 if range_data[1]<max_dist else 0 
            c=1 if range_data[2]<max_dist else 0            

            wall_dir=0
            if a==0 and b==0 and c==1:
                print('go straight')
                wall_dir=0
            elif a==0 and b==1 and c==1:
                print('go pi/3')
                wall_dir=pi/3 
            elif a==1 and b==1 and c==1:
                print('go 2pi/3')
                wall_dir=2*pi/3
            elif a==1 and b==0 and c==1:
                wall_dir=0
                print('go straight')    
            elif a==0 and b==1 and c==0:
                wall_dir=pi/3
                print('go left')
            elif a==1 and b==1 and c==0:
                wall_dir=2*pi/3
                print('go 2pi/3 1')
            elif a==1 and b==0 and c==0:
            # else :
                wall_dir=2*pi/3   
                print('go 2pi/3 2')    
            else:
                wall_dir=-1
                print("stop the bot")  
                print(a,b,c)                       



            # if wall_dir==0 or wall_dir==pi/3 :
            velocity_msg.angular.z = K2*wall_dir            
            velocity_msg.linear.x = 0.1*cos(wall_dir)
            if wall_dir==2*pi/3 or wall_dir==pi/2:            
                velocity_msg.linear.x = 0
            elif wall_dir==-1:
                velocity_msg.angular.z=0
                velocity_msg.linear.x=0
                print('stopped the bot') 
        elif state_index==-1:
            print('i dont know how we reached here')        



        # velocity_msg.angular.z = 2
        # velocity_msg.linear.x = 0                
        pub.publish(velocity_msg)
        xData.append(pose[0])
        yData.append(pose[1])        
        ### for dynamic plot
        # pubw.publish("[{},{}]".format(wp[0], wp[1]))
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
        plt.title('Robot Trajectory:BUG 0')

        # plt.hold()
        rectangle = plt.Rectangle((-0.5, -2), 1, 4, fc='r')
        plt.gca().add_patch(rectangle)
        plt.show()
        print("Done")       

