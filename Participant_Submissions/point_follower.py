#!/usr/bin/env python3

#Webots import statments
from controller import Robot
from controller import GPS , Motor ,  InertialUnit
import math

## ------------------------------------------------------------------------
#Edit point_follower and Sketch functions for Task 1, 2 , 3
def pythag(initial, target):
    #where initial and target are both 2 member lists representing coordinate points
    #returns distance between points
    
    return math.sqrt(((target[0]-initial[0])**2)+((target[1]-initial[1])**2))

def thetaRetrieval(initial, target):
    #where initial and target are both 2 member lists representing coordinate points
    #returns absolute angle between points
    return atan2(target[1]-intial[1], target[0]-initial[0])

def point_follower(current,target):
    # Implement a Controller to reach the goal.

    # current is [x,y,theta]
    # i.e x = current[0], y = current[1], theta = current[2]  
    # goal locations can be fetched the same way
    # goal is [x,y,theta]
    # leftSpeed is a float
    # rightSpeed is a float

    # Controller code here to reach from "current" location to "goal" location. 
    print("hey i just got called")
    xDiff = current[0]-target[0]
    yDiff = current[1]-target[1]
    leftSpeed = 1
    rightSpeed = 1.00025
    difference = target[2]-current[2]
    print("heres the difference")
    print(difference)

    if difference>0:
        leftSpeed = 0
        rightSpeed = 3
    if difference<0:
        rightSpeed = 0
        leftSpeed = 3
    if difference==0:
        rightSpeed = 0
        leftSpeed = 0
        
    if difference>0.003: 
        rightSpeed = rightSpeed + 0.1353 
        print("adjusted right speed up")
    if difference<(-0.003):
        leftSpeed = leftSpeed + 0.0048
        print("adjusted right speed down")
    
    if (abs(xDiff)<0.1 and abs(yDiff)<0.1):
        rightSpeed = 0
        leftSpeed = 0
      
    # Sample velocities of 1.0 provided to make robot move straight by default. 
    return leftSpeed,rightSpeed

def Sketch():
    #PARAMETERIZE LINE ONE
    #stepSize = 1 #approximation of the function (lower value is closer to actual function)
    #xMax = 37 #x value at end of functioon
    #i=0
    #while i<xMax
     #   goal.append [i, sin(i), cos(i)]
      #  i+stepSize
        
    # Use this function to calculate waypoints that can trace the given curve in the world. 
    # This is optional you can also impelement everything in point_follower function.
    # Clue: You can think of equations for the curve and create waypoints to follow it.
    # How to add additional goal points to the goal vector :
    # Eg: to add [1,1,0] as goal
    # goal =[]
    # goal.append([1,1,0])
    # then to add [1,2,1.57] as goal
    # goal.append([1,2,1.57])
    # To access 1st goal : goal[0], 2nd goal : goal[1] and so on..
    
    #input list of points to create target function
    #goal for square
    goal = [[4,-4,1.57], [4,4,3.14], [-4,4,4.71], [-4,-4,0]] # [x,y,theta]
    return goal

## ------------------------------------------------------------------------
print("hello world")

#Initializing robot to access sensor data from the robot.
robot = Robot()
timestep = int(robot.getBasicTimeStep()) # Defined timestep (in msec) to enable sensors and define sensor frequency.

# Initialize and Enable GPS object to get X,Y location of robot
gps = robot.getGPS("gps")
gps.enable(timestep) # x, y, z location received at time difference equat to timestep.

# Initialize and Enable IMU object to get theta (orientation) of robot
imu = robot.getInertialUnit("imu")
imu.enable(timestep) # Theta recieved at time difference equat to timestep.

print("uwu")
# Initialize and Enable robot wheels
wheels_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
wheels = []
print("hello again")
for i in range(4):
    wheels.append(robot.getMotor(wheels_names[i]))
    wheels[-1].setPosition(float('inf')) # setting max position limit of each wheel to infinity to convert it to velocity mode 
    wheels[-1].setVelocity(0.0) # Setting zero velocity for all the wheels.

if __name__=="__main__":
    print("well we made it this far")
    #Optional Edit here ------------------------------------------------------------
    #Call the Sketch function here if you want to generate vector of goals just once
    # ------------------------------------------------------------------------------
    
    while robot.step(timestep) != -1:

        # Fetch current position of robot using GPS and IMU : x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
       # print("current x, y, theta of robot: ",current)
        
        #comment this default goal location if you caculate your own set of goals vector 
        goal = Sketch() # initial goal to initialize goal array
        print(current)
        
        ## ------------------------------------------------------------------------------
        # Edit here 
        # Call the Sketch function here if you want to generate vector of goals continuously
        # Use point_follower controller to trace the curve using the above generated waypoints   
        # point_follower should return leftSpeed and rightSpeed
        #for i in range (8):
        goal = [4, -4, 0]
        if gps.getValues()[0] == 4 or gps.getValues()[0] > 3.9:
            goal = [4, 4, 1.571]
            print("if statement ACTIVATED")
        if gps.getValues()[1] > 3.9:
            goal = [-4.01, 4, 3.1]
        if gps.getValues()[0]<-4.000001:
            goal = [-4, -4, 4.712]
        leftSpeed,rightSpeed = point_follower([gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]],goal)
        wheels[0].setVelocity(leftSpeed) # Front left wheel
        wheels[1].setVelocity(rightSpeed) # Front right wheel
        wheels[2].setVelocity(leftSpeed) # Rear left wheel
        wheels[3].setVelocity(rightSpeed) # Rear right wheel
      
        # for i, value in enumerate(goal):
            # print(i)
            # while round(imu.getRollPitchYaw()[2],2)!=value[2]:
                # leftSpeed,rightSpeed = point_follower(current,value)
                # wheels[0].setVelocity(leftSpeed) # Front left wheel
                # wheels[1].setVelocity(rightSpeed) # Front right wheel
                # wheels[2].setVelocity(leftSpeed) # Rear left wheel
                # wheels[3].setVelocity(rightSpeed) # Rear right wheel
                # print(1)
            
            # while not (round(gps.getValues()[1],2)==value[1] and round(gps.getValues()[0],2)==value[0]):
                # leftSpeed,rightSpeed = point_follower(current,value)
                # wheels[0].setVelocity(leftSpeed) # Front left wheel
                # wheels[1].setVelocity(rightSpeed) # Front right wheel
                # wheels[2].setVelocity(leftSpeed) # Rear left wheel
                # wheels[3].setVelocity(rightSpeed) # Rear right wheel
                # print(2)
         
            