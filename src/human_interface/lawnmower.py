#!/usr/bin/env python3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d

from math import sqrt, atan2
import rospy
from nav_msgs.msg import Odometry
from blue_rov_custom_integration.srv import *
from keyboard import is_pressed
import numpy as np

ax3d = plt.axes(projection='3d')

ax = plt.figure()
plt.xlim(-20,20)
plt.ylim(-20,20)

ax3d.axes.set_xlim3d(left=-5, right=5) 
ax3d.axes.set_ylim3d(bottom=-5, top=5) 
ax3d.axes.set_zlim3d(bottom=-5, top=5) 

def plot_waypoints(x,y,z):
    plt.plot(x,y)
    ax3d.plot3D((x),(y),z,label='blah')
    plt.show()


Z_OFFSET_SET = False
DY_SET = False
DX_SET = False
DZ_SET = False
TH_OFFSET_SET = False
LINEAR_OFFSET_SET = False
READY_FOR_FULL_PATH = False
RESET_NEEDED = False


SEND_HOME_WP_ONS = True
SEND_RESET_ONS = True
SEND_FULL_PATH_ONS = True


Dx = 0
dx = 0
Dy = 0
dy = 0
Dz = 0
thz_offset = 0


x = []
y = []
z = []
thz = []

step = 5


def reset_configuration():
    '''
    reset configuration data
    '''
    global Dx, Dy, Dz, dx, dy, dz, thz_offset, x, y, z, thz, step
    global Z_OFFSET_SET, DY_SET, DX_SET, DZ_SET, TH_OFFSET_SET, LINEAR_OFFSET_SET, READY_FOR_FULL_PATH, RESET_NEEDED

    Dx = 0
    dx = 0
    Dy = 0
    dy = 0
    Dz = 0
    thz_offset = 0


    x = []
    y = []
    z = []
    thz = []

    step = 0

    Z_OFFSET_SET = False
    DY_SET = False
    DX_SET = False
    DZ_SET = False
    TH_OFFSET_SET = False
    LINEAR_OFFSET_SET = False
    READY_FOR_FULL_PATH = False
    RESET_NEEDED = False




def lawn_mower(_th_offset):
    '''
    lawnmore path generator.  generates path then sends wps to textfile to be used by system.
    '''
    global x, y, z, thz, Dx, Dz, dy

    Dy = 5
    dy = Dy/step

    # Dx and Dz set together
    Dx = 7




    # Logic used to create path. NOTE: the left and right values are flipped in relation to the trajectory due to the way the traj was originally coded
    Left = True
    Right = False
    Down1 = False
    Down2 = False
    rotate_down = False
    rotate_left = False
    rotate_right = False


    # th_d = 0#180+160
    th = _th_offset
    
    # Starting position
    x.append(0)
    y.append(0)
    z.append(0)
    thz.append(0)

    # Append all additional positions using logic
    for i in range(0,step*4*2):
        if rotate_down:
            x.append(x[i])
            y.append(y[i])
            z.append(z[i])
            thz.append(4.71)
            rotate_down = False
        elif rotate_left:
            x.append(x[i])
            y.append(y[i])
            z.append(z[i])
            thz.append(0)
            rotate_left = False
        elif rotate_right:
            x.append(x[i])
            y.append(y[i])
            z.append(z[i])
            thz.append(3.14)
            rotate_right = False
        elif Left:
            x.append(x[i] + Dx)
            y.append(y[i])
            z.append(z[i]+Dz)
            thz.append(0)
            Left = False
            rotate_down = True
            Down1 = True
        elif Down1:
            x.append(x[i])
            y.append(y[i]-dy) # NOTE: you have not defined dy yet.  Will need to us Dy and camera FOV information to determine this. 
            z.append(z[i])
            thz.append(4.71)
            Down1 = False
            rotate_right = True
            Right = True
        elif Right:
            x.append(x[i] - Dx)
            y.append(y[i])
            z.append(z[i]-Dz)
            thz.append(3.14)
            Right = False
            rotate_down = True
            Down2 = True
        else:
            x.append(x[i])
            y.append(y[i]-dy)
            z.append(z[i])
            thz.append(4.71)
            rotate_left = True
            Left = True


    # Offset path by th offset 
    for i in range(0,len(x)):
        xx = x[i]
        x[i] = x[i]*np.cos(th) - y[i]*np.sin(th)
        y[i] = y[i]*np.cos(th) + xx*np.sin(th)
        thz[i] = thz[i] + th             # Was checked in ploting script and it worked       # Start here if there are issues elkfjveflkvh <<<<<<<<<<<<<<---- Look at me
        if thz[i] != abs(thz[i]):
            thz[i] = 2*3.14 + thz[i]
        
        ax3d.quiver(x[i],y[i],z[i],np.cos(thz[i]), np.sin(thz[i]), 0)

    print('Dx: ', Dx)
    print('Dy: ', Dy)
    print('Dz: ', Dz)
    print('step: ', step)
    print('thz_offset', thz_offset)

    print('X: ', x)
    print('Y: ', y)
    print('Z: ', z)
    print('YAW: ', thz)
    

    # plot_waypoints(x,y,z)

    # Write path into text file to be used by system
    with open('/home/mason/catkin_ws/src/blue_rov_custom_integration/src/human_interface/readwaypoint.txt', 'w') as f:
        for i in range(len(x)):
            f.write(str(x[i]))
            f.write(' ')
            f.write(str(y[i]))
            f.write(' ')
            f.write(str(z[i]))
            f.write(' ')
            f.write(str(thz[i]))
            f.write('\n')



def return_home(y_current, x_current, z_current):
    '''
    used to write the necessary wps into system text file to get rov back to where its origin is
    '''
    yaw = atan2(-y_current,-x_current)
    if yaw != abs(yaw):
        yaw = 2*3.14 + yaw
    with open('/home/mason/catkin_ws/src/blue_rov_custom_integration/src/human_interface/readwaypoint.txt', 'w') as f:
        f.write(str(x_current))
        f.write(' ')
        f.write(str(y_current))
        f.write(' ')
        f.write(str(z_current))
        f.write(' ')
        f.write(str(yaw))
        f.write('\n')
        f.write(str(0))
        f.write(' ')
        f.write(str(0))
        f.write(' ')
        f.write(str(0))
        f.write(' ')
        f.write(str(yaw))
        f.write('\n')
        f.write(str(0))
        f.write(' ')
        f.write(str(0))
        f.write(' ')
        f.write(str(0))
        f.write(' ')
        f.write(str(0))




def system_callback(data):
    '''
    main path generator call back used to generate desiered planes for paths to be generated on as well as calling the path to then be generated

    Once configuration data hae been set the communication between the two system nodes goes as follows:
        1. Communication to ROSHUM to 
    '''

    global thz_offset, TH_OFFSET_SET, DY_SET, DX_SET, DZ_SET, Z_OFFSET_SET, READY_FOR_FULL_PATH, RESET_NEEDED, LINEAR_OFFSET_SET
    global Dy, Dx, Dz, dy, step
    global SEND_HOME_WP_ONS, SEND_FULL_PATH_ONS


    if is_pressed('h'): # linear offset that gets executed within ROVMAV.. outcome of odom data is sent here
        LINEAR_OFFSET_SET = True
        print('linear done')
    if is_pressed('j'): # thz offset that gets used in offseting x,y,and th data
        thz_offset = data.pose.pose.orientation.z
        TH_OFFSET_SET = True
        print('angular done')

    if TH_OFFSET_SET and LINEAR_OFFSET_SET:
        if is_pressed('n'): # ROV is in furthest desired ship wreck axial distance. This key is then pressed
            Dy = sqrt(pow(data.pose.pose.position.x,2)+pow(data.pose.pose.position.y,2))
            dy = Dy/step
            DY_SET = True
            SEND_HOME_WP_ONS = True
            print('dy set')

        if is_pressed('v'): # ROV is in furthest desired ship wreck lateral distance. Additionally the ROV is at the desired Z distance inorder to create a linear path that is parrallel with the ship wreck
            Dx = sqrt(pow(data.pose.pose.position.x,2) + pow(data.pose.pose.position.y,2))
            Dz = data.pose.pose.position.z
            DX_SET = True
            DZ_SET = True
            SEND_HOME_WP_ONS = True
            print('dx dz set')

        if is_pressed('r') and READY_FOR_FULL_PATH and SEND_FULL_PATH_ONS: # Lawnmower path is created.  All needed components must have been set
            lawn_mower(thz_offset)
            rospy.wait_for_service('wpTrigger')
            wpSetClient = rospy.ServiceProxy('wpTrigger',wpTrigger)
            ROSHUMresponse = wpSetClient(True)
            if ROSHUMresponse.checkComplete:
                RESET_NEEDED = True
            
            SEND_FULL_PATH_ONS = False

            READY_FOR_FULL_PATH = False  # acts as the ONS for this button

        if is_pressed('b') and SEND_HOME_WP_ONS: # Send return home wps to text file
            return_home(data.pose.pose.position.y, data.pose.pose.position.x, data.pose.pose.position.z)
            rospy.wait_for_service('wpTrigger')
            wpSetClient = rospy.ServiceProxy('wpTrigger',wpTrigger)
            ROSHUMresponse = wpSetClient(True)
            if ROSHUMresponse.checkComplete:
                RESET_NEEDED = True
            SEND_HOME_WP_ONS = False

        if is_pressed('u'):
            SEND_HOME_WP_ONS = True

        if RESET_NEEDED:
            rospy.wait_for_service('resetTrigger')
            resetByteClient = rospy.ServiceProxy('resetTrigger',resetTrigger)
            the_response = resetByteClient(True)
            if the_response.resetRecieved:
                RESET_NEEDED = False

        if is_pressed('y'):
            reset_configuration()

        if DY_SET and DX_SET and DZ_SET and TH_OFFSET_SET:
            READY_FOR_FULL_PATH = True



        


if __name__ == '__main__':

    # Dx = 10
    # dx = 0

    # Dy = 0
    # dy = 2

    # Dz = 5

    # step = 5


    # lawn_mower(20*3.14/180)

    rospy.init_node('LawnMower')

    systemServer = rospy.Subscriber('ROV_ODOMETRY',Odometry,system_callback)

    rospy.spin()















