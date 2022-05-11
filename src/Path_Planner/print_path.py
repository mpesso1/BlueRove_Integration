from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def traj(): # function for plotting out multiple trajectories and objects from shell.
            # Possile issues:   conditional for if statement and how data is coming in from cpp -gpmp.cpp

    ax = plt.axes(projection='3d')

    x = []
    y = []
    z = []


    obj_num = 1
    Traj_num = 1
    cout = 0
    doneWithTraj = False
    with open('/home/mason/catkin_ws/src/blue_rov_custom_integration/src/Path_Planner/store.txt') as f:
        for line in f:
            s = line.split()

            for i in range(0,len(s)):
                s[i] = float(s[i])

            if cout > 2:
                cout = 0

            if len(s) == 3:
                lab = 'obj' + str(obj_num)
                ax.scatter(float(s[0]),float(s[1]),float(s[2]),'yellow',s=100,label=lab)
                obj_num =  obj_num+1
                doneWithTraj = True

            elif doneWithTraj:
                x_vec = []
                y_vec = []
                for i in range(0,len(s)):
                    x_vec.append(np.cos(s[i]))
                    y_vec.append(np.sin(s[i]))

                    ax.quiver(x[i],y[i],z[i],np.cos(s[i]),np.sin(s[i]),0)

            else:
                if cout == 0:
                    x = s
                if cout == 1:
                    y = s
                if cout == 2:
                    z = s
                    # print((z))
                    lab = "Traj " + str(Traj_num)
                    ax.plot3D((x),(y),(z),label=lab)
                    
            cout += 1





                


    ax.axes.set_xlim3d(left=-0, right=5) 
    ax.axes.set_ylim3d(bottom=-0, top=5) 
    ax.axes.set_zlim3d(bottom=-0, top=5) 

    plt.legend()
    plt.show()

traj()
#surf()


