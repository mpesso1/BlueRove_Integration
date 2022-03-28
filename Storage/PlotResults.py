import matplotlib
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
#plt.style.use('classic')

'''
* rosrun package_name node > (pwd to file)

* if there are any issues make sure the file is saved. Ctrl+s in file &/or date and the file name are correct.
* inorder for there to be any data ouput BlueRov.traj(1) or BlueRov.surf(1) must be called within the pp_node.

* Reminider that if you have any trouble finding the node names to call rosrun then just check the CMakeLists.txt file
'''


# Month/Day
date = "3/22/"

# path file
path = "/path1.txt"

data = np.loadtxt("/home/mason/catkin_ws/src/blue_rov_custom_integration/Storage/" + date + path,dtype=float)


def traj(): # function for plotting out multiple trajectories and objects from shell.
            # Possile issues:   conditional for if statement and how data is coming in from cpp -gpmp.cpp
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    x = np.zeros(np.shape(data))
    y = np.zeros(np.shape(data))
    z = np.zeros(np.shape(data))

    obj_num = 1
    for i in range(0,((np.shape(data)[0]-1)//3)+1): 
        if sum(data[i*3,-6:-1]) == 0:
            for j in range(i*3,np.shape(data)[0]):
                print(obj_num)
                lab = 'obj' + str(obj_num)
                ax.scatter(data[j,0],data[j,1],data[j,2],'yellow',s=400) #,label=lab)
                obj_num =  obj_num+1
        else:
            #print(i)
            x[i] = data[i*3,:]
            y[i] = data[i*3+1,:]
            z[i] = data[i*3+2,:]
            lab = "Traj " + str(i)
            ax.plot3D(x[i],y[i],z[i]) #,label=lab)
    plt.legend()
    plt.show()

def surf():
    plt.imshow(data)
    plt.imshow(data,extent=[0,5,0,5],origin='lower',cmap='gray')
    plt.colorbar()
    #plt.axis(aspect='image')
    plt.show()

traj()




# There are currently 3 sensitivity paramaters that may effect the output of the motion planning algorithm
# 1. Whole:
#       this parameter scales the entire update rule.  What this means is that high valaues will cause the updates to have large steps
#       The value that this whole sensitivity scales is the difference between the cost of staying near the initial trajectory and the 
#       the cost of object gradients
# 2. Ability:
#       this parameter scales the gaussian cost.  What this means is that deceasing this parameter will allow for the obstical cost to
#       produce a larger effect on the overall cost.

# --> new_trajectory = current_trajectory - (Whole)[(Ability)(Gaussian_cost) - (Obj_cost)]

# 3. Power:
#       A parameter that scales the covariance in the prior.


