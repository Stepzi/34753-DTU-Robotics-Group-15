import matplotlib.pyplot as plt
import numpy as np

class DigitalTwin():
    def __init__(self,rbt):
        plt.ion()

        self.__rbt = rbt
        self.__fig = plt.figure()
        self.__ax = self.__fig.add_subplot(111, projection='3d')
        self.__ax.set_xlim(-0.2,0.2)
        self.__ax.set_ylim(-0.2,0.2)
        self.__ax.set_zlim(0,0.4)
        self.__ax.set_xlabel('X')
        self.__ax.set_ylabel('Y')
        self.__ax.set_zlabel('Z')
        self.__l1 = self.__ax.plot([],[],[], "k", marker=".", markersize=10, linewidth=3)


    def draw_arm(self):
        T = self.__rbt.fwd_kin()
        self.__l1[0].set_data_3d([T[0][0,3],T[1][0,3],T[2][0,3],T[3][0,3],T[4][0,3]],
                                 [T[0][1,3],T[1][1,3],T[2][1,3],T[3][1,3],T[4][1,3]],
                                 [T[0][2,3],T[1][2,3],T[2][2,3],T[3][2,3],T[4][2,3]])
        self.__fig.canvas.draw()
        plt.pause(0.05)
        
    def close(self):
        plt.close()
    























# # from dynamixelArm import RobotArm  
# import time
# import matplotlib.pyplot as plt
# plt.ion()

# x = [0,1,3,5]
# y = [2,2,5,5]
# z = [0,1,1,5]


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlim(-2,2)
# ax.set_ylim(-2,2)
# ax.set_zlim(-2,2)

# l1 = ax.plot(x, y, z)



# while True:
# #     time.sleep(1)
    
 
#     # drawing updated values
#     fig.canvas.draw()
 
#     # This will run the GUI event
#     # loop until all UI events
#     # currently waiting have been processed
#     # fig.canvas.flush_events()

#     l1[0].set_data_3d(np.random.uniform(-1,1,3), np.random.uniform(-1,1,3), np.random.uniform(-1,1,3))
#     plt.pause(0.01)
