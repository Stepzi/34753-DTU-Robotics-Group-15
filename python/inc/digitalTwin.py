import matplotlib.pyplot as plt
import numpy as np
import time

class DigitalTwin():
    def __init__(self,rbt):
        plt.ion()

        self.__rbt = rbt
        self.__fig_3D = None
        self.__fig_jointSpace = None
        

    def draw_arm(self,draw_3D = True, draw_jointSpace = True):
        if draw_3D:
            self._draw_3D()
        if draw_jointSpace:
            self._draw_jointSpace()

        plt.pause(0.05)

    def _init_draw_3D(self):            
            self.__fig_3D = {}            
            self.__fig_3D['fig'] = plt.figure()
            self.__fig_3D['ax1'] = self.__fig_3D['fig'].add_subplot(111, projection='3d')
            self.__fig_3D['ax1'].set_xlim(-0.2,0.2)
            self.__fig_3D['ax1'].set_ylim(-0.2,0.2)
            self.__fig_3D['ax1'].set_zlim(0,0.4)
            self.__fig_3D['ax1'].set_xlabel('X')
            self.__fig_3D['ax1'].set_ylabel('Y')
            self.__fig_3D['ax1'].set_zlabel('Z')
            self.__fig_3D['l1'] = self.__fig_3D['ax1'].plot([],[],[], "k", marker=".", markersize=10, linewidth=3)
            self.__fig_3D['l2'] = self.__fig_3D['ax1'].plot([],[],[], "r",linestyle='None', marker=".", markersize=10, linewidth=3)
            
            
    def _draw_3D(self):
        if self.__fig_3D is None:
            self._init_draw_3D()

        T = self.__rbt.fwd_kin()
        self.__fig_3D['l1'][0].set_data_3d([T[0][0,3],T[1][0,3],T[2][0,3],T[3][0,3],T[4][0,3]],
                                 [T[0][1,3],T[1][1,3],T[2][1,3],T[3][1,3],T[4][1,3]],
                                 [T[0][2,3],T[1][2,3],T[2][2,3],T[3][2,3],T[4][2,3]])
        self.__fig_3D['l2'][0].set_data_3d([T[5][0,3]],
                                 [T[5][1,3]],
                                 [T[5][2,3]])
        # self.__fig_3D['fig'].canvas.draw()

    def _init_draw_jointSpace(self):
        self.__fig_jointSpace = {} 
        
        self.__fig_jointSpace['fig'], self.__fig_jointSpace['ax1'] = plt.subplots(4, 1)
        self.__fig_jointSpace['fig'].subplots_adjust(hspace=0.5)  # Adjust space between subplots
        self.__time_history = []
        self.__joint_angles_history = [[] for _ in range(4)]  # Assuming 4 joints
        self.__last_sample = 0
        self.__fig_jointSpace['l1'] = [None for _ in range(4)]

        for i in range(4):
            self.__fig_jointSpace['ax1'][i].set_xlim(-10, 0)  # Time window of 10 seconds
            self.__fig_jointSpace['ax1'][i].set_ylim(-np.pi, np.pi)  # Assuming joint angles are in radians
            self.__fig_jointSpace['ax1'][i].set_ylabel(f'Joint {i + 1} [rad]')
            self.__fig_jointSpace['ax1'][i].grid()
            self.__fig_jointSpace['l1'][i] = self.__fig_jointSpace['ax1'][i].plot([], [], label=f'Joint {i + 1}')

        self.__fig_jointSpace['ax1'][3].set_xlabel('Time [s]')  # Label x-axis only for the last subplot
        


    def _draw_jointSpace(self):
        if self.__fig_jointSpace is None:
            self._init_draw_jointSpace()
            
        # Update joint angles
        joint_angles = self.__rbt.get_cached_jointAngles()  
        current_time = time.time()  
        
        dt_sample = current_time - self.__last_sample
        self.__last_sample = current_time
        
        # Append new data
        self.__time_history = [x - dt_sample for x in self.__time_history]
        
        for i in range(len(joint_angles)):
            self.__joint_angles_history[i].append(joint_angles[i])
        
        self.__time_history.append(0)

        # Keep only the last 10 seconds of data
        while self.__time_history and self.__time_history[0] < - 10:
            self.__time_history.pop(0)
            for j in range(4):
                self.__joint_angles_history[j].pop(0)

        # Update joint angle plots
        for i in range(4):
            self.__fig_jointSpace['l1'][i][0].set_data(self.__time_history, self.__joint_angles_history[i])

        # self.__fig_jointSpace['fig'].canvas.draw()
        
    def close(self):
        if self.__fig_3D is not None:
            plt.close(self.__fig_3D['fig'])
        if self.__fig_jointSpace is not None:  
            plt.close(self.__fig_jointSpace['fig'])
