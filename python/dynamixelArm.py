import numpy as np
from digitalTwin import DigitalTwin 
import threading
import time



# Add the Dynamixel SDK path to sys.path
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'DynamixelSDK/python/src')))

import dynamixel_sdk as dxl

class RobotArm():
    def __init__(self, device_name, baudrate=1000000, protocol_version=1.0):
        # initialize constants and configurations
        self.__ADDR_MX_TORQUE_ENABLE = 24
        self.__ADDR_MX_CW_COMPLIANCE_MARGIN = 26
        self.__ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
        self.__ADDR_MX_CW_COMPLIANCE_SLOPE = 28
        self.__ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
        self.__ADDR_MX_GOAL_POSITION = 30
        self.__ADDR_MX_MOVING_SPEED = 32
        self.__ADDR_MX_PRESENT_POSITION = 36
        self.__ADDR_MX_PUNCH = 48
        self.__TORQUE_ENABLE = 1
        self.__TORQUE_DISABLE = 0
        self.__DXL_MOVING_STATUS_THRESHOLD = 10  # Threshold for detecting movement completion
        self.__DXL_IDS = [1, 2, 3, 4]  # Motor IDs

        # Digital Twin
        self.twin = DigitalTwin(self)

        # Setup Multithreading:
        self.__lock = threading.Lock()
        self.__running = True
        self.__thread = threading.Thread(target=self._move_joints)
        

        # Robot Configuration
        self.__SV_joint_angles = [0.0,0.0,0.0,0.0] # [rad]
        with self.__lock:
            self.__PV_joint_angles = [0.0,0.0,0.0,0.0] # [rad]
        self.__motor_speeds = [2.0,2.0,2.0,2.0] # [rad/s] (0.0117,11.9)
       

        # Build Kinematic Chain
        self.Frames = self.assemble_robot()        

        # initialize __portHandler and __packetHandler
        self.__portHandler = dxl.PortHandler(device_name)
        self.__packetHandler = dxl.PacketHandler(protocol_version)

        # open port and set baudrate
        try: 
            self.__portHandler.openPort()
            self.__portHandler.setBaudRate(baudrate)
            print(f"Successfully opened port {device_name} and set baudrate")
            self.__has_hardware = True
        except:
            print(f"Failed to open port {device_name} and set baudrate")
            self.__has_hardware = False
            self.__thread.start()

        self.set_speed(self.__motor_speeds)

    def assemble_robot(self):
        frames = [] # List of frame Objects, ORDER is important, first {0} in global then all links, then the rest
        # Base Frame
        frames.append(Frame(T=np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 1, 0.05],
                                        [0, 0, 0, 1]])))
        # Link 1          
        frames.append(Frame(DH_params={'theta': 0,      'd':0.05,   'a': 0,     'alpha': np.pi/2,   'type': "revolute"}))
        # Link 2
        frames.append(Frame(DH_params={'theta': np.pi/2,'d':0,      'a': 0.093, 'alpha': 0,         'type': "revolute"}))
        # Link 3
        frames.append(Frame(DH_params={'theta': 0,      'd':0,      'a': 0.093, 'alpha': 0,         'type': "revolute"}))
        # Link 4
        frames.append(Frame(DH_params={'theta': 0,      'd':0,      'a': 0.05,  'alpha': 0,         'type': "revolute"}))
        # Tool
        frames.append(Frame(T=np.array([[1, 0, 0, 0.0],
                                        [0, 1, 0, 0.0],
                                        [0, 0, 1, 0.05],
                                        [0, 0, 0, 1]])))

        return frames 

    # Kinematic Methods
    def fwd_kin(self):
        with self.__lock:
            joint_angles = self.__PV_joint_angles

        T_global = [self.Frames[0].T_local(0)]  # Base Frame
        for i, frame in enumerate(self.Frames[1:]):
            if frame.is_Link:                
                T_global.append(T_global[-1] @ frame.T_local(joint_angles[i]))
            else:
                T_global.append(T_global[-1] @ frame.T_local())
            # T_global.append(T_global[-1] @ self.Frames[2].T_local(joint_angles[1]))
            # T_global.append(T_global[-1] @ self.Frames[3].T_local(joint_angles[2]))
            # T_global.append(T_global[-1] @ self.Frames[4].T_local(joint_angles[3]))
            # T_global.append(T_global[-1] @ self.Frames[5].T_local())
                   
        # if  not point == None:
        #     assert (point.shape == (4,4)), "reference point must be valid Transformation matrix"
        #     T_global.append(T_global[-1] @ point)

        return T_global          

    def inv_kin(self,gamma: float,origin: list, elbow="up",tool=None):
        """Computes the set of joint angles for desired tip position and orientation
        
        @param gamma: angle of stylus with horizontal plane [rad]
        @param origin: vector of  [x,y,z] in global frame of tip position [m]
        @param elbow: string "up"/"down" for elbow-up/-down solution
        @param tool: Local Transformation to tool in frame {4}
        @rtype: list
        @returns: gamma vector of joint angles
        """

        if tool is not None:
            x_tool_4 = tool[0,3]
            y_tool_4 = tool[1,3]
            z_tool_4 = tool[2,3]

            tool_g = np.array(origin,dtype=float).reshape(3,1)  # make numpy vector
            tool_g = np.insert(tool_g, 3, 1, axis=0)            # append 1 at (4,1)

            tool_0 = np.linalg.inv(self.Frames[0].T_local()) @ tool_g   # transform from {g} to {0} p_T^0 = T_g^0 * p_T^g

            x_tool_0 = tool_0[0]
            y_tool_0 = tool_0[1]
            z_tool_0 = tool_0[2]

            a1 = np.arctan2(y_tool_0,x_tool_0)
            l = np.sqrt(x_tool_0**2+y_tool_0**2)
            h = np.sqrt(l**2-z_tool_4**2)
            a2 = np.arctan2(z_tool_4,h)

            q1 = a1+a2

            T_T0 = np.array([[(np.cos(gamma)*np.cos(q1)).item(), (-np.cos(q1)*np.sin(gamma)).item(),  np.sin(q1).item(), x_tool_0.item()],
            [(np.cos(gamma)*np.sin(q1)).item(), (-np.sin(gamma)*np.sin(q1)).item(), (-np.cos(q1)).item(),y_tool_0.item()],
            [np.sin(gamma).item(),np.cos(gamma).item(),0,z_tool_0.item()],
            [0,0,0,1]])


            T_40 = T_T0 @ np.linalg.inv(tool)
            x_4z = T_40[2,0]
            tip_0 = T_40[0:3,3]


            gamma = np.arctan2(x_4z,np.sqrt(1-x_4z**2))




            # r21 = tool[1,0]
            # epsilon = np.arctan2(r21,np.sqrt(1-r21**2))
            # gamma_star = gamma-epsilon
            # sin_a2 = z_tool_4/(np.sqrt(x_tool_0**2+y_tool_0**2))
            # a2 = np.arctan2(sin_a2,np.sqrt(1-sin_a2**2))
            # a1 = np.arctan2(y_tool_0,x_tool_0)
            # q1 = a1+a2

            # tip_x_0 = x_tool_0 + y_tool_4*np.sin(gamma_star)*np.cos(q1) - x_tool_4*np.cos(gamma_star)*np.cos(q1)
            # tip_y_0 = y_tool_0 - z_tool_4*np.sin(q1) - x_tool_4*np.sin(q1)
            # tip_z_0 = z_tool_0 - y_tool_4*np.cos(gamma_star) - x_tool_4*np.sin(gamma_star)

            # tip_0 = np.array([tip_x_0, tip_y_0, tip_z_0],dtype=float).reshape(3,1)
            # gamma = gamma_star

        else:
            tip_g = np.array(origin,dtype=float).reshape(3,1)  # make numpy vector
            tip_g = np.insert(tip_g, 3, 1, axis=0)            # append 1 at (4,1)

            tip_0 = np.linalg.inv(self.Frames[0].T_local()) @ tip_g   # transform from {g} to {0}
        
        
        ox = tip_0[0]
        oy = tip_0[1]
        oz = tip_0[2]



        if tool is None:
            q1 = np.arctan2(oy,ox)

        w_x = ox-self.Frames[4].a*np.cos(gamma)*np.cos(q1)
        w_y = oy-self.Frames[4].a*np.cos(gamma)*np.sin(q1)
        w_z = oz-self.Frames[4].a*np.sin(gamma)
        


        r = np.sqrt(w_x**2 +w_y**2)
        s = w_z-self.Frames[1].d

        assert (np.sqrt(r**2+s**2)<self.Frames[3].a+self.Frames[2].a), "Point outside reachable workspace"

        cos3 = (r**2+s**2-self.Frames[2].a**2-self.Frames[3].a**2)/(2*self.Frames[2].a*self.Frames[3].a)
        if(elbow == "up"):
            q3 = np.arctan2(-np.sqrt(1-cos3**2),cos3)
            q2 = np.arctan2(-r,s)+np.arctan2(np.sqrt(1-cos3**2)*self.Frames[3].a,self.Frames[2].a+self.Frames[3].a*cos3)
        else:
            q3 = np.arctan2(+np.sqrt(1-cos3**2),cos3)
            q2 = np.arctan2(-r,s)-np.arctan2(np.sqrt(1-cos3**2)*self.Frames[3].a,self.Frames[2].a+self.Frames[3].a*cos3)
        
        q4 = gamma-np.pi/2-q2-q3

        return [arr.item() for arr in [q1, q2, q3, q4]]




    # Create "functions" for setting and moving motors:
    def enable_torque(self, motor_id):
        if self.__has_hardware:
            # enable torque for a motor
            result, error = self.__packetHandler.write1ByteTxRx(self.__portHandler, motor_id, self.__ADDR_MX_TORQUE_ENABLE, self.__TORQUE_ENABLE)
            if result != dxl.COMM_SUCCESS:
                print(f"Failed to enable torque for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")

    def disable_torque(self, motor_id):
         if self.__has_hardware:
            # Disable torque for a specific motor
            result, error = self.__packetHandler.write1ByteTxRx(self.__portHandler, motor_id, self.__ADDR_MX_TORQUE_ENABLE, self.__TORQUE_DISABLE)
            if result != dxl.COMM_SUCCESS:
                print(f"Failed to disable torque for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")

    def deg_to_rot(self,deg):
        #deg_to_rot function for the robot arm class.
        #   Converts degree to units per rotation of motors
        #
        #Inputs:
        #   deg : value [deg]
        #Outputs:
        #   rot : value in units per rotation of motor
        return deg*1/0.293+512

    def rad_to_rot(self,rad):
        #rad_to_rot function for the robot arm class.
        #   Converts radians to units per rotation of motors
        #
        #Inputs:
        #   rad : value [rad]
        #Outputs:
        #   rot : value in units per rotation of motor
        return self.deg_to_rot(np.rad2deg(rad))
    
    def radps_to_rot(self,radps):
        #radps_to_rot function for the robot arm class.
        #   Converts radians/s to units per rotation of motors
        #
        #Inputs:
        #   rad : value [rad/s]
        #Outputs:
        #   rot : value in units per rotation of motor
        return radps*86.03
    
    def rot_to_radps(self,rot):
        #radps_to_rot function for the robot arm class.
        #   Converts radians/s to units per rotation of motors
        #
        #Inputs:
        #   rot : value [rot]
        #Outputs:
        #   radps : [rad/s]
        return rot/86.03

    def rot_to_deg(self,rot):
        #rot_to_deg function for the robot arm class.
        #   Convers units per rotation of motors to degree
        #
        #Inputs:
        #   rot : value in units per rotation of motor
        #Outputs:
        #   deg : value [deg]
        return (rot-512)*0.293

    def rot_to_rad(self,rot):
        #rot_to_rad function for the robot arm class.
        #   Convers units per rotation of motors to radians
        #
        #Inputs:
        #   rot : value in units per rotation of motor
        #Outputs:
        #   rad : value [rad]
        return np.deg2rad(self.rot_to_deg(rot))
    
    def set_joint_angle(self, joint, position):
        
        self.__SV_joint_angles[joint-1] = position

        if self.__has_hardware:
            # Set the target position for a motor
            result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler, self.__DXL_IDS[joint-1], self.__ADDR_MX_GOAL_POSITION, self.rad_to_rot(position))
            if result != dxl.COMM_SUCCESS:
                print(f"Failed to set goal position for joint {joint}: {self.__packetHandler.getTxRxResult(result)}")

    def get_joint_angle(self, joint=None):
        if self.__has_hardware:
            for i, motor_id in enumerate(self.__DXL_IDS):                 
                    # Attempt to read the present position for the specified motor
                    
                    angle, result, error = self.__packetHandler.read2ByteTxRx(self.__portHandler, motor_id, self.__ADDR_MX_PRESENT_POSITION)
                    with self.__lock:
                        self.__PV_joint_angles[i] = self.rot_to_rad(angle)

                    if result != dxl.COMM_SUCCESS:
                        print(f"Failed to read position for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")
                        return None
                    elif error != 0:
                        print(f"Error occurred while reading position for motor {motor_id}: {self.__packetHandler.getRxPacketError(error)}")
                        return None

        if joint == None:
            with self.__lock:
                return self.__PV_joint_angles
        else:
            assert (joint >= 1 and joint <= 4), f"Joint {joint} does not exist"
            with self.__lock:
                return self.__PV_joint_angles[joint-1]


    def move_to_angles(self, goal_positions):
        # Move each motor to the target position specified in goal_positions
        for motor_id, goal_pos in zip(self.__DXL_IDS, goal_positions):
            self.set_joint_angle(motor_id, goal_pos)

        # Wait until all motors reach their goal positions
        for motor_id, goal_pos in zip(self.__DXL_IDS, goal_positions):
            while True:
                current_position = self.get_joint_angle(motor_id)
                if current_position is None:
                    break  # Exit if we failed to read the position
                if abs(goal_pos - current_position) < self.__DXL_MOVING_STATUS_THRESHOLD:
                    break  # Movement complete for this motor

    #TODO: Fix this function

    # def set_torque(self, enable=True):
    #     for motor_id in self.__DXL_IDS:
    #         result, error = self.__packetHandler.write1ByteTxRx(self)
           

    def set_speed(self, speeds):
        #set_speed function for the robot arm class.
        #   Sets individual motor speeds in rad/s
        #
        #Inputs:
        #   speeds : a vector representing motor speeds for each motor
        #Outputs:
        #   None
       
        self.__motor_speeds = speeds

        for motor_id, speed in zip(self.__DXL_IDS, speeds):
            assert (speed > 0.0117 and speed <= 11.9),"Movement speed out of range, enter value between ]0,1]"
            if self.__has_hardware:
                result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler,motor_id, self.__ADDR_MX_MOVING_SPEED, int(self.radps_to_rot(speed)))
                if result != dxl.COMM_SUCCESS:
                    print(f"Failed to set speed for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")
        
    def _move_joints(self):
        gain = 10
        Ts = 0.01
        last = time.time()
        while self.__running:
            now = time.time()
            dt = now-last
            last = now

            if dt > Ts:
                with self.__lock:
                    SV = self.__SV_joint_angles
                    PV = self.__PV_joint_angles
                    speeds = self.__motor_speeds

                # print(f"{dt:.3f}, ",end='')

                for i in range(0,4):
                    # print(f"SV {i+1}: {SV[i]:.3f}, PV {i+1}: {PV[i]:.3f}, ",end='')
                    error = SV[i]-PV[i]
                    # print(f"error {i+1}: {error:.3f}, ",end='')
                    dq = error*gain*Ts
                    
                    if (abs(dq) > speeds[i]*Ts):
                        dq = speeds[i]*Ts*np.sign(dq)
                                        
                    # if abs(dq) <= np.deg2rad(1.0):
                    #     dq = 0.0
                    # print(f"dq {i+1}: {dq:.3f}, ",end='')
                    
                    PV[i] += dq
                
                # print("")

                with self.__lock:
                    self.__PV_joint_angles = PV

            time.sleep(0.01)
    
    def get_cached_jointAngles(self):
        with self.__lock:
            return self.__PV_joint_angles


    def close(self):
        self.twin.close()

        self.__running = False
        self.__thread.join()

        # Disable torque and close port before exiting
        if self.__has_hardware:
            for motor_id in self.__DXL_IDS:
                self.disable_torque(motor_id)
            self.__portHandler.closePort()


            
class Frame:
    def __init__(self, DH_params=None, T=None):
        if (DH_params is not None) and (T is not None):
            raise ValueError("Invalid argument, only accepts either DH or Transformation Matrix")

        self.T = T  # Initialize T to None or the provided transformation matrix
        self.is_Link = False

        if DH_params is not None:
            self.is_Link = True
            self.revolute = False
            self.theta = DH_params['theta']  # joint angle is offset by this value (relevant for frame 2)
            self.d = DH_params['d']
            self.a = DH_params['a']
            self.alpha = DH_params['alpha']

            if DH_params['type'] == "revolute":
                self.revolute = True

        elif T is not None:
            # If T is provided, we can set it directly
            self.T = T

    def T_local(self, q=None):
        if self.T is not None:
            return self.T  # Return the provided transformation matrix if it exists

        if self.revolute:
            return np.array([[np.cos(q + self.theta), -np.sin(q + self.theta) * np.cos(self.alpha), np.sin(q + self.theta) * np.sin(self.alpha), self.a * np.cos(q + self.theta)],
                             [np.sin(q + self.theta), np.cos(q + self.theta) * np.cos(self.alpha), -np.cos(q + self.theta) * np.sin(self.alpha), self.a * np.sin(q + self.theta)],
                             [0, np.sin(self.alpha), np.cos(self.alpha), self.d],
                             [0, 0, 0, 1]])
        else:
            return np.array([[np.cos(self.theta), -np.sin(self.theta) * np.cos(self.alpha), np.sin(self.theta) * np.sin(self.alpha), self.a * np.cos(self.theta)],
                             [np.sin(self.theta), np.cos(self.theta) * np.cos(self.alpha), -np.cos(self.theta) * np.sin(self.alpha), self.a * np.sin(self.theta)],
                             [0, np.sin(self.alpha), np.cos(self.alpha), q],
                             [0, 0, 0, 1]])
        
    



# how to include kinematics

# try to copy the move_j function, use degree conversion like in myrobot
# if that doesnt work, try to use chatgpt's method of mapping degrees
# copy the function that checks if angle is within motor limits 

