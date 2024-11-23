import numpy as np
from digitalTwin import DigitalTwin 
import threading
import time
import matplotlib.pyplot as plt



# Add the Dynamixel SDK path to sys.path
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'DynamixelSDK/python/src')))

import dynamixel_sdk as dxl

class RobotArm():
    def __init__(self, device_name, end_effector="straight", baudrate=1000000, protocol_version=1.0):
        if end_effector not in ["straight", "angled"]:
            raise ValueError("end_effector must be either 'straight' or 'angled'")
        
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
        self.__DXL_MOVING_STATUS_THRESHOLD = 0.05  # [rad] Threshold for detecting movement completion
        self.__DXL_IDS = [1, 2, 3, 4]  # Motor IDs

        # Digital Twin
        self.twin = DigitalTwin(self)

        # Setup Multithreading:
        self.__lock = threading.Lock()
        self.__running = True
        self.__thread = threading.Thread(target=self._move_joints)
        self.__thread.start()
        

        # Robot Configuration
        self.__SV_joint_angles = [0.0,0.0,0.0,0.0] # [rad]
        with self.__lock:
            self.__PV_joint_angles = [0.0,0.0,0.0,0.0] # [rad]
        self.__motor_speeds = [1,1,1,1] # [rad/s] (0.0117,11.9)
        self.__end_effector = end_effector

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
            

        self.set_speed(self.__motor_speeds)
        self.move_to_angles(self.__SV_joint_angles)

    def assemble_robot(self):
        frames = [] # List of frame Objects, ORDER is important, first {0} in global then all links, then the rest
        # Base Frame
        frames.append(Frame(T=np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 1, 0.05],
                                        [0, 0, 0, 1]])))
        # Link 1          
        frames.append(Frame(DH_params={'theta': 0,      'd':0.04,   'a': 0,     'alpha': np.pi/2,   'type': "revolute"}))
        # Link 2
        frames.append(Frame(DH_params={'theta': np.pi/2,'d':0,      'a': 0.093, 'alpha': 0,         'type': "revolute"}))
        # Link 3
        frames.append(Frame(DH_params={'theta': 0,      'd':0,      'a': 0.093, 'alpha': 0,         'type': "revolute"}))
        # Link 4
        if self.__end_effector == "straight":
            frames.append(Frame(DH_params={'theta': 0,      'd':0,      'a': 0.05,  'alpha': 0,         'type': "revolute"}))
             # Tool
            frames.append(Frame(T=np.array([[0, 1, 0, 0.0],
                                            [-1, 0, 0, -0.05],
                                            [0, 0, 1, 0.0],
                                            [0, 0, 0, 1]])))
        elif self.__end_effector == "angled":
            frames.append(Frame(DH_params={'theta': -np.pi/2,      'd':0,      'a': 0.05,  'alpha': 0,         'type': "revolute"}))
            # Tool
            frames.append(Frame(T=np.array([[1, 0, 0, 0.0],
                                            [0, 1, 0, 0.015],
                                            [0, 0, 1, 0.0],
                                            [0, 0, 0, 1]])))

        return frames 

    # Kinematic Methods
    def fwd_kin(self,q=None):
        if q is None:
            with self.__lock:
                joint_angles = self.__PV_joint_angles
        else:
            joint_angles = q

        T_global = [self.Frames[0].T_local(0)]  # Base Frame
        for i, frame in enumerate(self.Frames[1:]):
            if frame.is_Link:                
                T_global.append(T_global[-1] @ frame.T_local(joint_angles[i]))
            else:
                T_global.append(T_global[-1] @ frame.T_local())

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
        
        if self.__end_effector == "straight":
            q4 = gamma-np.pi/2-q2-q3
        elif self.__end_effector == "angled":
            q4 = gamma-np.pi/2-q2-q3 + np.pi/2

        return [arr.item() for arr in [q1, q2, q3, q4]]

    def jacobian(self,frame_no: int,q=None):
        """Computes the jacobian for a frame
        
        @param frame_no: frame number   
        @param q: joint configuration, if omitted, current PV_angles
        @returns: Jacobian Matrix in base frame
        """

        T_g = self.fwd_kin(q)
        J = np.zeros((6,4))

        for i in range(1,5): #iterate through frame 1 -> frame number
                z_i_1 = T_g[i-1][0:3,2]
                o_n = T_g[frame_no][0:3,3]
                o_i_1 = T_g[i-1][0:3,3]

                if self.Frames[i].revolute:
                    
                    J[0:3,i-1] = np.cross(z_i_1,(o_n-o_i_1))
                    J[3:6,i-1] = z_i_1
                else:
                    J[0:3,i-1] = z_i_1
                    J[3:6,i-1] = 0

        return J

    
    def poly_interpol(self,A: list,B:list,tA,tB,order = 5):
        assert (order in [5]), f"Order {order} is not supported"
        assert (len(A) == (order+1)/2 and len(B) == (order+1)/2), "Missing boundary conditions for selected order"
        
        boundary_cond = A.copy()        
        boundary_cond.extend(B)

        boundary_cond = np.array(boundary_cond).reshape(order+1,1)

        T = np.array([[1,tA,tA**2,tA**3,tA**4,tA**5],
                      [0,1,2*tA,3*tA**2,4*tA**3,5*tA**4],
                      [0,0,2,6*tA,12*tA**2,20*tA**3],
                      [1,tB,tB**2,tB**3,tB**4,tB**5],
                      [0,1,2*tB,3*tB**2,4*tB**3,5*tB**4],
                      [0,0,2,6*tB,12*tB**2,20*tB**3]])
        

        return np.linalg.inv(T) @ boundary_cond

    def poly_traj(self,frame_no,A: dict,B: dict,tA,tB,order):
        """Smoothly interpolates between two configurations A and B       
        
        The configuration Dicts contain the following key-value pairs:
        {'gamma': gamma [rad],'origin': [x,y,z] in {g}, 'elbow':"up"/"down",
          'v': [v_x, v_y ,v_z], 'gamma_d': [rad/s]}

        @param frame_no: frame number > 3  
        @param A: dict of configuration at time tA
        @param B: dict of configuration at time tB
        @param tA: time tA
        @param tB: time tB
        @param order: order of polynomial
        @returns: Trajectory Object
        """
        assert frame_no > 3, "Cant compute inverse Kinematics for non-endeffecor Links"
        
        tool = None
        if frame_no > 4:
            tool = self.Frames[frame_no].T_local()

        qA = self.inv_kin(A['gamma'],A['origin'],elbow=A['elbow'],tool=tool)
        qB = self.inv_kin(B['gamma'],B['origin'],elbow=B['elbow'],tool=tool)


        # Assume velocity in {g} and gamma_d is given. 
        # Compute remaining:

        # normally: [x_d, y_d, z_d, w_x, w_y, w_z]' = J * q_d
        # to find inverse use pinv. But we dont know w_x, w_y, w_z but instead gamma_d
        # r_31 = sin(gamma) -> r_31_d = cos(gamma)*gamma_d
        # we want to find the matrix M that reduces the Jacobian to 4x4
        # we know (skew symmetric Matrix) R_d[0:3,2] = R_yx*w_x - R_xx*w_y = r_31_d = cos(gamma)*gamma_d
        # [x_d, y_d, z_d, r_31_d]' = M * [x_d, y_d, z_d, w_x, w_y, w_z]'
        # -> M = (see below)
        # [x_d, y_d, z_d, r_31_d]' = M * J * q_d
        # q_d = (M * J)^-1 * [x_d, y_d, z_d, r_31_d]'

        T_g = self.fwd_kin(qA)
        R_fg = T_g[frame_no][0:3,0:3]                  # Should it be {4} or {frame_no}?
        M = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, R_fg[1,0], -R_fg[0,0], 0]])
        Xi = np.array([A['v'][0], A['v'][1], A['v'][2],np.cos(A['gamma'])*A['gamma_d']]).reshape(4,1)
        J = self.jacobian(frame_no=frame_no,q=qA)
        qA_d = np.linalg.inv(M @ J) @ Xi
        qA_dd = [0,0,0,0]

        T_g = self.fwd_kin(qB)
        R_fg = T_g[frame_no][0:3,0:3]                  # Should it be {4} or {frame_no}?
        M = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, R_fg[1,0], -R_fg[0,0], 0]])
        Xi = np.array([B['v'][0], B['v'][1], B['v'][2],np.cos(B['gamma'])*B['gamma_d']]).reshape(4,1)
        J = self.jacobian(frame_no=frame_no,q=qB)
        qB_d = np.linalg.inv(M @ J) @ Xi
        qB_dd = [0,0,0,0]

        # Calculate Polynomial for each joint

        coeffs = []

        for i in range(4):
            coeffs.append(self.poly_interpol(A=[qA[i],qA_d[i].item(),qA_dd[i]],B=[qB[i],qB_d[i].item(),qB_dd[i]],tA=tA,tB=tB,order=order))

        
        # Return List of trajectory objects
        return Trajectory(coeffs,tA,tB)
    
    def follow_traj(self,traj,Ts=0.1,tol=0.05):
        """Arm follows `traj`   

        Before the arm follows the desired trajectory,
        the current arm configuration and inital trajectory arm
        configuration must be within `tolerance` [rad]

        @param traj: trajectory object
        @param Ts: Sampling time of movement
        @param tol: tolerance at inital conditions
        """
        outside_tol = False
        for i, q_i in enumerate(self.get_cached_jointAngles()):
            if abs(traj.q_0[i] - q_i) > tol:
                outside_tol = True
                print(f"Non-Continous Trajectory, linear movement to: {traj.q_0}")
                break

        self.set_speed([2.0]*4)
        self.move_to_angles(traj.q_0)

        start_time = time.time()
        last_time = 0
        # t = Ts
        while True:
            current_time = time.time()        
            if (current_time - last_time > Ts):
                t =  (current_time - start_time) + Ts
                # print(f"{t:.3f}")
                if t >= traj.tB:
                    # print(f"Total: {current_time-start_time:.3f}") 
                    break             
                
                try:
                    q = traj.eval(t)
                    q_d = traj.eval(t,diff = 1,Ts= Ts)
                except Exception as e: print(e)
                else:
                    self.set_speed(q_d)
                    self.move_to_angles(q,blocking=False)

                last_time = current_time
                # t += Ts
                
            self.twin.draw_arm()    
            time.sleep(0.005)

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
        return int(deg*1/0.293+512)

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


    def move_to_angles(self, goal_positions,blocking=True):
        # Move each motor to the target position specified in goal_positions
        for motor_id, goal_pos in zip(self.__DXL_IDS, goal_positions):
            self.set_joint_angle(motor_id, goal_pos)

        if blocking:
            # Wait until all motors reach their goal positions
            start = time.time()
            for motor_id, goal_pos in zip(self.__DXL_IDS, goal_positions):
                while True:
                    # if time.time()-start > 10:
                    #     raise ValueError("Timout: Movement took to long")
                    current_position = self.get_joint_angle(motor_id)
                    if current_position is None:
                        break  # Exit if we failed to read the position
                    if abs(goal_pos - current_position) < self.__DXL_MOVING_STATUS_THRESHOLD:
                        break  # Movement complete for this motor

    def set_speed(self, speeds):
        #set_speed function for the robot arm class.
        #   Sets individual motor speeds in rad/s
        #
        #Inputs:
        #   speeds : a vector representing motor speeds for each motor
        #Outputs:
        #   None
        speeds = [abs(item) for item in speeds]
       
        self.__motor_speeds = speeds

        for motor_id, speed in zip(self.__DXL_IDS, speeds):
            # assert (speed > 0.0117 and speed <= 11.9),"Movement speed out of range, enter value between ]0,1]"
            if speed < 0.0117:
                speed = 0.0117
            if speed > 11.9:
                speed = 11.9
            if self.__has_hardware:
                result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler,motor_id, self.__ADDR_MX_MOVING_SPEED, int(self.radps_to_rot(speed)))
                if result != dxl.COMM_SUCCESS:
                    print(f"Failed to set speed for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")
        
    def _move_joints(self):
        gain = 30
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
        

class Trajectory():
    def __init__(self,coeffs,tA,tB):
        
        self.dim = len(coeffs)

        self.tA = tA
        self.tB = tB

        self.coeffs = coeffs

        self.q_0 = self.eval(tA)

        
        

    def eval(self,t: float,diff = 0,Ts = None):
        assert diff in [0, 1, 2], f"Derivative of order {diff} not supported"
        
        if Ts is not None:
            assert Ts > 0, "Sample time must be Ts > 0"
            assert diff == 1, f"Discrete Implementation for order {diff} not supported"
            disc = True
        else:
            disc = False

        
        out = [0.0] * self.dim

        if t < self.tA:
            t = self.tA
        if t > self.tB:
            t = self.tB

        for i in range(self.dim):
            if self.coeffs[i].size == 2:
                if diff == 0:
                    out[i] = (np.array([[1, t]]) @ self.coeffs[i]).item()
                elif diff == 1:
                    if disc:
                        dt = (Ts, -Ts) [t + Ts > self.tB]
                        out[i] = ((np.array([[1, t+dt]]) @ self.coeffs[i]).item() - (np.array([[1, t]]) @ self.coeffs[i]).item())/dt
                    else:
                        out[i] = (np.array([[0, 1]]) @ self.coeffs[i]).item()
                elif diff == 2:
                    out[i] = 0.0          
            
            elif self.coeffs[i].size == 4:
                if diff == 0:
                    out[i] = (np.array([[1, t, t**2, t**3]]) @ self.coeffs[i]).item()
                elif diff == 1:
                    if disc:
                        dt = (Ts, -Ts) [t + Ts > self.tB]
                        out[i] = ((np.array([[1, t+dt, (t+dt)**2, (t+dt)**3]]) @ self.coeffs[i]).item() - (np.array([[1, t, t**2, t**3]]) @ self.coeffs[i]).item())/dt
                    else:
                        out[i] = (np.array([[0, 1, 2*t, 3*t**2]]) @ self.coeffs[i]).item()
                elif diff == 2:
                    out[i] = (np.array([[0, 0, 2, 6*t]]) @ self.coeffs[i]).item()

            elif self.coeffs[i].size == 6:
                if diff == 0:
                    out[i] = (np.array([[1, t, t**2, t**3, t**4, t**5]]) @ self.coeffs[i]).item()
                elif diff == 1:
                    if disc:
                        dt = (Ts, -Ts) [t + Ts > self.tB]
                        out[i] = ((np.array([[1, (t+dt), (t+dt)**2, (t+dt)**3, (t+dt)**4, (t+dt)**5]]) @ self.coeffs[i]).item() - (np.array([[1, t, t**2, t**3, t**4, t**5]]) @ self.coeffs[i]).item())/dt
                    else:
                        out[i] = (np.array([[0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4]]) @ self.coeffs[i]).item()
                elif diff == 2:
                    out[i] = (np.array([[0, 0, 2, 6*t, 12*t**2, 20*t**3]]) @ self.coeffs[i]).item()
            else:
                raise ValueError(f"Unsupported order {self.coeffs[i].len()-1}")
            
        return out
    
    def plot(self):
        # Create a figure with subplots
        fig, axs = plt.subplots(self.dim, 1)
        t_values = np.linspace(self.tA, self.tB, 100)  # Generate 100 points between tA and tB

        for i in range(self.dim):
            # Evaluate the polynomial for the current dimension
            y_values = [sum(c * (t ** j) for j, c in enumerate(self.coeffs[i])) for t in t_values]
            axs[i].plot(t_values, y_values, label=f'q')
            axs[i].set_title(f'q_{i+1}')
            axs[i].set_xlabel('Time (t)')
            axs[i].set_ylabel('Value')
            axs[i].grid()
            # axs[i].legend()

        plt.tight_layout()
        plt.show()
