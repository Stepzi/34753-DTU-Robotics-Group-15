import numpy as np
from digitalTwin import DigitalTwin 
from patterns import Patterns 
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
        self.__ADDR_MX_JOINT_LIMIT_CW = 6
        self.__ADDR_MX_JOINT_LIMIT_CCW = 8
        self.__ADDR_MX_PUNCH = 48
        self.__TORQUE_ENABLE = 1
        self.__TORQUE_DISABLE = 0
        self.__DXL_MOVING_STATUS_THRESHOLD = 0.05  # [rad] Threshold for detecting movement completion
        self.__DXL_MIN_SPEED = 0.2 # 0.0117
        self.__DXL_IDS = [1, 152, 3, 4]  # Motor IDs
        if end_effector == "angled":
            self.joint_limits = [[self.rot_to_rad(0),self.rot_to_rad(1023)],
            [self.rot_to_rad(93),self.rot_to_rad(927)],
            [self.rot_to_rad(0),self.rot_to_rad(1023)],
            [self.rot_to_rad(500),self.rot_to_rad(1023)]]
        elif end_effector == "straight":
            self.joint_limits = [[self.rot_to_rad(0),self.rot_to_rad(1023)],
            [self.rot_to_rad(93),self.rot_to_rad(927)],
            [self.rot_to_rad(0),self.rot_to_rad(1023)],
            [self.rot_to_rad(178),self.rot_to_rad(853)]]

        # Digital Twin
        self.twin = DigitalTwin(self)

        # Patterns
        self.patterns = Patterns(self)

        # Setup Multithreading:
        self.__lock = threading.Lock()
        self.__running = True
        self.__thread = threading.Thread(target=self._move_joints)
        self.__thread.start()
        

        # Robot Configuration
        self.__SV_joint_angles = [0.0,-np.pi/4,np.pi/2,-np.pi/4] # [rad]
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
            
        self.set_jointLimits(self.joint_limits)
        self.set_speed(self.__motor_speeds)
        self.move_to_angles(self.__SV_joint_angles)

    def assemble_robot(self):
        frames = [] # List of frame Objects, ORDER is important, first {0} in global then all links, then the rest
        # Base Frame
        frames.append(Frame(T=np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 1, 0.045],
                                        [0, 0, 0, 1]])))
        # Link 1          
        frames.append(Frame(DH_params={'theta': 0,      'd':0.046,   'a': 0,     'alpha': np.pi/2,   'type': "revolute"}))
        # Link 2
        frames.append(Frame(DH_params={'theta': np.pi/2,'d':0,      'a': 0.093, 'alpha': 0,         'type': "revolute"}))
        # Link 3
        frames.append(Frame(DH_params={'theta': 0,      'd':0,      'a': 0.093, 'alpha': 0,         'type': "revolute"}))
        # Link 4
        if self.__end_effector == "straight":
            frames.append(Frame(DH_params={'theta': 0,      'd':0,      'a': 0.065,  'alpha': 0,         'type': "revolute"}))
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
    def fwd_kin(self,q=None,frame_no=4,return_details=False):
        if q is None:
            joint_angles = self.get_cached_jointAngles()
        else:
            joint_angles = q

        T_global = [self.Frames[0].T_local(0)]  # Base Frame
        for i, frame in enumerate(self.Frames[1:]):
            if frame.is_Link:                
                T_global.append(T_global[-1] @ frame.T_local(joint_angles[i]))
            else:
                T_global.append(T_global[-1] @ frame.T_local())
        
        
        
        
        if return_details:
            gamma = np.arccos(T_global[frame_no][0,0]/np.cos(joint_angles[0]))*np.sign(T_global[frame_no][2,0])
            gamma = gamma.item()
            origin = [T_global[frame_no][0,3].item(),T_global[frame_no][1,3].item(),T_global[frame_no][2,3].item()]
            return T_global, origin, gamma
        else:
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

            gamma = np.arccos(T_40[0,0]/np.cos(q1))*np.sign(T_40[2,0])
            gamma = gamma.item()
            # gamma = np.arctan2(x_4z,np.sqrt(1-x_4z**2))

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
        s = (w_z-self.Frames[1].d)

        assert np.sqrt(r**2+s**2) < (self.Frames[3].a+self.Frames[2].a) , "Point outside reachable workspace"

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
        assert (order in [1,3,5]), f"Order {order} is not supported"
        assert (len(A) == (order+1)/2 and len(B) == (order+1)/2), "Missing boundary conditions for selected order"
        
        boundary_cond = A.copy()        
        boundary_cond.extend(B)

        boundary_cond = np.array(boundary_cond).reshape(order+1,1)

        if order == 1:
            T = np.array([[1,tA],
                        [1,tB]])
        elif order == 3:
            T = np.array([[1,tA,tA**2,tA**3],
                        [0,1,2*tA,3*tA**2],
                        [1,tB,tB**2,tB**3],
                        [0,1,2*tB,3*tB**2]])
        elif order == 5:
            T = np.array([[1,tA,tA**2,tA**3,tA**4,tA**5],
                        [0,1,2*tA,3*tA**2,4*tA**3,5*tA**4],
                        [0,0,2,6*tA,12*tA**2,20*tA**3],
                        [1,tB,tB**2,tB**3,tB**4,tB**5],
                        [0,1,2*tB,3*tB**2,4*tB**3,5*tB**4],
                        [0,0,2,6*tB,12*tB**2,20*tB**3]])
        

        return np.linalg.inv(T) @ boundary_cond

    def task_polyTraj(self,A: dict,B: dict,T,order):
        """Creates a straight-line trajectory in the task space between configurations `A` and `B`       
        
        The configuration Dicts contain the following key-value pairs:
        {'gamma': gamma [rad],'origin': [x,y,z] in {g}, 'elbow':"up"/"down",
          'v': scalar [m/s], 'a': scalar [m/^2]}

        @param A: dict of configuration at time tA
        @param B: dict of configuration at time tB
        @param T: Trajectory Duration
        @param order: order of polynomial
        @returns: Trajectory Object in joint space
        """

        if order == 1:
            s = self.poly_interpol(A=[0],B=[1],tA=0,tB=T,order=order)
        elif order == 3:
            s = self.poly_interpol(A=[0,A['v']],B=[1,B['v']],tA=0,tB=T,order=order)
        elif order == 5:
            s = self.poly_interpol(A=[0,A['v'],A['a']],B=[1,B['v'],B['a']],tA=0,tB=T,order=order)

        cA = A['origin'].copy()
        cA.append(A['gamma'])
        cA = np.array(cA).reshape(4,1)

        cB = B['origin'].copy()
        cB.append(B['gamma'])
        cB = np.array(cB).reshape(4,1)

        dc = cB-cA

        # Calculate Polynomial for each dimension x,y,z,gamma
        coeffs = []
        if order == 1:
            for i in range(4):
                coeffs.append(np.array([cA[i]+s[0]*dc[i],s[1]*dc[i]]).reshape(order+1,1))
        elif order == 3:
            for i in range(4):
                coeffs.append(np.array([cA[i]+s[0]*dc[i],s[1]*dc[i],s[2]*dc[i],s[3]*dc[i]]).reshape(order+1,1))
        elif order == 5:
            for i in range(4):
                coeffs.append(np.array([cA[i]+s[0]*dc[i],s[1]*dc[i],s[2]*dc[i],s[3]*dc[i],s[4]*dc[i],s[5]*dc[i]]).reshape(order+1,1))
        # Return Trajectory object in joint space
        return Trajectory(0,T,coeffs=coeffs,space="task")

    def task_genericTraj(self,funcs: list,A: dict,B: dict,T,order):
        """Creates a circular trajectory in the task space between configurations `A` and `B`       
        
        TODO

        @param A: dict of configuration at time tA
        @param B: dict of configuration at time tB
        @param T: Trajectory Duration
        @param order: order of polynomial
        @returns: Trajectory Object in joint space
        """

        if order == 1:
            s = self.poly_interpol(A=[0],B=[1],tA=0,tB=T,order=order)
        elif order == 3:
            s = self.poly_interpol(A=[0,A['v']],B=[1,B['v']],tA=0,tB=T,order=order)
        elif order == 5:
            s = self.poly_interpol(A=[0,A['v'],A['a']],B=[1,B['v'],B['a']],tA=0,tB=T,order=order)

        
        s = Trajectory(0,T,coeffs=[s],space="traj")

        
        functions = []
        for func in funcs:
            if func['type'] == "const":
                def genFun(t, a0=func['a0']):  # Capture a0 as a default argument
                    return a0
                functions.append(genFun)

            elif func['type'] == "linear":
                def genFun(t, a0=func['a0'], a1=func['a1']):  # Capture a0 and a1
                    return a0 + a1 * s.eval(t)[0]
                functions.append(genFun)

            elif func['type'] == "sin":
                def genFun(t, offset=func['offset'], amplitude=func['amplitude'], freq=func['freq'], phase=func['phase']):  # Capture all parameters
                    return offset + amplitude * np.sin(freq * s.eval(t)[0] + phase)
                functions.append(genFun)

            elif func['type'] == "cos":
                def genFun(t, offset=func['offset'], amplitude=func['amplitude'], freq=func['freq'], phase=func['phase']):  # Capture all parameters
                    return offset + amplitude * np.cos(freq * s.eval(t)[0] + phase)
                functions.append(genFun)

            else:
                raise ValueError(f"unsupported function: '{func['type']}'")
        
        
        return Trajectory(0,T,funcs=functions,space="task")

        
        
        # cA = A['origin'].copy()
        # cA.append(A['gamma'])
        # cA = np.array(cA).reshape(4,1)

        # cB = B['origin'].copy()
        # cB.append(B['gamma'])
        # cB = np.array(cB).reshape(4,1)

        # dc = cB-cA

        #  # Calculate Polynomial for each joint
        # coeffs = []
        # if order == 1:
        #     for i in range(4):
        #         coeffs.append(np.array([cA[i]+s[0]*dc[i],s[1]*dc[i]]).reshape(order+1,1))
        # elif order == 3:
        #     for i in range(4):
        #         coeffs.append(np.array([cA[i]+s[0]*dc[i],s[1]*dc[i],s[2]*dc[i],s[3]*dc[i]]).reshape(order+1,1))
        # elif order == 5:
        #     for i in range(4):
        #         coeffs.append(np.array([cA[i]+s[0]*dc[i],s[1]*dc[i],s[2]*dc[i],s[3]*dc[i],s[4]*dc[i],s[5]*dc[i]]).reshape(order+1,1))
        # # Return Trajectory object in joint space
        # return Trajectory(coeffs,0,T,space="task")

    def joint_polyTraj(self,frame_no,A: dict,B: dict,tA,tB,order):
        """Creates a trajectory for the frame `frame_no` in the joint space between configurations `A` and `B`       
        
        The configuration Dicts contain the following key-value pairs:
        {'gamma': gamma [rad],'origin': [x,y,z] in {g}, 'elbow':"up"/"down",
          'v': [v_x, v_y ,v_z], 'gamma_d': [rad/s]}

        @param frame_no: frame number > 3  
        @param A: dict of configuration at time tA
        @param B: dict of configuration at time tB
        @param tA: time tA
        @param tB: time tB
        @param order: order of polynomial
        @returns: Trajectory Object in joint space
        """
        assert frame_no > 3, "Cant compute inverse Kinematics for non-endeffecor Links"
        
        tool = None
        if frame_no > 4:
            tool = self.Frames[frame_no].T_local()

        qA = self.inv_kin(A['gamma'],A['origin'],elbow=A['elbow'],tool=tool)
        qB = self.inv_kin(B['gamma'],B['origin'],elbow=B['elbow'],tool=tool)

        if order >= 3:
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
        if order == 1:
            for i in range(4):
                coeffs.append(self.poly_interpol(A=[qA[i]],B=[qB[i]],tA=tA,tB=tB,order=order))
        elif order == 3:
            for i in range(4):
                coeffs.append(self.poly_interpol(A=[qA[i],qA_d[i].item()],B=[qB[i],qB_d[i].item()],tA=tA,tB=tB,order=order))
        elif order == 5:
            for i in range(4):
                coeffs.append(self.poly_interpol(A=[qA[i],qA_d[i].item(),qA_dd[i]],B=[qB[i],qB_d[i].item(),qB_dd[i]],tA=tA,tB=tB,order=order))

        
        # Return Trajectory object in joint space
        return Trajectory(tA,tB,coeffs=coeffs)
    
    def follow_traj(self,trajs: list, Ts=0.1, tol=0.05, frame_no=4, elbow="up", DONE=None):
        """Arm follows `trajs`   

        Before the arm follows the desired trajectory,
        the current arm configuration and inital trajectory arm
        configuration must be within `tolerance` [rad].
        If the provided trajectory is in task space, the frame number and elbow configuration can be provided.

        @param trajs: list of trajectory objects
        @param Ts: Sampling time of movement
        @param tol: tolerance at inital conditions
        """

        tool = None
        for traj in trajs:
            assert traj.space in["joint","task"], "Trajectory must be in joint or task space"
            if traj.space == "task" and frame_no > 4:
                tool = self.Frames[frame_no].T_local()

        # Move to start of first trajectory
        # TODO: check continuitiy for all other traj in trajs
        for i, q_i in enumerate(self.get_cached_jointAngles()):
            if abs(trajs[0].s_0[i] - q_i) > tol:
                print(f"Non-Continous Trajectory, linear movement to: {trajs[0].s_0}")
                self.set_speed([0.5]*4)
                if trajs[0].space == "task":
                    s_0 = self.inv_kin(gamma=trajs[0].s_0[3],origin=trajs[0].s_0[0:3],elbow=elbow,tool=tool)
                elif trajs[0].space == "joint":
                    s_0 = trajs[0].s_0
                thread, DONE_init = self.run_in_thread(self.move_to_angles,s_0)
                DONE_init.wait()
                thread.join()
                break          
        
        for traj in trajs:
            # print("next traj")
            # input()

            start_time = time.time()
            last_time = time.time()-Ts
            while True:
                current_time = time.time()
                dt = current_time - last_time        
                if (dt >= Ts):
                    t =  (current_time - start_time)
                    # print(f"{t:.3f}")   
                    if t+Ts > traj.tB:                    
                        if traj.space == "task":
                            X = traj.eval(traj.tB) # next
                            X_1 = traj.eval(t)     # current
                            # print(f"X: {X}")
                            # print(f"X_1: {X_1}")             
                            q = self.inv_kin(gamma=X[3],origin=X[0:3],elbow=elbow,tool=tool)
                            q_1 = self.inv_kin(gamma=X_1[3],origin=X_1[0:3],elbow=elbow,tool=tool)
                            # print(f"q: {q}")
                            # print(f"q_1: {q_1}")  
                            # print(f"dt: {dt}")
                            # print(f"q_diff: {[(a_i - b_i) for a_i, b_i in zip(q,q_1)]}")            
                            
                            q_d = [(a_i - b_i)/(traj.tB-t) for a_i, b_i in zip(q,q_1)]
                            
                            
                        elif traj.space == "joint": 
                            q = traj.eval(traj.tB)
                            q_d = traj.eval(traj.tB,diff = 1,Ts= traj.tB - t)
                        
                        # print(f"q_d: {q_d}")
                        self.set_speed(q_d)
                        self.move_to_angles(q,blocking=True)
                        # print(f"Total: {time.time() - start_time:.3f}")
                        
                        break

                    else:
                        if traj.space == "task":
                            X = traj.eval(t+Ts) # next
                            X_1 = traj.eval(t)     # current   
                            # print(f"X: {X}")
                            # print(f"X_1: {X_1}")             
                            q = self.inv_kin(gamma=X[3],origin=X[0:3],elbow=elbow,tool=tool)
                            q_1 = self.inv_kin(gamma=X_1[3],origin=X_1[0:3],elbow=elbow,tool=tool)
                            # print(f"q: {q}")
                            # print(f"q_1: {q_1}")  
                            # print(f"dt: {dt}")
                            # print(f"q_diff: {[(a_i - b_i) for a_i, b_i in zip(q,q_1)]}")      
                            q_d = [(a_i - b_i)/Ts for a_i, b_i in zip(q,q_1)]

                        elif traj.space == "joint": 
                            q = traj.eval(t+Ts)
                            q_d = traj.eval(t+Ts,diff = 1,Ts= dt) # TODO: dt instead of TS?
                        
                        # print(f"q_d: {q_d}")
                        self.set_speed(q_d)  
                        self.move_to_angles(q,blocking=False)

                    last_time = current_time
                    
                time.sleep(0.005)

        # Signal that the function has finished
        if DONE is not None:
            DONE.set()

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


    def move_to_angles(self, goal_positions,blocking=True, DONE = None):
        # Move each motor to the target position specified in goal_positions
        for joint, goal_pos in zip(range(1,5), goal_positions):
            self.set_joint_angle(joint, goal_pos)

        if blocking:
            # Wait until all motors reach their goal positions
            # print("Moving joints:")
            # q_diff = [float(a_i - b_i) for a_i, b_i in zip(goal_positions,self.get_cached_jointAngles())]
            # ts = [(float(a_i / (b_i)) if b_i > 0 else 0) for a_i, b_i in zip(q_diff,self.__motor_speeds)]
            # print(f"error: {q_diff}")
            # print(f"time: {ts}")
            # print(f"Should take max: {max(ts)}")
                    
            start = time.time()
            for joint, goal_pos in zip(range(1,5), goal_positions):
                
                while True:
                    # print(f"SV: {self.__SV_joint_angles}")
                    # print(f"PV: {[float(i) for i in self.__PV_joint_angles]}")
                    # print(f"VE: {[float(i) for i in self.__motor_speeds]}")
                    # if time.time()-start > 10:
                    #     raise ValueError("Timout: Movement took to long")
                    current_position = self.get_joint_angle(joint)
                    if current_position is None:
                        break  # Exit if we failed to read the position
                    # print(f"ID: {joint}, error: {abs(goal_pos - current_position)}, VE: {self.__motor_speeds[joint-1]}")
                    if abs(current_position - self.joint_limits[joint-1][0]) < self.__DXL_MOVING_STATUS_THRESHOLD:
                        print(f"Position of joint {joint} at CW joint Limit")
                        break
                    if abs(current_position - self.joint_limits[joint-1][1]) < self.__DXL_MOVING_STATUS_THRESHOLD:
                        print(f"Position of joint {joint} at CCW joint Limit")
                        break
                    if abs(goal_pos - current_position) < self.__DXL_MOVING_STATUS_THRESHOLD:
                        break  # Movement complete for this motor

                    time.sleep(0.05)
            # print(f"took: {time.time()-start}")

        if DONE is not None:
            DONE.set()

    def set_speed(self, speeds):
        #set_speed function for the robot arm class.
        #   Sets individual motor speeds in rad/s
        #
        #Inputs:
        #   speeds : a vector representing motor speeds for each motor
        #Outputs:
        #   None
       
        self.__motor_speeds = speeds
        speeds = [abs(item) for item in speeds]

        for motor_id, speed in zip(self.__DXL_IDS, speeds):
            # assert (speed > 0.0117 and speed <= 11.9),"Movement speed out of range, enter value between ]0,1]"
            if speed < self.__DXL_MIN_SPEED:
                speed = self.__DXL_MIN_SPEED
            if speed > 11.9:
                speed = 11.9
            if self.__has_hardware:
                result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler,motor_id, self.__ADDR_MX_MOVING_SPEED, int(self.radps_to_rot(speed)))
                if result != dxl.COMM_SUCCESS:
                    print(f"Failed to set speed for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")
    
    def set_jointLimits(self, limits):
              
        self.joint_limits = limits

        for motor_id, limit in zip(self.__DXL_IDS, limits):
            # assert (speed > 0.0117 and speed <= 11.9),"Movement speed out of range, enter value between ]0,1]"
            
            if self.__has_hardware:
                result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler,motor_id, self.__ADDR_MX_JOINT_LIMIT_CW, int(self.rad_to_rot(limit[0])))
                if result != dxl.COMM_SUCCESS:
                    print(f"Failed to set limit for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")
                
                result, error = self.__packetHandler.write2ByteTxRx(self.__portHandler,motor_id, self.__ADDR_MX_JOINT_LIMIT_CCW, int(self.rad_to_rot(limit[1])))
                if result != dxl.COMM_SUCCESS:
                    print(f"Failed to set limit for motor {motor_id}: {self.__packetHandler.getTxRxResult(result)}")

    def run_in_thread(self, target_function, *args, **kwargs):
        """Runs a target function in a separate thread."""
        DONE = threading.Event()
        kwargs['DONE'] = DONE
        thread = threading.Thread(target=target_function, args=args, kwargs=kwargs)
        thread.start()
        return thread, DONE  # Optionally return the thread object
    
    def _move_joints(self):
        gain = 100
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
                    dq = error*gain*dt
                    
                    if (abs(dq) > speeds[i]*dt):
                        dq = abs(speeds[i])*dt*np.sign(dq)
                                        
                    # print(f"dq {i+1}: {dq:.4f}, ",end='')

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
    def __init__(self,tA,tB,coeffs=None,funcs = None, space= "joint"):
        assert coeffs is not None or funcs is not None, "Coeffs or Funcs needs to be defined"
        self.space = space
        self.tA = tA
        self.tB = tB

        if coeffs is not None:
            self.type = "poly"
            self.coeffs = coeffs
            self.dim = len(coeffs)

        if funcs is not None:
            self.type = "generic"
            self.funcs = funcs
            self.dim = len(funcs)


        self.s_0 = self.eval(tA)
        self.s_e = self.eval(tB)
        
        

    def eval(self,t: float,diff = 0,Ts = None):
        if self.type == "poly":
            return self._poly_eval(t,diff,Ts)
        elif self.type == "generic":
            return self._generic_eval(t)
        
    def _poly_eval(self,t,diff,Ts):         
        assert diff in [0, 1, 2], f"Derivative of order {diff} not supported"
        
        if Ts is not None:
            assert Ts != 0, "Sample time must not be Ts = 0"
            assert diff == 1, f"Discrete Implementation for order {diff} not supported"
            disc = True
        else:
            disc = False

        
        out = [0.0] * self.dim

        for i in range(self.dim):
            if self.coeffs[i].size == 2:
                if diff == 0:
                    out[i] = (np.array([[1, t]]) @ self.coeffs[i]).item()
                elif diff == 1:
                    if disc:
                        out[i] = ((np.array([[1, t]]) @ self.coeffs[i]).item() - (np.array([[1, t-Ts]]) @ self.coeffs[i]).item())/Ts
                    else:
                        out[i] = (np.array([[0, 1]]) @ self.coeffs[i]).item()
                elif diff == 2:
                    out[i] = 0.0          
            
            elif self.coeffs[i].size == 4:
                if diff == 0:
                    out[i] = (np.array([[1, t, t**2, t**3]]) @ self.coeffs[i]).item()
                elif diff == 1:
                    if disc:
                        out[i] = ((np.array([[1, t, (t)**2, (t)**3]]) @ self.coeffs[i]).item() - (np.array([[1, (t-Ts), (t-Ts)**2, (t-Ts)**3]]) @ self.coeffs[i]).item())/Ts
                    else:
                        out[i] = (np.array([[0, 1, 2*t, 3*t**2]]) @ self.coeffs[i]).item()
                elif diff == 2:
                    out[i] = (np.array([[0, 0, 2, 6*t]]) @ self.coeffs[i]).item()

            elif self.coeffs[i].size == 6:
                if diff == 0:
                    out[i] = (np.array([[1, t, t**2, t**3, t**4, t**5]]) @ self.coeffs[i]).item()
                elif diff == 1:
                    if disc:
                        out[i] = ((np.array([[1, (t), (t)**2, (t)**3, (t)**4, (t)**5]]) @ self.coeffs[i]).item() - (np.array([[1, (t-Ts), (t-Ts)**2, (t-Ts)**3, (t-Ts)**4, (t-Ts)**5]]) @ self.coeffs[i]).item())/Ts
                    else:
                        out[i] = (np.array([[0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4]]) @ self.coeffs[i]).item()
                elif diff == 2:
                    out[i] = (np.array([[0, 0, 2, 6*t, 12*t**2, 20*t**3]]) @ self.coeffs[i]).item()
            else:
                raise ValueError(f"Unsupported order {self.coeffs[i].len()-1}")
           
        return out
    
    def _generic_eval(self,t):
        out = [0.0] * self.dim

        for i in range(self.dim):
            out[i] = self.funcs[i](t)
           
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