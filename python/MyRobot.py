import os
import numpy as np

import os
import sys
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,os.path.join(parentdir,"DynamixelSDK-3.7.31","python","src","dynamixel_sdk"))

from dynamixel_sdk import *

class MyRobot():
    def __init__(self):
   
    
        lib_name = 'dxl_x64_c'                     # Library name for Win10
        
        ADDR_MX_TORQUE_ENABLE       = 24           # Control table address for enabling torque mode
        ADDR_MX_GOAL_POSITION       = 30           # Control table address for reading goal position
        ADDR_MX_PRESENT_POSITION    = 36           # Control table address for reading current position
        PROTOCOL_VERSION            = 1.0          # See which protocol version is used in the Dynamixel
        BAUDRATE                    = 1000000      # Baudrate for Motors
        DEVICENAME                  = 'COM3'       # Check which port is being used on your controller
        # ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
        TORQUE_ENABLE               = 1            # Value for enabling the torque
        TORQUE_DISABLE              = 0            # Value for disabling the torque
        DXL_MOVING_STATUS_THRESHOLD = 10           # Dynamixel moving status threshold
        COMM_SUCCESS                = 0            # Communication Success result value
        COMM_TX_FAIL                = -1001        # Communication Tx Failed
        
        port_num=0                                 # Portnumber gets automatically handled by Porthandler

        motor_ids = [0, 1, 2, 3]                      # Motor IDs chronologically (see Dynamixel Wizard for more info)
        gripper_motor_id = 4                        # ID of gripper motor
        dh = [[0,-np.pi/2,	0.0955, 0],               # Denavit Hartenberg Parameters for Robot (a, alpha, d, theta)
            [0.116,	0,0,0],
            [0.096,0,0,0],
            [0.064,0,0,0]]
        forward_transform = np.zeros((4,4))             # Forward transformation Matrix        
        joint_angles = [0, 0, 0, 0]                   # Internal joint angles in degree
        joint_pos = np.zeros((4,4))                     # Internal joint positions calculated with each move_j        
        draw_robot_flag = 0                        # Flag for drawing robot configuration
        use_smooth_speed_flag = 0                  # Flag for using smooth speed 
        gripper_open_flag = 1                      # Flag for gripper status
        rbt = 0                                    # RigidBodyTree        
        joint_limits = [-130, 130, -180, 0, -100, 100, -100, 100] #Joint Limits in degree        
        ik = 0                                     # Inverse Kinematics Object
        ik_weights = [0.25, 0.25, 0.25, 1, 1, 1]        # Weights for inverse kinematics 
        joint_offsets = [171-5, 150+90, 150, 150]     # Joint offsets to send to motor
        joint_angle_error = [0, 0, 0, 0]              # Internal joint angle error between read out of joint angles and input joint angles
        init_status = 0                            # Initialization succesfull flag
        movement_history = []                      # List to record movement history
        motor_speed = 0                            # List for motor speed
        motor_torque = 0                           # List for motor torque
        pitch = 0                                  # Pitch Angle for motor 3
   

        #MyRobot Constructor for the MyRobot Class.
        #   Initializes robot, setting initial motor speeds to 10#, motor
        #   torque to 100# and sets initial joint angles to zero
        #
        #Inputs:
        #   None
        #Outputs:
        #   self : MyRobot Object
        try:
            if ~libisloaded(self.lib_name):
                loadlibrary(self.lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h')
            
            self.port_num = portHandler(self.DEVICENAME)
            packetHandler()
            if (openPort(self.port_num)):
                print('Succeeded to open the port!\n')
            else:
                print('Failed to open the port!\nReconnect Robot!\n')
                closePort(self.port_num)
                unloadlibrary(lib_name)

            if (setBaudRate(self.port_num, self.BAUDRATE)):
                print('Succeeded to change the baudrate!\n')
            else:
                unloadlibrary(self.lib_name)
                print('Failed to change the baudrate!\nReconnect Robot!\n')
                return

            self.set_speed([0.1,0.1,0.1,0.1],true)
            self.set_torque_limit([1,1,1,1])
            self.move_j(0,0,0,0)
            self.init_status = 1
        except Exception as e:
            print(Exception)
            self.init_status = 0
        
        
        function open_gripper(self)
            #open_gripper function for the MyRobot Class.
            #   Opens the gripper
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            if ~self.gripper_open_flag
                write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.gripper_motor_id, self.ADDR_MX_GOAL_POSITION, 0)
                self.gripper_open_flag = 1
            end
        end
        
        function close_gripper(self)
            #close_gripper function for the MyRobot Class.
            #   Closes the gripper
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            if self.gripper_open_flag
                write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.gripper_motor_id, self.ADDR_MX_GOAL_POSITION, 1023)
                self.gripper_open_flag = 0
            end
        end
        
        function actuate_gripper(self)
            #actuate_gripper function for the MyRobot Class.
            #   opens gripper if closed, closes gripper if open
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            if self.gripper_open_flag
                self.close_gripper()
            else
                self.open_gripper()
            end
        end
        
        function smooth_speed(self,joint_angles)
            #smooth_speed function for the MyRobot Class.
            #   Dynamically changes the speed of each joint to create
            #   smoother motion. It assures all joint movements finish at
            #   the same time
            #
            #Inputs:
            #   joint_angles : a vector representing joint angles [deg]
            #Outputs:
            #   None
            max_angle = max(abs(joint_angles))
            speed_per_deg = max_angle/100
            if speed_per_deg~=0
                new_speeds = abs(joint_angles/speed_per_deg)*0.01
                for i=1:length(self.motor_speed)
                    if new_speeds(i)==0
                        new_speeds(i)=self.motor_speed(i)
                    else
                        new_speeds(i)=new_speeds(i)*self.motor_speed(i)
                    end
                end
                self.set_speed(new_speeds,false)
            end
        end
        
        function create_rbt(self)
            #create_rbt function for the MyRobot Class.
            #   Creates a rigid body tree using the DH parameters of the
            #   MyRobot Class and sets up inverse kinematics using the
            #   matlab robotics toolbox
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            self.rbt = rigidBodyTree
            bodies = cell(4,1)
            joints = cell(4,1)
            for i = 1:4
                bodies{i} = rigidBody(['link' num2str(i)])
                joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute")
                joints{i}.PositionLimits = [self.joint_limits(i,1)*pi/180,self.joint_limits(i,2)*pi/180]
                setFixedTransform(joints{i},self.dh(i,:),"dh")
                bodies{i}.Joint = joints{i}
                if i == 1 # Add first body to base
                    addBody(self.rbt,bodies{i},"base")
                else # Add current body to previous body by name
                    addBody(self.rbt,bodies{i},bodies{i-1}.Name)
                end
            end 
            self.ik = inverseKinematics('RigidBodyTree',self.rbt)
        end
        
        function set_speed(self, speeds, overwrite_speeds)
            #set_speed function for the MyRobot Class.
            #   Sets individual motor speeds between 0# and 100#
            #
            #Inputs:
            #   speeds : a vector representing motor speeds for each motor
            #   ID between 0 and 1
            #   overwrite_speeds: boolean, if true class internal motor
            #   speeds are overwritten to motor speeds of function call
            #Outputs:
            #   None
            if overwrite_speeds
                self.motor_speed = speeds
            end
            for i=1:length(self.motor_ids)
                if speeds(i) > 0 && speeds(i) <= 1
                    speed = speeds(i)*1023
                    write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), 32, speed)
                    dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION)
                    dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION)
                    if dxl_comm_result ~= self.COMM_SUCCESS
                        print('\n#s', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result))
                    elseif dxl_error ~= 0
                        print('\n#s', getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
                    end
                else
                   print("\nMovement speed out of range, enter value between ]0,1]") 
                end
            end
        end
        
        function set_torque_limit(self, torques)
            #set_torque_limit function for the MyRobot Class.
            #   Sets individual motor torques between 0# and 100#
            #
            #Inputs:
            #   speeds : a vector representing motor torque for each motor
            #   ID between 0 and 1
            #Outputs:
            #   None
            
            self.motor_torque = torques
            for i=1:length(self.motor_ids)
                if torques(i) > 0 && torques(i) <= 1
                    write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), 34, torques(i)*1023)
                    dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION)
                    dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION)
                    if dxl_comm_result ~= self.COMM_SUCCESS
                        print('#s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result))
                    elseif dxl_error ~= 0
                        print('#s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
                    end
                else
                   print("\nTorque limit out of range, enter value between ]0,1]") 
                end
            end
                end
        
        function enable_motors(self)
            #enable_motors function for the MyRobot Class.
            #   Enables all motors
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            
            for i=1:length(self.motor_ids)
                write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
                dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION)
                dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION)
                if dxl_comm_result ~= self.COMM_SUCCESS
                    print('#s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result))
                elseif dxl_error ~= 0
                    print('#s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
                else
                    print('\nDynamixel has been successfully connected, torque mode enabled \n')
                end
            end
            
        end
        
        function deg_present_position = get_position(self, motor_id)
            #get_position function for the MyRobot Class.
            #   Reads current position of motor
            #
            #Inputs:
            #   motor_id : integer representing the motors ID
            #Outputs:
            #   deg_present_position : value of current position [deg]
            
            dxl_present_position = read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, motor_id, self.ADDR_MX_PRESENT_POSITION)
            deg_present_position = self.rot_to_deg(dxl_present_position)
            dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION)
            dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION)
            if dxl_comm_result ~= self.COMM_SUCCESS
                print('#s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result))
            elseif dxl_error ~= 0
                print('#s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
            end
        end
        
        function disable_motors(self)
            #disable_motors function for the MyRobot Class.
            #   Disables all motors
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            
            for i=1:length(self.motor_ids)
                write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE)
                dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION)
                dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION)
                if dxl_comm_result ~= self.COMM_SUCCESS
                    print('#s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result))
                elseif dxl_error ~= 0
                    print('#s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
                else
                    print('\nDynamixel succesfully disconnected')
                    
                end
            end
            closePort(self.port_num)          
            unloadlibrary(self.lib_name)
            self.init_status = 0
        end
        
        function deg = check_limits(self,deg, motor_id)
            #check_limits function for the MyRobot Class.
            #   Checks if joint angle is within motor limits, depending on
            #   the motor, see https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
            #
            #Inputs:
            #   deg : value for joint angle [deg]
            #   motor_id : int of motors ID
            #Outputs:
            #   deg : returns input value if checks pass [deg]
            if motor_id==self.motor_ids(1)
                assert(abs(deg) <= 130, "Angle Limits for first Axis Reached, Min/Max: +-130°")
            elseif motor_id==self.motor_ids(2)
                assert(deg <= 0 && deg >= -180, "Angle Limits for second Axis Reached, Min/Max: [0,-180]°")
            else
                assert(abs(deg) <= 100, "Angle Limits Reached, Min/Max: +-100°")
            end

        end
        
        function rot = deg_to_rot(self,deg)
            #deg_to_rot function for the MyRobot Class.
            #   Converts degree to units per rotation of motors
            #
            #Inputs:
            #   deg : value [deg]
            #Outputs:
            #   rot : value in units per rotation of motor
            rot = deg*1/0.29
        end
        
        function deg = rot_to_deg(self,rot)
            #rot_to_deg function for the MyRobot Class.
            #   Convers units per rotation of motors to degree
            #
            #Inputs:
            #   rot : value in units per rotation of motor
            #Outputs:
            #   deg : value [deg]
            deg = rot*0.29
        end
        
        function move_j(self,j1,j2,j3,j4)
            #move_j function for the MyRobot Class.
            #   Moves the robot arm to the desired joint angles, checks
            #   joint limits, updates internal robot state and waits until
            #   the joint angle error between desired and mesured joint
            #   angle is below 2°
            #
            #Inputs:
            #   j1 : value for joint one [deg]
            #   j2 : value for joint two [deg]
            #   j3 : value for joint three [deg]
            #   j4 : value for joint four [deg]

            #Outputs:
            #   None
            
            j1 = self.check_limits(j1, self.motor_ids(1))
            j2 = self.check_limits(j2, self.motor_ids(2))
            j3 = self.check_limits(j3, self.motor_ids(3))
            j4 = self.check_limits(j4, self.motor_ids(4))

            if self.use_smooth_speed_flag
                self.smooth_speed([j1 j2 j3 j4]-self.joint_angles)
            end
            self.joint_angles = [j1 j2 j3 j4]
            self.forward([j1 j2 j3 j4])
            if self.draw_robot_flag
                self.draw_robot()
            end
            j1 = j1 + self.joint_offsets(1)
            j2 = j2 + self.joint_offsets(2)
            j3 = j3 + self.joint_offsets(3)
            j4 = j4 + self.joint_offsets(4)           

            
            write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(1), self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j1))
            write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(2), self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j2))
            write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(3), self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j3))
            write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(4), self.ADDR_MX_GOAL_POSITION, self.deg_to_rot(j4))
            
            while 1
                self.read_joint_angles()
                if self.joint_angle_error<2
                    break
                end
            end
        end
        
        
        function draw_robot(self)
            #draw_robot function for the MyRobot Class.
            #   Draws robot coordinate frames using the rigid body tree
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            
            if self.rbt == 0
                self.create_rbt()
            end
           
            config = homeConfiguration(self.rbt)
            for i=1:length(self.joint_angles)
                config(i).JointPosition = self.joint_angles(i)*pi/180
            end
            if self.draw_robot_flag == 0
                figure(Name="RRRR Robot Model")
            end
            show(self.rbt,config)
            self.draw_robot_flag = 1
        end
        
        function ee_cartesian_coords = forward(self, j_a)
           #forward function for the MyRobot Class.
            #   Calculates forward transformation for all joint positions
            #   and end effector coordinates
            #
            #Inputs:
            #   j_a : a vector of four joint angles in [rad]
            #Outputs:
            #   ee_cartesian_coords : returns cartesian coordinates of end
            #   effector in the base coordinate system in [m]
            
            self.forward_transform = [cosd(j_a(1)) -sind(j_a(1))*cos(self.dh(1,2))  sind(j_a(1))*sin(self.dh(1,2)) self.dh(1,1)*cos(j_a(1))
                sind(j_a(1)) cosd(j_a(1))*cos(self.dh(1,2)) -cosd(j_a(1))*sin(self.dh(1,2)) self.dh(1,1)*sind(j_a(1))
                0 sin(self.dh(1,2)) cos(self.dh(1,2)) self.dh(1,3)
                0 0 0 1]
            
            self.joint_pos(:,1) = self.forward_transform * [0 0 0 1]' 
            self.joint_pos(:,1) = self.joint_pos(:,1) / self.joint_pos(4,1)
            
            for i=2:length(j_a)
                self.forward_transform = self.forward_transform * [cosd(j_a(i)) -sind(j_a(i))*cos(self.dh(i,2))  sind(j_a(i))*sin(self.dh(i,2)) self.dh(i,1)*cosd(j_a(i))
                    sind(j_a(i)) cosd(j_a(i))*cos(self.dh(i,2)) -cosd(j_a(i))*sin(self.dh(i,2)) self.dh(i,1)*sind(j_a(i))
                    0 sin(self.dh(i,2)) cos(self.dh(i,2)) self.dh(i,3)
                    0 0 0 1]  
                self.joint_pos(:,i) = self.forward_transform * [0 0 0 1]' 
                self.joint_pos(:,i) = self.joint_pos(:,i) / self.joint_pos(4,i)
            end
            
            ee_cartesian_coords = self.forward_transform * [0 0 0 1]' 
            ee_cartesian_coords = ee_cartesian_coords/ee_cartesian_coords(4)
        end
        
        function j_a = inverse(self, x,y,z,pitch)
            #inverse function for the MyRobot Class.
            #   Calculates inverse kinematics for the robot
            #
            #Inputs:
            #   x : value for desired x position of the robot end effector
            #   [m]
            #   y : value for desired y position of the robot end effector
            #   [m]
            #   z : value for desired z position of the robot end effector
            #   [m]
            #   pitch : value for desired x position of the robot end
            #   effector [deg]

            #Outputs:
            #   j_a : a vector containing joint angles [deg]
            
            j1 = atan2(y,x)
            j3 = acos( ((sqrt(x^2+y^2)-self.dh(4,1)*cos(pitch))^2 + (self.dh(1,3)-z)^2 - self.dh(2,1)^2 - self.dh(3,1)^2) / (2*self.dh(2,1)*self.dh(3,1)) )
            assert(isreal(j3),"Configuration Impossible")
            j2 = -atan2(z-self.dh(1,3)-self.dh(4,1)*sin(pitch),sqrt(x^2+y^2)-self.dh(4,1)*cos(pitch)) - atan2(self.dh(3,1)+self.dh(2,1)*cos(j3),self.dh(2,1)*sin(j3)) + (pi/2-j3)
            j4 = pitch - j2 - j3
            
            j_a = rad2deg([j1 j2 j3 j4])
            self.pitch = pitch
            assert(isreal(j_a),"Configuration Impossible")
        end
        
        function j_a = read_joint_angles(self)
            #read_joint_angles function for the MyRobot Class.
            #   reads joint angles of all motor IDs
            #
            #Inputs:
            #   None
            #Outputs:
            #   j_a : a vector containing joint angles [deg]
            j_a = zeros(4,1)
            for i=1:length(self.motor_ids)
                dxl_present_position = read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.motor_ids(i), 36)
                dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION)
                dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION)
                if dxl_comm_result ~= self.COMM_SUCCESS
                    print('#s\n', getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result))
                elseif dxl_error ~= 0
                    print('#s\n', getRxPacketError(self.PROTOCOL_VERSION, dxl_error))
                else
                    j_a(i) = self.rot_to_deg(dxl_present_position) - self.joint_offsets(i)
                    self.joint_angle_error(i) = j_a(i)-self.joint_angles(i)
                end 
            end
        end
        
        function ee_pos = read_ee_position(self)
           #read_ee_position function for the MyRobot Class.
            #   Reads motors joint angles and calculates the end effector
            #   position from that
            #
            #Inputs:
            #   None
            #Outputs:
            #   ee_pos : a vector containing the end effector position [m]
           j_a = self.read_joint_angles()
           ee_pos = self.forward(j_a)
        end
        
        function move_c (self,x,y,z,pitch)
            #move_c function for the MyRobot Class.
            #   Moves robot in cartesian space using inverse kinematics
            #
            #Inputs:
            #   x : value for desired x position of the robot end effector
            #   [m]
            #   y : value for desired y position of the robot end effector
            #   [m]
            #   z : value for desired z position of the robot end effector
            #   [m]
            #   pitch : value for desired x position of the robot end
            #   effector [deg]

            #Outputs:
            #   None
           j_a = self.inverse(x,y,z,deg2rad(pitch))
           self.move_j(j_a(1),j_a(2),j_a(3),j_a(4))
        end
        
        function record_configuration(self)
            #record_configuration function for the MyRobot Class.
            #   Records current robot configuration (joint angles, speed,
            #   torque, gripper state)
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            j_a = self.joint_angles
            torque = self.motor_torque(1)
            speed = self.motor_speed(1)
            if isempty(self.movement_history)
                self.movement_history = [j_a(1), j_a(2),j_a(3),j_a(4), speed, torque,self.gripper_open_flag]
            else
                self.movement_history = [self.movement_history j_a(1), j_a(2),j_a(3),j_a(4), speed, torque,self.gripper_open_flag]
            end
            print("\nRecorded Speed: #f, Torque: #f, \nJoint Positions: #f, #f, #f, #f,\nGripper open: #f",speed,torque,j_a(1),j_a(2),j_a(3),j_a(4), self.gripper_open_flag)
        end
        
        function delete_last_recorded_configuration(self)
            #delete_last_recorded_configuration function for the MyRobot Class.
            #   Deletes last recorded robot configuration
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            length_history = size(self.movement_history)
            if isempty(self.movement_history)
                print("No last history position") 
            elseif length_history(1)==1
                self.movement_history = []
            else
               self.movement_history(end,:) = []
            end
        end
        
        function play_configuration_history(self)
            #play_configuration_history function for the MyRobot Class.
            #   Plays recorded configuration history
            #
            #Inputs:
            #   None
            #Outputs:
            #   None
            if ~isempty(self.movement_history)
               length_history = size(self.movement_history)
               for i=1:length_history(1)
                  speed = self.movement_history(i,5)
                  torque = self.movement_history(i,6)
                  self.set_speed([speed,speed, speed, speed],true)
                  self.set_torque_limit([torque, torque, torque, torque])
                  self.move_j(self.movement_history(i,1),self.movement_history(i,2),self.movement_history(i,3),self.movement_history(i,4))
                  pause(1)
                  if self.gripper_open_flag ~= self.movement_history(i,7)
                      self.actuate_gripper()
                      pause(3)
                  end
               end
            end
        end

    end
end



