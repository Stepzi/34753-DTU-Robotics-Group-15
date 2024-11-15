import sys, os
import numpy as np

# Add the Dynamixel SDK path to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'DynamixelSDK/python/src')))

import dynamixel_sdk as dxl

# Constants and configuration
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
ADDR_MX_CW_COMPLIANCE_SLOPE = 28
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_PUNCH = 48
PROTOCOL_VERSION = 1.0
DXL_IDS = [1,2,3,4]
DEVICENAME = 'COM5'
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0



# Check if the specified COM port exists before attempting to open it
if os.path.exists(f"//./{DEVICENAME}"):
    # Initialize portHandler and packetHandler
    portHandler = dxl.PortHandler(DEVICENAME)
    packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
    
    # Open the port
    if portHandler.openPort():
        print(f"Successfully opened port {DEVICENAME}")
        
        # Set baud rate
        portHandler.setBaudRate(BAUDRATE)
        
        # Enable torque and configure other settings for each Dynamixel
        for DXL_ID in DXL_IDS:
            packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
            packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
            packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
            packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
            packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, 100)
            packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, 512)
    else:
        print(f"Failed to open port {DEVICENAME}")
else:
    print(f"Port {DEVICENAME} does not exist. Please connect a device.")

