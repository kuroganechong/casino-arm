import os
import numpy as np
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                     # Uses Dynamixel SDK library

# Data Byte Length
LEN_AX12A_GOAL_POSITION       = 4
LEN_AX12A_PRESENT_POSITION    = 4

# Control table address
ADDR_AX12A_TORQUE_ENABLE      = 24                 # Control table address is different in Dynamixel model
ADDR_AX12A_GOAL_POSITION      = 30
ADDR_AX12A_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1                 # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                 # Shoulder
DXL2_ID                      = 2                 # Elbow
DXL3_ID                      = 4                 # Wrist 1 LR
DXL4_ID                      = 5                 # Wrist 2 UD
DXL5_ID                      = 5                # Gripper
BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX12A_GOAL_POSITION, LEN_AX12A_GOAL_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Function to Enabling/Disabling torque
def torque_enable(id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_AX12A_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print('id' + str(id) + 'torque_enable result error')
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return 0
    elif dxl_error != 0:
        print('id' + str(id) + 'torque_enable packet error')
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        return 0
    else:
        print("Dynamixel#%d has been successfully connected" % id)
        return 1

def torque_disable(id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_AX12A_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print('id' + str(id) + 'torque_disable result error')
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return 0
    elif dxl_error != 0:
        print('id' + str(id) + 'torque_disable packet error')
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        return 0
    else:
        return 1

# Function to setup parameters for dynamixel sync write
def add_params(id,params):
    dxl_addparam_result = groupSyncWrite.addParam(id, params)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % id)
        quit()

# Function to read present angle for motor #id, output radians
def read_pos(id):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, id, ADDR_AX12A_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print('id' + str(id) + 'read_pos result error')
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print('id' + str(id) + 'read_pos packet error')
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    return dxl_present_position/1024*300*np.pi/180

class Kinematics:
    # AX12A/AX18A rotate 300 degrees
    def __init__(self, R1, R2, R3):
        self.R1 = R1
        self.R2 = R2
        self.R3 = R3

        # d = depth, T = rotation angle, r = radius, a = tilt angle
        self.d = [0, 0, 0, 0] 
        self.T = [0, 0, 0, 0]  #radians
        self.r = [self.R1, self.R1, self.R2, self.R3]
        self.a = [0, 0, -np.pi/2, np.pi/2] #radians

        # initialise
        self.dynamixel_write(0,0,0,0)
        print('Initial pos: ')
        print(np.transpose(self.forwardk()))

    def x_der(self, t):
        return [-(self.R3*np.cos(t[3])+self.R2)*np.sin(t[0]+t[1]+t[2])-self.R1*(np.sin(t[0]+t[1])+np.sin(t[0])),\
        -(self.R3*np.cos(t[3])+self.R2)*np.sin(t[0]+t[1]+t[2])-self.R1*np.sin(t[0]+t[1]),\
        -(self.R3*np.cos(t[3])+self.R2)*np.sin(t[0]+t[1]+t[2]),\
        -self.R3*np.sin(t[3])*np.cos(t[0]+t[1]+t[2])]

    def y_der(self, t):
        return [(self.R3*np.cos(t[3])+self.R2)*np.cos(t[0]+t[1]+t[2])-self.R1*(np.cos(t[0]+t[1])+np.cos(t[0])),\
        (self.R3*np.cos(t[3])+self.R2)*np.cos(t[0]+t[1]+t[2])+self.R1*np.cos(t[0]+t[1]),\
        (self.R3*np.cos(t[3])+self.R2)*np.cos(t[0]+t[1]+t[2]),\
        -self.R3*np.sin(t[3])*np.sin(t[0]+t[1]+t[2])]

    def z_der(self, t):
        return [0,0,0,-self.R3*np.cos(t[3])]
    
    def jacobian(self, t):
        return np.matrix([self.x_der(t),self.y_der(t),self.z_der(t)])
    
    # update current joint angles and return the list in radians
    def dynamixel_read(self):
        # function to round off angles to principal angles [0,360] in radians and remove offset by 150 degrees
        def unoffset_angle(x):
            x -= 150/180*np.pi   # remove offset, then reduce to 0,360
            if x < 0:
                while x < 0:
                    x += np.pi*2
                return x
            if x > 2*np.pi:
                while x > 2*np.pi:
                    x -= np.pi*2
                return x
            else:
                return x
        # save as unoffset for easier analysis
        self.T = [unoffset_angle(read_pos(DXL1_ID)),unoffset_angle(read_pos(DXL2_ID)),unoffset_angle(read_pos(DXL3_ID)),unoffset_angle(read_pos(DXL4_ID))]
        print("Read current angles!")
        return self.T
    
    # input angles radians
    # outputs motor position and update self angle list
    def dynamixel_write(self, angle1, angle2, angle3, angle4):
        # function to round off angles to principal angles [0,360] in radians and add offset by 150 degrees
        def offset_angle(x):
            x += 150/180*np.pi   # offset, then reduce to 0,360
            if x < 0:
                while x < 0:
                    x += np.pi*2
                return x
            if x > 2*np.pi:
                while x > 2*np.pi:
                    x -= np.pi*2
                return x
            else:
                return x
        # function to round off angles to principal angles [0,360] in radians and remove offset by 150 degrees
        def unoffset_angle(x):
            x -= 150/180*np.pi   # remove offset, then reduce to 0,360
            if x < 0:
                while x < 0:
                    x += np.pi*2
                return x
            if x > 2*np.pi:
                while x > 2*np.pi:
                    x -= np.pi*2
                return x
            else:
                return x
        angle1 = int(offset_angle(angle1)/300/np.pi*180*1024)
        angle2 = int(offset_angle(angle2)/300/np.pi*180*1024)
        angle3 = int(offset_angle(angle3)/300/np.pi*180*1024)
        angle4 = int(offset_angle(angle4)/300/np.pi*180*1024)
        if (angle1 > 1024 or angle2 > 1024 or angle3 > 1024 or angle4 > 1024):
            print("angles out of limit!")
            return 0
        # Enable Dynamixels Torque
        enabled1 = torque_enable(DXL1_ID)
        enabled2 = torque_enable(DXL2_ID)
        enabled3 = torque_enable(DXL3_ID)
        enabled4 = torque_enable(DXL4_ID)
        if (enabled1 != 1):
            print("Fail to enable motor 1")
            return 0
        if (enabled2 != 1):
            print("Fail to enable motor 2")
            return 0
        if (enabled3 != 1):
            print("Fail to enable motor 3")
            return 0
        if (enabled4 != 1):
            print("Fail to enable motor 4")
            return 0
        # Allocate goal position value into byte array
        param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(angle1)), DXL_HIBYTE(DXL_LOWORD(angle1)), DXL_LOBYTE(DXL_HIWORD(angle1)), DXL_HIBYTE(DXL_HIWORD(angle1))]
        param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(angle2)), DXL_HIBYTE(DXL_LOWORD(angle2)), DXL_LOBYTE(DXL_HIWORD(angle2)), DXL_HIBYTE(DXL_HIWORD(angle2))]
        param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(angle3)), DXL_HIBYTE(DXL_LOWORD(angle3)), DXL_LOBYTE(DXL_HIWORD(angle3)), DXL_HIBYTE(DXL_HIWORD(angle3))]
        param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(angle4)), DXL_HIBYTE(DXL_LOWORD(angle4)), DXL_LOBYTE(DXL_HIWORD(angle4)), DXL_HIBYTE(DXL_HIWORD(angle4))]

        # Add Dynamixels goal position value to the Syncwrite parameter storage
        add_params(DXL1_ID, param_goal_position_1)
        add_params(DXL2_ID, param_goal_position_2)
        add_params(DXL3_ID, param_goal_position_3)
        add_params(DXL4_ID, param_goal_position_4)
        
        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print('dynamixel_write result error')
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        while 1:
            # both angle and present_position already offset
            dxl1_present_position = int(read_pos(DXL1_ID)/300/np.pi*180*1024)
            dxl2_present_position = int(read_pos(DXL2_ID)/300/np.pi*180*1024)
            dxl3_present_position = int(read_pos(DXL3_ID)/300/np.pi*180*1024)
            dxl4_present_position = int(read_pos(DXL4_ID)/300/np.pi*180*1024)
            if ((abs(angle1 - dxl1_present_position) < DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(angle2 - dxl2_present_position) < DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(angle3 - dxl3_present_position) < DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(angle4 - dxl4_present_position) < DXL_MOVING_STATUS_THRESHOLD)):
                break

        # Disable Dynamixel Torque
        torque_disable(DXL1_ID)
        torque_disable(DXL2_ID)
        torque_disable(DXL3_ID)
        torque_disable(DXL4_ID)
        
        # Update final angles, save as unoffset for easier analysis
        self.T = [unoffset_angle(read_pos(DXL1_ID)),unoffset_angle(read_pos(DXL2_ID)),unoffset_angle(read_pos(DXL3_ID)),unoffset_angle(read_pos(DXL4_ID))]
        print("Written new angles!")
        return 1
    
    # DH Forward Kinematic Matrices
    # Take current joint variables, return only end pt position
    def forwardk(self):
        angle = [item for item in self.T]
        i = [0,1,2,3]
        dh_fk = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        for i in i:
            dh_fk = np.matmul(dh_fk, np.matrix([[np.cos(angle[i]), -np.cos(self.a[i])*np.sin(angle[i]), np.sin(self.a[i])*np.sin(angle[i]), self.r[i]*np.cos(angle[i])],
                [np.sin(angle[i]), np.cos(self.a[i])*np.cos(angle[i]), -np.sin(self.a[i])*np.cos(angle[i]), self.r[i]*np.sin(angle[i])],
                [0, np.sin(self.a[i]), np.cos(self.a[i]), self.d[i]],
                [0, 0, 0, 1]]))
        return np.matmul(dh_fk, np.matrix([[0],[0],[0],[1]]))

    def move_to(self, target_x, target_y, target_z):
        # function to round off angles to principal angles [0,360] in radians
        def round_angle(x):
            if x < 0:
                while x < 0:
                    x += np.pi*2
                return x
            if x > 2*np.pi:
                while x > 2*np.pi:
                    x -= np.pi*2
                return x
            else:
                return x
        # function to round off angles to principal angles [0,360] in radians and add offset by 150 degrees
        def offset_angle(x):
            x += 150/180*np.pi   # offset, then reduce to 0,360
            if x < 0:
                while x < 0:
                    x += np.pi*2
                return x
            if x > 2*np.pi:
                while x > 2*np.pi:
                    x -= np.pi*2
                return x
            else:
                return x
        # step size
        DELTA = 8
        # error size
        ERROR = 0.5
        target = np.matrix([[target_x], [target_y], [target_z], [1]])
        # Start timer
        start = time.monotonic()
        # Update latest angles before starting
        self.dynamixel_read()
        while(1):
            current_pos = self.forwardk()
            error_vector = np.subtract(target, current_pos)
            # angle increment = transpose of jacobian * errorvector * step, store as list
            angle_increment = np.matrix.tolist(np.matmul(np.transpose(self.jacobian(self.T)), np.multiply(error_vector[:-1], 1/DELTA)))
            # within acceptable error
            if (abs(error_vector[0]) < ERROR and abs(error_vector[1]) < ERROR and abs(error_vector[2]) < ERROR and abs(error_vector[3]) < ERROR):
                break
            # too slow
            elif (time.monotonic() - start > 4) or (sum([abs(x) for [x] in angle_increment]) < 0.01):
                #pick another random input if angle_increment ~ 0 but error_vector > ERROR
                self.T = np.random.randint(7, size=4).tolist()

            self.T[0] += angle_increment[0][0]
            self.T[1] += angle_increment[1][0]
            self.T[2] += angle_increment[2][0]
            self.T[3] += angle_increment[3][0]
        self.T = [round_angle(item) for item in self.T]
        end = time.monotonic()
        print("Calculation done! Time used: ")
        print(end - start)
        print('calculated angles in original axis')
        print([round(np.rad2deg(item),2) for item in self.T])
        print('calculated angles in offset axis')
        print([round(np.rad2deg(offset_angle(item)),2) for item in self.T])
        print('calculated position')
        print(self.forwardk())
        
        # write to motor
        if(self.dynamixel_write(self.T[0],self.T[1],self.T[2],self.T[3]) != 1):
            print("Error writing to target!")
            return 'error'

        return (end - start)

# Main loop
# Initialisation at position (x,y,z) = (r1 + r1 + r2 + r3,0,0)
chain1 = Kinematics(5,1,2)    # joint variables
while (1):
    input_x = float(input("X: "))
    input_y = float(input("Y: "))
    input_z = float(input("Z: "))
    print("Calculating... do not press any keys!")
    if(chain1.move_to(input_x,input_y,input_z) == 'error'):
        break
    print("press any key to continue, or ESC to quit")
    if getch() == chr(0x1b):
        break
'''
    x=-13
    y=-13
    z=-2
    loopstart = time.monotonic()
    counter = 0
    success = 0
    longestt = 0.0
    longestpos = [0,0,0]
    while (x<=13):
        y = -13
        while (y<=13):
            z = -2
            while (z<=2):
                counter += 1
                #f = open('output.txt','a')
                input_x=x
                input_y=y
                input_z=z
                #f.write("\ncurrent target: ")
                #f.write(str([input_x,input_y,input_z]))
                print("\ncurrent target: ")
                print(str([input_x,input_y,input_z]))
                if (input_x**2 +input_y**2 > 121):
                    #f.write("this target is not reachable!\n")
                    print("this target is not reachable!\n")
                    #f.close()
                else:
                    #f.close()
                    returntime = chain1.move_to(input_x,input_y,input_z)
                    print(returntime)
                    if returntime > longestt:
                        longestt = returntime
                        longestpos = [x,y,z]
                        print("current slowest: ")
                        print(longestt)
                    success += 1
                z += 1
            y += 1
        x += 1
    elapsed = time.monotonic() - loopstart
    print("Finished!")
    print("Total Positions: ")
    print(counter)
    print("Positions successfully calculated: ")
    print(success)
    print("Total time elapsed: ")
    print(elapsed)
    print("Slowest: ")
    print(longestpos)
    print(longestt)
'''