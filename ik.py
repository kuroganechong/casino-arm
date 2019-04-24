import os
import time

import numpy as np
import serial

from dynamixel_sdk import (BROADCAST_ID,  # Uses Dynamixel SDK library
                           COMM_NOT_AVAILABLE, COMM_RX_CORRUPT,
                           COMM_RX_TIMEOUT, COMM_SUCCESS, COMM_TX_FAIL,
                           DXL_HIBYTE, DXL_HIWORD, DXL_LOBYTE, DXL_LOWORD,
                           DXL_MAKEDWORD, DXL_MAKEWORD, INST_BULK_READ,
                           INST_READ, RXPACKET_MAX_LEN, GroupSyncWrite,
                           PortHandler, Protocol1PacketHandler)

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()

class Kinematics:
    # Arduino setup
    ARDUINO_PORT = 'COM5'
    ARDUINO_BAUD = 9600

    # Data Byte Length
    LEN_AX12A_GOAL_POSITION       = 2
    LEN_AX12A_PRESENT_POSITION    = 4

    # Control table address
    ADDR_AX12A_TORQUE_ENABLE      = 24                 # Control table address is different in Dynamixel model
    ADDR_AX12A_GOAL_POSITION      = 30
    ADDR_AX12A_PRESENT_POSITION   = 36
    ADDR_AX12A_MOVE_SPEED =  32                      # Joint mode 0 - 1023 (114rpm)/Wheel mode 0 - 1023 CCW 1024 - 2047 CW (0 or 1024 as stop byte)
    ADDR_AX12A_CW_ANGLE_LIMIT = 6                 # For wheel mode, set both limits
    ADDR_AX12A_CCW_ANGLE_LIMIT = 8                 # to 0. For Joint, 255/3 (default)

    # Default setting
    DXL1_ID                      = 5                 # Shoulder
    DXL2_ID                      = 2                 # Elbow
    DXL3_ID                      = 4                 # Wrist 1 LR
    DXL4_ID                      = 1                 # Wrist 2 UD
    DXL5_ID                      = 6                # Gripper
    DXL6_ID                      = 3                # Card dispenser
    BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
    DEVICENAME                  = 'COM3'            # Check which port is being used on your controller

    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
    DXL_MAXIMUM_POSITION_VALUE  = 1023              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

    JOINT_LIM_HIGH = np.pi/2 - np.pi/6 # max 150 degrees
    JOINT_LIM_LOW = np.pi*3/2 + np.pi/6 # min 210 degrees

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = Protocol1PacketHandler()
    # for Protocol 1.0 Packet
    PKT_HEADER0 = 0
    PKT_HEADER1 = 1
    PKT_ID = 2
    PKT_LENGTH = 3
    PKT_INSTRUCTION = 4
    PKT_ERROR = 4
    PKT_PARAMETER0 = 5

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
        
    # Fix some error in library for read4ByteTxRx
    def read4ByteTxRx(self, port, dxl_id, address):
        data, result, error = self.readTxRx(port, dxl_id, address, 4)
        data_read = DXL_MAKEDWORD(DXL_MAKEWORD(data[0], data[1]),
                                    DXL_MAKEWORD(data[2], data[3])) if (result == COMM_SUCCESS) else 0
        return data_read, result, error

    def readTxRx(self, port, dxl_id, address, length):
        PKT_PARAMETER0 = 5
        txpacket = [0] * 8
        data = []

        if dxl_id >= BROADCAST_ID:
            return data, COMM_NOT_AVAILABLE, 0

        txpacket[self.PKT_ID] = dxl_id
        txpacket[self.PKT_LENGTH] = 4
        txpacket[self.PKT_INSTRUCTION] = INST_READ
        txpacket[self.PKT_PARAMETER0 + 0] = address
        txpacket[self.PKT_PARAMETER0 + 1] = length

        rxpacket, result, error = self.txRxPacket(port, txpacket)
        if result == COMM_SUCCESS:
            error = rxpacket[self.PKT_ERROR]

            data.extend(rxpacket[PKT_PARAMETER0: PKT_PARAMETER0 + length])

        return data, result, error

    def txRxPacket(self, port, txpacket):
        rxpacket = None
        error = 0

        # tx packet
        result = self.packetHandler.txPacket(port, txpacket)
        if result != COMM_SUCCESS:
            return rxpacket, result, error

        # (Instruction == BulkRead) == this function is not available.
        if txpacket[self.PKT_INSTRUCTION] == INST_BULK_READ:
            result = COMM_NOT_AVAILABLE

        # (ID == Broadcast ID) == no need to wait for status packet or not available
        if (txpacket[self.PKT_ID] == BROADCAST_ID):
            port.is_using = False
            return rxpacket, result, error

        # set packet timeout
        if txpacket[self.PKT_INSTRUCTION] == INST_READ:
            port.setPacketTimeout(txpacket[self.PKT_PARAMETER0 + 1] + 6)
        else:
            port.setPacketTimeout(6)  # HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

        # rx packet
        while True:
            rxpacket, result = self.rxPacket(port)
            if result != COMM_SUCCESS or txpacket[self.PKT_ID] == rxpacket[self.PKT_ID]:
                break

        if result == COMM_SUCCESS and txpacket[self.PKT_ID] == rxpacket[self.PKT_ID]:
            error = rxpacket[self.PKT_ERROR]

        return rxpacket, result, error

    def rxPacket(self, port):
        rxpacket = []

        result = COMM_TX_FAIL
        checksum = 0
        rx_length = 0
        wait_length = 10  # minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)
        # FIX: CHANGED 6 TO 10 for read_pos

        while True:
            rxpacket.extend(port.readPort(wait_length - rx_length))
            rx_length = len(rxpacket)
            if rx_length >= wait_length:
                # find packet header
                for idx in range(0, (rx_length - 1)):
                    if (rxpacket[idx] == 0xFF) and (rxpacket[idx + 1] == 0xFF):
                        break

                if idx == 0:  # found at the beginning of the packet
                    if (rxpacket[self.PKT_ID] > 0xFD) or (rxpacket[self.PKT_LENGTH] > RXPACKET_MAX_LEN) or (
                            rxpacket[self.PKT_ERROR] > 0x7F):
                        # unavailable ID or unavailable Length or unavailable Error
                        # remove the first byte in the packet
                        del rxpacket[0]
                        rx_length -= 1
                        continue

                    # re-calculate the exact length of the rx packet
                    if wait_length != (rxpacket[self.PKT_LENGTH] + self.PKT_LENGTH + 1):
                        wait_length = rxpacket[self.PKT_LENGTH] + self.PKT_LENGTH + 1
                        continue

                    if rx_length < wait_length:
                        # check timeout
                        if port.isPacketTimeout():
                            if rx_length == 0:
                                result = COMM_RX_TIMEOUT
                            else:
                                result = COMM_RX_CORRUPT
                            break
                        else:
                            continue

                    # calculate checksum
                    for i in range(2, wait_length - 1):  # except header, checksum
                        checksum += rxpacket[i]
                    checksum = ~checksum & 0xFF

                    # verify checksum
                    if rxpacket[wait_length - 1] == checksum:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                    break

                else:
                    # remove unnecessary packets
                    del rxpacket[0: idx]
                    rx_length -= idx

            else:
                # check timeout
                if port.isPacketTimeout():
                    if rx_length == 0:
                        result = COMM_RX_TIMEOUT
                    else:
                        result = COMM_RX_CORRUPT
                    break

        port.is_using = False

        #print "[RxPacket] %r" % rxpacket

        return rxpacket, result
    # end fix

    # Function to Enabling/Disabling torque
    def torque_enable(self, id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_AX12A_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            
            return self.torque_enable(id)
        elif dxl_error != 0:
            
            return self.torque_enable(id)
        else:
            print("Enable Dynamixel#%d success" % id)
            return 1

    def torque_disable(self, id):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_AX12A_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            
            return self.torque_disable(id)
        elif dxl_error != 0:
            
            return self.torque_disable(id)
        else:
            print("Disable Dynamixel#%d success" %id)
            return 1

    # Function to setup parameters for dynamixel sync write
    def add_params(self, id,params):
        dxl_addparam_result = self.groupSyncWrite.addParam(id, params)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed. Retrying..." % id)
            
            self.add_params(id,params)

    # Function to read present angle for motor #id, output radians
    def read_pos(self, id,enable_msg):
        dxl_present_position, dxl_comm_result, dxl_error = self.read4ByteTxRx(self.portHandler, id, self.ADDR_AX12A_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            
            
            return self.read_pos(id,enable_msg)
        elif dxl_error != 0:
            
            
            return self.read_pos(id,enable_msg)
        else:
            if(dxl_present_position > 1023 or dxl_present_position<0):
                return self.read_pos(id,enable_msg)
            if(enable_msg == 1):
                print("Read_pos success for Dynamixel#%d" %id)
            return dxl_present_position/1024*300*np.pi/180

    # Set motor 6 to wheel mode
    def set_wheel(self, id):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_AX12A_CCW_ANGLE_LIMIT, 0)
        if dxl_comm_result != COMM_SUCCESS:
            
            return self.set_wheel(id)
        elif dxl_error != 0:
            
            return self.set_wheel(id)
        else:
            print("Dynamixel#%d has been set as wheel mode" % id)
            return 1

    # Set move speed
    def set_joint_speed(self, id,speed):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, self.ADDR_AX12A_MOVE_SPEED, speed)
        if dxl_comm_result != COMM_SUCCESS:
            
            return self.set_joint_speed(id,speed)
        elif dxl_error != 0:
            
            return self.set_joint_speed(id,speed)
        else:
            print("Dynamixel#%d speed has been set" % id)
            return 1

    # AX12A/AX18A rotate 300 degrees
    def __init__(self, length1, length2, length3, length4):
        print("Start initialise")        
        self.length = [length1,length2,length3,length4]
        self.joint = [0,0,0,0] # joint = real joint angles (motor angles), non-offset. dynamixel_write/read use this
        self.theta = [0,0,0,0] # theta = angles based on fixed axes, non-offset. fk/ik use this
        self.dynamixel_read()
        # margin of error
        self.ERROR = 0.1
        self.STEP = 0.001

        # initialise
        self.set_wheel(self.DXL6_ID)
        self.set_joint_speed(self.DXL1_ID,100)
        self.set_joint_speed(self.DXL2_ID,100)
        self.set_joint_speed(self.DXL3_ID,100)
        self.set_joint_speed(self.DXL4_ID,100)
        self.set_joint_speed(self.DXL5_ID,300)
        self.grip(0)
        self.dynamixel_write([0,0,0,0])
        print('Initial pos: %s' %(self.fk([0,0,0,0])))
        print('Gripper: open')

    # read and update current joint angles, theta angles and return joint list in radians (non-offset)
    # Real setup motors range are (0, 300) and are offset by 150 degrees so that motor can move in either positive/negative directions 
    # For easier analysis the program sets initial position at 0 degrees so that the range becomes (0, 150) and (210,360)
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
        # round angle to [0,360] degrees (in radians)
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
        # save as non-offset for easier analysis
        self.joint = [unoffset_angle(self.read_pos(self.DXL1_ID,0)),unoffset_angle(self.read_pos(self.DXL2_ID,0)),\
        unoffset_angle(self.read_pos(self.DXL3_ID,0)),unoffset_angle(self.read_pos(self.DXL4_ID,0))]
        joint = self.joint
        self.theta = [joint[0],round_angle(joint[1]+joint[0]),round_angle(joint[2]+joint[1]+joint[0]),joint[3]]
        print("Read current angles: %s" %[round(np.rad2deg(joint),2) for joint in self.joint])
        return self.joint
    
    # input joint angles radians (non offset)
    # outputs motor position (after offset) and update self angle list (non offset)
    def dynamixel_write(self, joint):
        print("-Start- \ndynamixel_write for angles %s before offset [limit (270,360) and (0,90)]" % [round(np.rad2deg(joint),2) for joint in joint])
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
        joint = [int(offset_angle(joint)/300/np.pi*180*1024) for joint in joint]
        print("after offset %s" % [round(np.rad2deg(joint*300*np.pi/180/1024),2) for joint in joint])
        angle1 = joint[0]
        angle2 = joint[1]
        angle3 = joint[2]
        angle4 = joint[3]
        # Enable Dynamixels Torque
        enabled1 = self.torque_enable(self.DXL1_ID)
        enabled2 = self.torque_enable(self.DXL2_ID)
        enabled3 = self.torque_enable(self.DXL3_ID)
        enabled4 = self.torque_enable(self.DXL4_ID)
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
        param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(angle1)), DXL_HIBYTE(DXL_LOWORD(angle1))]
        param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(angle2)), DXL_HIBYTE(DXL_LOWORD(angle2))]
        param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(angle3)), DXL_HIBYTE(DXL_LOWORD(angle3))]
        param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(angle4)), DXL_HIBYTE(DXL_LOWORD(angle4))]

        # Add Dynamixels goal position value to the Syncwrite parameter storage
        self.add_params(self.DXL1_ID, param_goal_position_1)
        self.add_params(self.DXL2_ID, param_goal_position_2)
        self.add_params(self.DXL3_ID, param_goal_position_3)
        self.add_params(self.DXL4_ID, param_goal_position_4)
        
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("dynamixel_write result error %s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        print("Waiting to stop moving...")
        while 1:
            # both angle and present_position already offset
            dxl1_present_position = int(self.read_pos(self.DXL1_ID,0)/300/np.pi*180*1024)
            dxl2_present_position = int(self.read_pos(self.DXL2_ID,0)/300/np.pi*180*1024)
            dxl3_present_position = int(self.read_pos(self.DXL3_ID,0)/300/np.pi*180*1024)
            dxl4_present_position = int(self.read_pos(self.DXL4_ID,0)/300/np.pi*180*1024)
            if ((abs(angle1 - dxl1_present_position) <= self.DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(angle2 - dxl2_present_position) <= self.DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(angle3 - dxl3_present_position) <= self.DXL_MOVING_STATUS_THRESHOLD) and \
                (abs(angle4 - dxl4_present_position) <= self.DXL_MOVING_STATUS_THRESHOLD)):
                break
        print("Stopped")

        # Disable Dynamixel Torque
        self.torque_disable(self.DXL1_ID)
        self.torque_disable(self.DXL2_ID)
        self.torque_disable(self.DXL3_ID)
        self.torque_disable(self.DXL4_ID)
        
        # Update final angles, save as unoffset for easier analysis
        print("Verifying final angles...")
        self.dynamixel_read()
        print("Written the following new angles: %s" % [round(np.rad2deg(joint),2) for joint in self.joint])
        print("-end-")
        return 1
    
    def lengthc(self,theta4):
        output = self.length[2]+self.length[3]*np.cos(theta4)
        return output

    # forward kinematics, input: theta(radians), output: [x,y,z]
    def fk(self,theta):
        length = self.length
        return [self.lengthc(theta[3])*np.cos(theta[2])+length[1]*np.cos(theta[1])+length[0]*np.cos(theta[0]),\
        self.lengthc(theta[3])*np.sin(theta[2])+length[1]*np.sin(theta[1])+length[0]*np.sin(theta[0]),\
        length[3]*np.sin(theta[3])]

    # input: current position [x,y,z], target [x,y,z] output: vector pointing from current pos to target [x,y,z]
    def error_pos(self,current_pos,target):
        return [target[0]-current_pos[0],target[1]-current_pos[1],target[2]-current_pos[2]]

    # calculate theta 3 based on theta1,2,x,y (radians)
    def caltheta3(self,theta1,theta2,x,y):
        length = self.length
        return np.arctan((y - length[1]*np.sin(theta2)-length[0]*np.sin(theta1))/(x - length[1]*np.cos(theta2)-length[0]*np.cos(theta1)))

    # calculate theta 4 based on z (radians)
    def caltheta4(self,z):
        length = self.length
        return np.arcsin(z/length[3])

    # transpose jacobian for ik calculation, input: theta1, theta2, output: 2x2 matrix
    def transposejacobian(self,theta1,theta2):
        length = self.length
        ii = -np.sin(theta1)*length[0]
        ij = np.cos(theta1)*length[0]
        ji = -np.sin(theta2)*length[1]
        jj = np.cos(theta2)*length[1]
        return [[ii,ij],[ji,jj]]

    # inverse kinematics, input: current theta(radians), target [x,y,z], output: joint(radians)
    def ik(self,theta,target):
        length = self.length
        # round angle to [0,360] degrees (in radians)
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
        # margin of error
        ERROR = self.ERROR
        # checking validity of z pos
        if target[2] > length[3]:
            print("z is too high, replacing with max value %s" % (length[3]))
            target[2] = length[3]
            print("new target is %s" %target)
        if target[2] < -length[3]:
            print("z is too low, replacing with min value %s" % (-length[3]))
            target[2] = -length[3]
            print("new target is %s" %target)
        # checking validity of target distance
        real_l_xy = np.sum(length) - length[3] + length[3]*np.cos(self.caltheta4(target[2]))
        target_distance_xy = np.sqrt(target[0]**2 + target[1]**2)
        if target_distance_xy > real_l_xy:
            # too far, scale down to size
            target = np.multiply(target,real_l_xy/target_distance_xy)
            print("replaced unreachable old target with new target %s" %target)
        # error vector
        error = self.error_pos(self.fk(theta),target)
        count = 1
        time_start = time.monotonic()
        time_chkpt_start = time.monotonic()
        while abs(error[0]) > ERROR or abs(error[1]) > ERROR or abs(error[2]) > ERROR:
            angles = np.matmul(self.transposejacobian(theta[0],theta[1]),[error[0],error[1]])
            # too slow
            if (time.monotonic() - time_chkpt_start > 3) or (sum([abs(x) for x in angles]) < 0.01):
                print("restart!")
                time_chkpt_start = time.monotonic()
                #pick another random input if angle_increment ~ 0 but error_vector > ERROR
                theta = np.random.randint(7, size=4).tolist()
            theta[0] = round_angle(theta[0] + angles[0]*self.STEP)
            theta[1] = round_angle(theta[1] + angles[1]*self.STEP)
            theta[2] = self.caltheta3(theta[0],theta[1],target[0],target[1])
            theta[3] = self.caltheta4(target[2])
            theta = [round_angle(x) for x in theta]
            error = self.error_pos(self.fk(theta),target)
            count += 1
        # finished calculating, get real joint angles (motor angles) based on theta
        joint = [theta[0],round_angle(theta[1]-theta[0]),round_angle(theta[2]-theta[1]),theta[3]]
        end_time = time.monotonic() - time_start
        print("number of tries: %s" %count)
        print("time elapsed: %s" %end_time)
        print("original theta: %s" %[round(np.rad2deg(item),2) for item in theta])
        print("original joint: %s" %[round(np.rad2deg(item),2) for item in joint])
        print("original pos: %s" %([round(item,2) for item in self.fk(theta)]))
        print("APPLY JOINT LIMIT")
        # applying joint limit to result
        if (joint[0] > self.JOINT_LIM_HIGH and joint[0] < self.JOINT_LIM_LOW):
            if abs(joint[0] - self.JOINT_LIM_HIGH) < abs(joint[0] - self.JOINT_LIM_LOW):
                print("joint 1 is over limit and closer to 90 degrees")
                joint[0] = theta[0] = self.JOINT_LIM_HIGH
            else:
                print("joint 1 is over limit and closer to 270 degrees")
                joint[0] = theta[0] = self.JOINT_LIM_LOW
        if (joint[1] > self.JOINT_LIM_HIGH and joint[1] < self.JOINT_LIM_LOW):
            if abs(joint[1] - self.JOINT_LIM_HIGH) < abs(joint[1] - self.JOINT_LIM_LOW):
                print("joint 2 is over limit and closer to 90 degrees")
                joint[1] = self.JOINT_LIM_HIGH
                theta[1] = round_angle(self.JOINT_LIM_HIGH+theta[0])
            else:
                print("joint 2 is over limit and closer to 270 degrees")
                joint[1] = self.JOINT_LIM_LOW
                theta[1] = round_angle(self.JOINT_LIM_LOW+theta[0])
        if (joint[2] > self.JOINT_LIM_HIGH and joint[2] < self.JOINT_LIM_LOW):
            if abs(joint[2] - self.JOINT_LIM_HIGH) < abs(joint[2] - self.JOINT_LIM_LOW):
                print("joint 3 is over limit and closer to 90 degrees")
                joint[2] = self.JOINT_LIM_HIGH
                theta[2] = round_angle(self.JOINT_LIM_LOW+theta[1])
            else:
                print("joint 3 is over limit and closer to 270 degrees")
                joint[2] = self.JOINT_LIM_LOW
                theta[2] = round_angle(self.JOINT_LIM_LOW+theta[1])
        print("final theta: %s" %[round(np.rad2deg(item),2) for item in theta])
        print("final joint: %s" %[round(np.rad2deg(item),2) for item in joint])
        print("final pos: %s" %([round(item,2) for item in self.fk(theta)]))
        return joint

    # function to move arm, returns run time
    def move_to(self,target):
        start_time = time.monotonic()
        print("\nStep 1: Update current pos")
        self.dynamixel_read()
        print("\nStep 2: Calculate theta/joint angles")
        joint = self.ik(self.theta,target)
        print("\nStep 3: Write joint angles")
        self.dynamixel_write(joint)
        time_elapsed = time.monotonic() - start_time
        print("\nEND: Time elapsed %s" %time_elapsed)
        return time_elapsed

    # Input = 1: grip, 0: ungrip
    def grip(self, input):
        CLOSE_POS = 512
        OPEN_POS = 0
        if input == 1:
            enabled = self.torque_enable(self.DXL5_ID)
            if (enabled != 1):
                print("Fail to enable motor 5")
                return 0
            
            # Write goal position
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_AX12A_GOAL_POSITION, CLOSE_POS)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            print("Waiting to stop moving...")
            while 1:
                dxl5_present_position = int(self.read_pos(self.DXL5_ID,0)/300/np.pi*180*1024)
                if ((abs(CLOSE_POS - dxl5_present_position) <= self.DXL_MOVING_STATUS_THRESHOLD)):
                    break
            print("Stopped")

            self.torque_disable(self.DXL5_ID)
            print("Gripper activated!")
            return 1
        if input == 0:
            enabled = self.torque_enable(self.DXL5_ID)
            if (enabled != 1):
                print("Fail to enable motor 5")
                return 0
            
            # Write goal position
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL5_ID, self.ADDR_AX12A_GOAL_POSITION, OPEN_POS)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            
            print("Waiting to stop moving...")
            while 1:
                dxl5_present_position = int(self.read_pos(self.DXL5_ID,0)/300/np.pi*180*1024)
                if ((abs(OPEN_POS - dxl5_present_position) <= self.DXL_MOVING_STATUS_THRESHOLD)):
                    break
            print("Stopped")

            self.torque_disable(self.DXL5_ID)
            print("Gripper deactivated!")
            return 1

    # Dispense a card
    # Connected to digital distance sensor through arduino
    def dispense(self):
        # Start serial communication with arduino
        dist_sensor = serial.Serial(self.ARDUINO_PORT, self.ARDUINO_BAUD)
        CW_SPEED = 1374   #0-1023 ccw, 1024 - 2047 CW
        CW_STOP_SPEED = 1024   #0 for CCW, 1024 for CW
        CCW_SPEED = 350   #0-1023 ccw, 1024 - 2047 CW
        CCW_STOP_SPEED = 0   #0 for CCW, 1024 for CW
        CCW_INTERVAL = 1.5

        enabled = self.torque_enable(self.DXL6_ID)
        if (enabled != 1):
            print("Fail to enable motor 6")
            return 0

        # Start dispensing
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_AX12A_MOVE_SPEED, CW_SPEED)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        while (dist_sensor.readline() != b'1\r\n'):
            dist_sensor.flush()

        # Stop
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_AX12A_MOVE_SPEED, CW_STOP_SPEED)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        time.sleep(0.5)

        # Retract
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_AX12A_MOVE_SPEED, CCW_SPEED)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        time.sleep(CCW_INTERVAL)

        # Stop
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL6_ID, self.ADDR_AX12A_MOVE_SPEED, CCW_STOP_SPEED)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        
        self.torque_disable(self.DXL6_ID)
        print("Card dispensed!")
        return 1
        
# Main loop
chain1 = Kinematics(28,28,7,4)    # joint variables
while (1):
    mode = int(input("select what you want to do(1:move_to, 2: dispense card, 3: grip, 4: view variables): "))
    if (mode == 1):
        target = [float(input("X: ")),\
        float(input("Y: ")),\
        float(input("Z: "))]
        chain1.move_to(target)
        print("press any key to continue, or ESC to quit")
        if getch() == chr(0x1b):
            break
    elif (mode == 2):
        chain1.dispense()
        print("press any key to continue, or ESC to quit")
        if getch() == chr(0x1b):
            break
    elif (mode == 3):
        pos = chain1.read_pos(chain1.DXL5_ID,1)
        print("\nCurrent gripper pos: %s" %(pos/np.pi*180/300*1024))
        if (pos > np.pi/2):
            chain1.grip(0)
        else:
            chain1.grip(1)
        print("press any key to continue, or ESC to quit")
        if getch() == chr(0x1b):
            break
    elif (mode == 4):
        print("Length: %s, Joint: %s, Theta: %s, ERROR: %s" % (chain1.length,[round(np.rad2deg(joint),2) for joint in chain1.joint],\
        [round(np.rad2deg(joint),2) for joint in chain1.theta],chain1.ERROR))
        print("press any key to continue, or ESC to quit")
        if getch() == chr(0x1b):
            break
