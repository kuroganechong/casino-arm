import os
import numpy as np
#from scipy.optimize import minimize
# from numpy.linalg import inv

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
        
class Kinematics:
    # input accepts bit value of angles, store as radians
    def __init__(self, T1, T2, T3, T4, R1, R2, R3):
        self.T1 = T1/1024*2*np.pi
        self.T2 = T2/1024*2*np.pi
        self.T3 = T3/1024*2*np.pi
        self.T4 = T4/1024*2*np.pi
        self.R1 = R1
        self.R2 = R2
        self.R3 = R3
        self.input_x = self.input_y = self.input_z = 0

        # d = depth, T = rotation angle, r = radius, a = tilt angle
        self.d = [0, 0, 0, 0]
        self.T = [self.T1, self.T2, self.T3, self.T4]
        self.r = [self.R1, self.R1, self.R2, self.R3]
        self.a = [0, 0, -np.pi/2, np.pi/2]

    def dynamixel_read(self):
        # update current joint variables and return it
        print("Read current angles!")
        return [self.T1, self.T2, self.T3, self.T4]
    
    def dynamixel_write(self, angle1, angle2, angle3, angle4):
        # output motor position (does not update list)
        # Here I set it to update the list as substitution for now
        self.T1 = angle1
        self.T2 = angle2
        self.T3 = angle3
        self.T4 = angle4
        self.T = [self.T1, self.T2, self.T3, self.T4]
        
        print("Written new angles!")
    
    def forwardk(self):
        # DH Forward Kinematic Matrices
        # Take current joint variables, return only end pt position
        i = [0,1,2,3]
        dh_fk = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        for i in i:
            dh_fk = np.matmul(dh_fk, np.matrix([[np.cos(self.T[i]), -np.cos(self.a[i])*np.sin(self.T[i]), np.sin(self.a[i])*np.sin(self.T[i]), self.r[i]*np.cos(self.T[i])],
                [np.sin(self.T[i]), np.cos(self.a[i])*np.cos(self.T[i]), -np.sin(self.a[i])*np.cos(self.T[i]), self.r[i]*np.sin(self.T[i])],
                [0, np.sin(self.a[i]), np.cos(self.a[i]), self.d[i]],
                [0, 0, 0, 1]]))
        return np.matmul(dh_fk, np.matrix([[0],[0],[0],[1]]))

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
    
    def move_to(self, target_x, target_y, target_z):
        # update object variables
        self.input_x = target_x
        self.input_y = target_y
        self.input_z = target_z
        # function to round off angles
        def round_angle(x):
            if x < -np.pi:
                while x < -np.pi:
                    x += np.pi*2
                return x
            if x > np.pi:
                while x > np.pi:
                    x -= np.pi*2
                return x
            else:
                return x
        # step size
        DELTA = 8
        target = np.matrix([[self.input_x], [self.input_y], [self.input_z], [1]])
        while(1):
            current_pos = self.forwardk()
            # print('current pos: ')
            # print(current_pos)
            error_vector = np.subtract(target, current_pos)
            # within acceptable error
            if (abs(error_vector[0]) < 0.5 and abs(error_vector[1]) < 0.5 and abs(error_vector[2]) < 0.5 and abs(error_vector[3]) < 0.5):
                break
            # trunc_error_vector = np.multiply(error_vector[:-1], 1/DELTA)
            # jac_t = np.transpose(self.jacobian(self.T))
            angle_increment = np.matrix.tolist(np.matmul(np.transpose(self.jacobian(self.T)), np.multiply(error_vector[:-1], 1/DELTA)))
            # print(angle_increment)
            # print(self.T)
            self.T[0] += angle_increment[0][0]
            self.T[1] += angle_increment[1][0]
            self.T[2] += angle_increment[2][0]
            self.T[3] += angle_increment[3][0]
            self.T = [round_angle(item) for item in self.T]
        print('final position:')
        print(self.forwardk())
        print('final angles:')
        print([round(np.rad2deg(item),2) for item in self.T])
        return 1

    #def x(self, t):
    #    return self.input_x - ((self.R3*np.cos(t[3])+self.R2)*np.cos(t[0]+t[1]+t[2])+self.R1*np.cos(t[0]+t[1])+self.R1*np.cos(t[0]))

    #def y(self, t):
    #    return self.input_y - ((self.R3*np.cos(t[3])+self.R2)*np.sin(t[0]+t[1]+t[2])+self.R1*np.sin(t[0]+t[1])+self.R1*np.sin(t[0]))

    #def z(self, t):
    #    return self.input_z + self.R3*np.sin(t[3])

    #def min(self):
    #    return minimize(self.x, [0,0,0,0], method='BFGS', jac=self.x_der, bounds=bounds)

# Initialisation at position (x,y,z) = (0,3,0)
chain1 = Kinematics(0,512,512,0,5,1,2)    # joint variables
while(1):
    input_x = int(input("X: "))
    input_y = int(input("Y: "))
    input_z = int(input("Z: "))
    print("Calculating... do not press any keys!")
    chain1.move_to(input_x,input_y,input_z)
    print("press any key to continue, or ESC to quit")
    if getch() == chr(0x1b):
        break