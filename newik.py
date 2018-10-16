# Test ik algorithm with visual
import numpy as np
import time
import tkinter as tk

class Kinematics:
    def __init__(self,length1, length2, length3, length4):
        self.length = [length1,length2,length3,length4]

    def lengthc(self,theta4):
        output = self.length[2]+self.length[3]*np.cos(theta4)
        return output

    def fk(self,theta):
        length = self.length
        output = [self.lengthc(theta[3])*np.cos(theta[2])+length[1]*np.cos(theta[1])+length[0]*np.cos(theta[0]),\
        self.lengthc(theta[3])*np.sin(theta[2])+length[1]*np.sin(theta[1])+length[0]*np.sin(theta[0]),\
        length[3]*np.sin(theta[3])]
        return output

    def error_pos(self,current_pos,target):
        output = [target[0]-current_pos[0],target[1]-current_pos[1],target[2]-current_pos[2]]
        return output

    def caltheta3(self,theta1,theta2,x,y):
        length = self.length
        output = np.arctan((y - length[1]*np.sin(theta2)-length[0]*np.sin(theta1))/(x - length[1]*np.cos(theta2)-length[0]*np.cos(theta1)))
        return output

    def caltheta4(self,z):
        length = self.length
        output = np.arcsin(z/length[3])
        return output

    def inversejacobian(self,theta1,theta2):
        length = self.length
        alpha = theta2 - theta1
        if alpha == 0:
            return 0
        else:
            ii = np.cos(theta2)/length[0]*np.sin(alpha)
            ij = np.sin(theta2)/length[0]*np.sin(alpha)
            ji = -np.cos(theta1)/length[1]*np.sin(alpha)
            jj = -np.sin(theta1)/length[1]*np.sin(alpha)
            output = [[ii,ij],[ji,jj]]
            return output

    def transposejacobian(self,theta1,theta2):
        length = self.length
        ii = -np.sin(theta1)*length[0]
        ij = np.cos(theta1)*length[0]
        ji = -np.sin(theta2)*length[1]
        jj = np.cos(theta2)*length[1]
        output = [[ii,ij],[ji,jj]]
        return output

    def ik(self,theta,target):
        length = self.length
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
        ERROR = 0.1
        if target[2] > length[3]:
            print("z is too high, replacing with max value %s" % (length[3]))
            target[2] = length[3]
            print("new target is %s" %target)
        if target[2] < -length[3]:
            print("z is too low, replacing with min value %s" % (-length[3]))
            target[2] = -length[3]
            print("new target is %s" %target)
        real_l_xy = np.sum(length) - length[3] + length[3]*np.cos(self.caltheta4(target[2]))
        target_distance_xy = np.sqrt(target[0]**2 + target[1]**2)
        if target_distance_xy > real_l_xy:
            target = np.multiply(target,real_l_xy/target_distance_xy)
            print("replaced unreachable old target with new target %s" %target)
        error = self.error_pos(self.fk(theta),target)
        count = 1
        time_start = time.monotonic()
        while abs(error[0]) > ERROR or abs(error[1]) > ERROR or abs(error[2]) > ERROR:
            angles = np.matmul(self.transposejacobian(theta[0],theta[1]),[error[0],error[1]])
            theta[0] += round_angle(angles[0])
            theta[1] += round_angle(angles[1])
            theta[2] = self.caltheta3(theta[0],theta[1],target[0],target[1])
            theta[3] = self.caltheta4(target[2])
            theta = [round_angle(x) for x in theta]
            error = self.error_pos(self.fk(theta),target)
            count += 1
        joint = [theta[0],round_angle(theta[1]-theta[0]),round_angle(theta[2]-theta[1]),theta[3]]
        end_time = time.monotonic() - time_start
        print("number of tries: %s" %count)
        print("time elapsed: %s" %end_time)
        print("original theta: %s" %[round(np.rad2deg(item),2) for item in theta])
        print("original joint: %s" %[round(np.rad2deg(item),2) for item in joint])
        print("original pos: %s" %([round(item,2) for item in self.fk(theta)]))
        print("APPLY JOINT LIMIT")
        if (joint[0] > np.pi/2 and joint[0] < np.pi*3/2):
            if abs(joint[0] - np.pi/2) < abs(joint[0] - np.pi*3/2):
                print("joint 1 is over limit and closer to 90 degrees")
                joint[0] = theta[0] = np.pi/2
            else:
                print("joint 1 is over limit and closer to 270 degrees")
                joint[0] = theta[0] = np.pi*3/2
        if (joint[1] > np.pi/2 and joint[1] < np.pi*3/2):
            if abs(joint[1] - np.pi/2) < abs(joint[1] - np.pi*3/2):
                print("joint 2 is over limit and closer to 90 degrees")
                joint[1] = np.pi/2
                theta[1] = round_angle(np.pi/2+theta[0])
            else:
                print("joint 2 is over limit and closer to 270 degrees")
                joint[1] = np.pi*3/2
                theta[1] = round_angle(np.pi*3/2+theta[0])
        if (joint[2] > np.pi/2 and joint[2] < np.pi*3/2):
            if abs(joint[2] - np.pi/2) < abs(joint[2] - np.pi*3/2):
                print("joint 3 is over limit and closer to 90 degrees")
                joint[2] = np.pi/2
                theta[2] = round_angle(np.pi*3/2+theta[1])
            else:
                print("joint 3 is over limit and closer to 270 degrees")
                joint[2] = np.pi*3/2
                theta[2] = round_angle(np.pi*3/2+theta[1])
        print("final theta: %s" %[round(np.rad2deg(item),2) for item in theta])
        print("final joint: %s" %[round(np.rad2deg(item),2) for item in joint])
        print("final pos: %s" %([round(item,2) for item in self.fk(theta)]))
        return theta

class MyApp(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.title("Arm")
        self.w = tk.Canvas(self, width=320, height=320, bg="white")
        self.w.pack()

        self.chain = Kinematics(30,30,30,30)
        chain = self.chain
        x = float(input("X: "))
        y = float(input("Y: "))
        z = float(input("Z: "))
        target = [x,y,z]
        current_angle = [0,0,0,0]
        self.theta = chain.ik(current_angle,target)
        theta = self.theta

        x0 = y0 = 125
        x1 = x0 + chain.length[0]*np.cos(theta[0])
        x2 = x1 + chain.length[1]*np.cos(theta[1])
        x3 = x2 + chain.length[2]*np.cos(theta[2])
        x4 = x3 + chain.length[3]*np.cos(theta[3])*np.cos(theta[2])
        y1 = y0 + chain.length[0]*np.sin(theta[0])
        y2 = y1 + chain.length[1]*np.sin(theta[1])
        y3 = y2 + chain.length[2]*np.sin(theta[2])
        y4 = y3 + chain.length[3]*np.cos(theta[3])*np.sin(theta[2])

        self.w.create_line(10, 10, 50, 10, fill="red", tags="x",width = 4)
        self.w.create_line(10, 10, 10, 50, fill="blue", tags="y",width = 4)
        self.w.create_line(x0, y0, x1, y1, fill="blue", tags="tibia",width = 4)
        self.w.create_line(x1, y1, x2, y2, fill="red", tags="filia",width = 4)
        self.w.create_line(x2, y2, x3, y3, fill="black", tags="wrist",width = 4)
        self.w.create_line(x3, y3, x4, y4, fill="green", tags="hand",width = 4)
        self.w.create_line(x0, y0, x4, y4, fill="yellow", tags="total",width = 4)

    def update_arm(self):
        chain = self.chain
        x = float(input("X: "))
        y = float(input("Y: "))
        z = float(input("Z: "))
        target = [x,y,z]
        current_angle = [0.1,0,0,0]
        self.theta = chain.ik(current_angle,target)
        theta = self.theta
        x0 = y0 = 125
        x1 = x0 + chain.length[0]*np.cos(theta[0])
        x2 = x1 + chain.length[1]*np.cos(theta[1])
        x3 = x2 + chain.length[2]*np.cos(theta[2])
        x4 = x3 + chain.length[3]*np.cos(theta[3])*np.cos(theta[2])
        y1 = y0 + chain.length[0]*np.sin(theta[0])
        y2 = y1 + chain.length[1]*np.sin(theta[1])
        y3 = y2 + chain.length[2]*np.sin(theta[2])
        y4 = y3 + chain.length[3]*np.cos(theta[3])*np.sin(theta[2])
        self.w.coords("tibia", (x0, y0, x1, y1))
        self.w.coords("filia", (x1, y1, x2, y2))
        self.w.coords("wrist", (x2, y2, x3, y3))
        self.w.coords("hand", (x3, y3, x4, y4))
        self.w.coords("total", (x0, y0, x4, y4))

        self.after(2000, self.update_arm)

app = MyApp()
app.mainloop()