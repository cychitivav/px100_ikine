from spatialmath.base import *
import numpy as np
from pynput.keyboard import Key, Listener, KeyCode  # Libraries for keyboard input
import rospy  # ROS libraries
from px100_ikine.PXrobot import PX



class myKeyboard(Listener):
    def __init__(self):
        super().__init__(on_press=self.onPress)
        self.PX = PX()

        # Variables to move robot
        self.motions = ['trax', 'tray', 'traz', 'rot']
        self.currentMotion = 0
        self.step = 0.02
        self.joints = [0, 0, 0, 0]

        # Welcome message and start the listener
        welcome = """\nControls:
        * w: next axis
        * s: previous axis

        * a: positive step
        * d: negative step"""
        rospy.loginfo(welcome)  # Show welcome message

        rospy.logwarn('You are moving the '+self.motions[self.currentMotion])

        self.start()
        while not rospy.is_shutdown():  # Loop to keep the listener running
            pass
        self.stop()

    def onPress(self, key):
        print("\033[A")  # Print especial character to erase key pressed
        if not rospy.is_shutdown():
            if key == KeyCode.from_char('w'): # Next axis
                if self.currentMotion == 3:
                    self.currentMotion = 0
                else:
                    self.currentMotion += 1

                rospy.logwarn('You are moving the ' +
                              self.motions[self.currentMotion])

            if key == KeyCode.from_char('s'): # Previous axis
                if self.currentMotion == 0:
                    self.currentMotion = 3
                else:
                    self.currentMotion -= 1
                rospy.logwarn('You are moving the ' +
                              self.motions[self.currentMotion])

            if key == KeyCode.from_char('a'): # Positive step
                T = self.calcT(self.motions[self.currentMotion], -self.step)
                q = self.PX.ikine(T)

                # self.sendJoins(q)

            if key == KeyCode.from_char('d'): # Negative step
                T = self.calcT(self.motions[self.currentMotion], self.step)
                q = self.PX.ikine(T)

                # self.sendJoins(q)

    def sendJoins(self, q):
        """ SENDJOINTS Send the desired joint configuration to robot through the 
            dynamixel_command service

            sendJoints(q) receive the joint states vector q (1x4) corresponding 
            to the pincher x100 robot joints."""

        self.PX.jointCommand(1, 'Goal_Position', self.rad2bin(q[0]))
        self.PX.jointCommand(2, 'Goal_Position', self.rad2bin(q[1]))
        self.PX.jointCommand(3, 'Goal_Position', self.rad2bin(q[2]))
        self.PX.jointCommand(4, 'Goal_Position', self.rad2bin(q[3]))

    def calcT(self, axis, step):
        T = self.PX.fkine(self.joints).A

        if axis == 'trax':
            T[0, 3] = T[0, 3] + step
        elif axis == 'tray':
            T[1, 3] = T[1, 3] + step
        elif axis == 'traz':
            T[2, 3] = T[2, 3] + step
        elif axis == 'rot':
            T[0:3, 0:3] = T[0:3, 0:3] @ roty(step/10)
        else:
            rospy.logerror('Invalid axis')

        return T

if __name__ == "__main__":
    # Initialize the node and instantiate the class myKeyboard
    rospy.init_node('TeleopKey', anonymous=False)

    myKeyboard()
