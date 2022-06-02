import roboticstoolbox as rtb  # Robotics and numerical libraries
from spatialmath import *
from spatialmath.base import *
import numpy as np
from pynput.keyboard import Key, Listener, KeyCode  # Libraries for keyboard input
import rospy  # ROS libraries
from dynamixel_workbench_msgs.srv import DynamixelCommand
from sensor_msgs.msg import JointState


class myKeyboard(rtb.DHRobot):
    def __init__(self, L1=0.08945, L2=0.1, L3=0.1, L4=0.119, Lm=0.035):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4
        self.Lm = Lm

        # Create robot
        super().__init__(
            [rtb.DHLink(d=L1,   a=0,        alpha=-np.pi/2, offset=0),
             rtb.DHLink(d=0,    a=np.sqrt(L2**2+self.Lm**2),
                        alpha=0,        offset=-np.arctan2(self.L2, self.Lm)),
             rtb.DHLink(d=0,    a=self.L3,  alpha=0,
                        offset=np.arctan2(self.L2, self.Lm)),
             rtb.DHLink(d=0,    a=self.L4,  alpha=0,        offset=0)],
            name="Filoberta",
            tool=SE3(trotx(-np.pi/2)@troty(np.pi/2))
        )

        # Variables to move robot
        self.motions = ['trax', 'tray', 'traz', 'rot']
        self.currentMotion = 0
        self.step = 0.02
        self.joints = [0, 0, 0, 0]

        # ROS node to publish joint states if run_dynamixel is False
        self.sub = rospy.Subscriber(
            '/joint_states', JointState, self.updateJoints)

        # Welcome message and start the listener
        welcome = """\nControls:
        * w: next axis
        * s: previous axis

        * a: positive step
        * d: negative step"""
        rospy.logwarn('You are moving the '+self.motions[self.currentMotion])

        rospy.loginfo(welcome)  # Show welcome message

        listener = Listener(on_press=self.onPress)
        listener.start()
        while not rospy.is_shutdown():  # Loop to keep the listener running
            pass
        listener.stop()

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
                q = self.ikine(T)

                self.sendJoins(q)

            if key == KeyCode.from_char('d'): # Negative step
                T = self.calcT(self.motions[self.currentMotion], self.step)
                q = self.ikine(T)

                self.sendJoins(q)

    def sendJoins(self, q):
        """ SENDJOINTS Send the desired joint configuration to robot through the 
            dynamixel_command service

            sendJoints(q) receive the joint states vector q (1x4) corresponding 
            to the pincher x100 robot joints."""
        self.jointCommand(1, 'Goal_Position', self.rad2bin(q[0]))
        self.jointCommand(2, 'Goal_Position', self.rad2bin(q[1]))
        self.jointCommand(3, 'Goal_Position', self.rad2bin(q[2]))
        self.jointCommand(4, 'Goal_Position', self.rad2bin(q[3]))

    def calcT(self, axis, step):
        T = self.fkine(self.joints).A

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

    # Function to send commands to the dynamixel with the dynamixel_command service
    def jointCommand(self, id_num, addr_name, value):
        rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
        try:
            dynamixel_command = rospy.ServiceProxy(
                '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            result = dynamixel_command('', id_num, addr_name, value)
            return result.comm_result
        except rospy.ServiceException as e:
            rospy.logwarn(str(e))

    def rad2bin(self, angle):  # Function to convert radians to binary (0-4095)
        return round(angle/(2*np.pi)*4095+2048)

    def updateJoints(self, msg):
        names = msg.name
        unorderedJoints = msg.position

        # Order joints in the same order as the motors
        self.joints[0] = unorderedJoints[names.index('waist')]
        self.joints[1] = unorderedJoints[names.index('shoulder')]
        self.joints[2] = unorderedJoints[names.index('elbow')]
        self.joints[3] = unorderedJoints[names.index('wrist')]

    def ikine(self, T, **kwargs):
        """ IKINE Returns the position of the joints to reach the desired pose T of 
        the TCP

            q = ikine(T) are the joint coordinates (1x4) corresponding to the  
            pincher x100 robot end-effector pose T (4x4) which is a homogeneous 
            transform.

            q = ikine(...,OPTION,Value) S

            OPTIONS:
                'elbow', E: Change the return configuration between elbow up or elbow 
                            down. Can be specified as 'up' or 'down'."""

        up = True

        if len(kwargs) > 1:
            if 'elbow' in kwargs:
                if kwargs['elbow'] == 'up':
                    up = True
                elif kwargs['elbow'] == 'down':
                    up = False
                else:
                    rospy.logerror('Invalid elbow option. Use "up" or "down"')
            else:
                rospy.logerror(kwargs.keys()[0] + 'option unknown')
        else:
            rospy.logerror('Invalid number of arguments')


        q = np.zeros([1, 4])[0]

        # q1 (Waist)
        q[0] = np.arctan2(T[1, 3], T[0, 3])

        # Wrist decoupling
        a = T[0:3, 2]
        w = T[0:3, 3]-self.L4*a

        # 2R mechanism
        r = np.sqrt(w[0]**2+w[1]**2)
        h = w[2]-self.L1

        c = np.sqrt(r**2+h**2)

        beta = np.arctan2(self.Lm, self.L2)
        psi = np.pi/2-beta
        Lr = np.sqrt(self.Lm**2+self.L2**2)

        phi = np.arccos((c**2-self.L3**2-Lr**2)/(-2*Lr*self.L3))

        gamma = np.arctan2(h, r)
        alpha = np.arccos((self.L3**2-Lr**2-c**2)/(-2*Lr*c))

        # q2 (Shoulder)
        if up:
            q[1] = np.pi/2-beta-alpha-gamma
        else:
            q[1] = np.pi/2-(gamma-alpha+beta)

        # q3 (Elbow)
        if up:
            q[2] = np.pi - psi-phi
        else:
            q[2] = -np.pi+(phi-psi)

        # q4 (Wrist)
        angA = np.arctan2(np.sqrt(T[1, 2]**2+T[0, 2]**2), T[2, 2])
        q[3] = angA-q[1]-np.pi/2-q[2]

        return q


if __name__ == "__main__":
    # Initialize the node and instantiate the class myKeyboard
    rospy.init_node('TeleopKey', anonymous=False)

    myKeyboard()
