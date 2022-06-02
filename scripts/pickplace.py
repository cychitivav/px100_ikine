import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
import numpy as np
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from sensor_msgs.msg import JointState

L1 = 0.08945  # m
L2 = 0.1000  # m
L3 = 0.1000  # m
L4 = 0.119  # m
Lm = 0.0350  # m


def ikine(T, **kwargs):
    """ IKINE Returns the position of the joints to reach the desired pose T of 
    the TCP

        q = ikine(T) are the joint coordinates (1x4) corresponding to the  
        pincher x100 robot end-effector pose T (4x4) which is a homogeneous 
        transform.

        q = ikine(...,OPTION,Value) S

        OPTIONS:
            'lenghts', L: Change the robot arm lenghts with a  L, a vector of
                          1x5 [L1 L2 L3 L4 Lm].
            'elbow', E: Change the return configuration between elbow up or elbow 
                        down. Can be specified as 'up' or 'down'."""

    L1 = 0.08945  # m
    L2 = 0.1000  # m
    L3 = 0.1000  # m
    L4 = 0.1190  # m (TCP)
    Lm = 0.035  # m

    up = True

    # if nargin > 1
    #     if nargin ~= 3 && nargin ~= 5
    #         error('Bad number of input arguments.')
    #     end

    #     if any(strcmp(varargin,'elbow'))
    #         idx = find(strcmp(varargin,'elbow'));

    #         if strcmp(varargin{idx+1},'up')
    #             up=true;
    #         elseif strcmp(varargin{idx+1},'down')
    #             up=false;
    #         else
    #             error("Bad configuration. Possible options 'up' or 'down'")
    #         end
    #     elseif any(strcmp(varargin,'lengths'))
    #         idx = find(strcmp(varargin,'lengths'));
    #         L = varargin{idx+1};

    #         L1 = L(1);
    #         L2 = L(2);
    #         L3 = L(3);
    #         L4 = L(4);
    #         Lm = L(5);
    #     else
    #         error(varargin{1} + 'unknowm.')
    #     end
    # end

    q = np.zeros([1, 4])[0]

    # q1 (Waist)
    q[0] = np.arctan2(T[1, 3], T[0, 3])

    # Wrist decoupling
    a = T[0:3, 2]
    w = T[0:3, 3]-L4*a

    # 2R mechanism
    r = np.sqrt(w[0]**2+w[1]**2)
    h = w[2]-L1

    c = np.sqrt(r**2+h**2)

    beta = np.arctan2(Lm, L2)
    psi = np.pi/2-beta
    Lr = np.sqrt(Lm**2+L2**2)

    phi = np.arccos((c**2-L3**2-Lr**2)/(-2*Lr*L3))

    gamma = np.arctan2(h, r)
    alpha = np.arccos((L3**2-Lr**2-c**2)/(-2*Lr*c))

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


PX = rtb.DHRobot(
    [rtb.DHLink(d=L1, a=0, alpha=-np.pi/2, offset=0),
     rtb.DHLink(d=0, a=np.sqrt(L2**2+Lm**2),
                alpha=0, offset=-np.arctan2(L2, Lm)),
     rtb.DHLink(d=0, a=L3, alpha=0, offset=np.arctan2(L2, Lm)),
     rtb.DHLink(d=0, a=L4, alpha=0, offset=0)],
    name="Filoberta",
    tool=SE3(trotx(-np.pi/2)@troty(np.pi/2))
)

print(PX)

Thome = PX.fkine([0, 0, 0, 0])
Tp1 = SE3(transl([0, -0.1, 0.015])@troty(np.pi)@trotz(-np.pi/2))
Tp2 = SE3(transl([0, 0.1, 0.015])@troty(np.pi)@trotz(np.pi/2))
Tpg = SE3(transl([0.15, 0, 0.030])@trotx(np.pi))


hsz = 0.085  # m Height safe zone
qhome = ikine(Thome.A)

Ttemp = Tp1.A.copy()
Ttemp[2, 3] = hsz
q1h = ikine(Ttemp)
q1 = ikine(Tp1.A)

Ttemp = Tp2.A.copy()
Ttemp[2, 3] = hsz
q2h = ikine(Ttemp)
q2 = ikine(Tp2.A)

Ttemp = Tpg.A.copy()
Ttemp[2, 3] = hsz
qgh = ikine(Ttemp)
qg = ikine(Tpg.A)

nh = 10
na = 10

qs = rtb.jtraj(qhome, q1h, na)
qs2 = rtb.jtraj(q1h, q1, nh)
qs3 = rtb.jtraj(q1, q1h, nh)
qs4 = rtb.jtraj(q1h, qgh, na)
qs5 = rtb.jtraj(qgh, qg, nh)
qs6 = rtb.jtraj(qg, qgh, nh)
qs7 = rtb.jtraj(qgh, q2h, na)
qs8 = rtb.jtraj(q2h, q2, nh)
qs9 = rtb.jtraj(q2, q2h, nh)
qs10 = rtb.jtraj(q2h, qgh, na)
qs11 = rtb.jtraj(qgh, qg, nh)
qs12 = rtb.jtraj(qg, qgh, nh)
qs13 = rtb.jtraj(qgh, qhome, na)

# trplot(Tp1)
# trplot(Tp2)
# trplot(Tpg)


Qs = np.vstack((qs.q, qs2.q, qs3.q, qs4.q, qs5.q, qs6.q, qs7.q,
               qs8.q, qs9.q, qs10.q, qs11.q, qs12.q, qs13.q))

print(Qs.shape)
print(Qs)

# PX.plot(Qs)


# Function to send commands to the dynamixel with the dynamixel_command service
def jointCommand(id_num, addr_name, value):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command('', id_num, addr_name, value)
        return result.comm_result
    except rospy.ServiceException as e:
        rospy.logwarn(str(e))


def rad2bin(angle):  # Function to convert degrees to binary (0-4095)
    return round(angle/(2*np.pi)*4095+2048)


for n, q in enumerate(Qs):
    for i, angle in enumerate(q):
        jointCommand(i+1, 'Goal_Position', rad2bin(angle))
    if n == na+nh-1:
        jointCommand(5, 'Goal_Position', 2350)
    if n == 3*na+5*nh-1:
        jointCommand(5, 'Goal_Position', 2100)
    if n == 2*na+3*nh-1 or n == 4*na+7*nh-1:
        jointCommand(5, 'Goal_Position', 2600)
