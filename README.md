# PincherX 100 inverse kinematics and simple path planning

This repository contains the development of the inverse kinematics of the pincher x100 robot[^px100], as well as a simple trajectory to perform a pick and place task.

### Requirements

- ROS Noetic
- ROS toolbox for MATLAB
- Robotics toolbox for MATLAB [^rvc]
- [Type 1](https://drive.google.com/file/d/14Xp9oiURIpeZDVdC0CXu595G7Bunqgh5/view) and [type 2](https://drive.google.com/file/d/1FriMnXi1eF2S0NBVbNXHXbg8Je8QTHQi/view) parts
- Pynput library for python (`pip install pynput` or `pip3 install pynput`)
- PincherX 100 Robot arm [^px100]
- [Pincher X100 description package](https://github.com/cychitivav/px100_description)

## Inverse kinematics

To find the inverse kinematics of the pincher x100 a geometrical approach with a wrist decoupling is used, the following diagram is used for this purpose:

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/170734625-8a467f70-3a1b-4563-915e-150ae77759cf.svg#gh-light-mode-only" alt="px100_ik_white" width="600px" >
    <img src="https://user-images.githubusercontent.com/30636259/170617707-bd4c1035-94aa-473a-a24e-09973a84d942.svg#gh-dark-mode-only" alt="px100_ik_black" width="600px" >
</p>

> **Note**: For a more detailed view of the location of the frames and the dimensions of the robot see the [README](https://github.com/cychitivav/px100_description) file of the px100_description package.

### First joint (Waist)

It is trivial to see that being a planar mechanism the value of $q_1$ is:

$$
\begin{gather*}
    q_1=atan2(y_T,x_T)
\end{gather*}
$$

where $x_T$ and $y_T$ are the coordinates of the target TCP.

### Wrist decoupling

In order to reduce the number of variables, the wrist is decoupled from the last joint. Therefore, the position of the TCP is moved a distance of $-L_4$ in the direction of the approach vector and finally, the position of the wrist is:

$$
\begin{align*}
    w&=
    \begin{bmatrix}
        x_T\\
        y_T\\
        z_T
    \end{bmatrix}
    -L_4
    \begin{bmatrix}
        a_x\\
        a_y\\
        a_z
    \end{bmatrix}
\end{align*}
$$

where $a_x$, $a_y$ and $a_z$ are the components of the approach vector in the base link frame, these components can be obtained from the rotation matrix of TCP.

### 2R mechanism

With all these simplifications made, the problem of the inverse kinematics of the robot is reduced to that of a 2R mechanism. This mechanism has two possible solutions, elbow up and elbow down.

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/170742083-2b6b11e9-6990-4abd-8802-836f498bc924.svg#gh-light-mode-only" alt="2r_white" width="100%" >
    <img src="https://user-images.githubusercontent.com/30636259/170741909-dfffa2ba-2f4f-4f3f-b2a9-596a6da834fc.svg#gh-dark-mode-only" alt="2r_black" width="100%" >
</p>

the equations of the 2R mechanism are:

$$
\begin{gather*}
    r = \sqrt{x_w^2+y_w^2}\\
    h = z_w-L_1\\
    \\
    c = \sqrt{r^2+h^2}\\
    \\
    \beta = \arctan2{(L_m,L_2)}\\
    \psi = \frac{\pi}{2}-\beta\\
    L_r = \sqrt{L_m^2+L_2^2}\\
    \\
    \phi = \arccos{\frac{c^2-L_3^2-L_r^2}{-2L_rL_3}}\\
    \\
    \gamma = \arctan2{(h,r)}\\
    \alpha =  \arccos{\frac{L_3^2-L_r^2-c^2}{-2L_rc}}
\end{gather*}
$$

#### Second and third joints (Shoulder and elbow)

To define the elbow up and elbow down solution, the following equations are used:

<div align="center">

|     Joint      |              Elbow up               |              Elbow down               |
| :------------: | :---------------------------------: | :-----------------------------------: |
| $\mathbf{q_2}$ | $\frac{\pi}{2}-\beta-\alpha-\gamma$ | $\frac{\pi}{2}-(\gamma-\alpha+\beta)$ |
| $\mathbf{q_3}$ |           $\pi-\psi-\phi$           |          $-\pi+(\phi-\psi)$           |

</div>

### Wrist Joint

Once the angles of the waist, shoulder and elbow are defined, the wrist is reattached and the angle of the wrist is defined as:

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/170784419-c36347e9-0043-40ba-bd57-056f06728878.svg#gh-light-mode-only" alt="wrist_coupling" width="400px" >
    <img src="https://user-images.githubusercontent.com/30636259/170784264-ad5c0488-e15b-4189-8d5e-55cb0fcb69be.svg#gh-dark-mode-only" alt="wrist_coupling" width="400px" >
</p>

$$
\begin{gather*}
    \theta_a=\arctan2{\left(\sqrt{x_a^2+y_a^2},z_a\right)}\\
    q_4=\theta_a-q_2-q_3-\frac{\pi}{2}
\end{gather*}
$$

where $\theta_a$ is the angle of the approach vector respect to $z_0$ axis.

> **Note**: The development of this equations is done in [ikine funtion](matlab/ikine.m) for matlab or PXrobot [ikine method](scripts/px100_ikine/PXrobot.py) for python.

### Workspace

the workspace can be represented in a plane as shown in the image below, and with this figure we can make a revolution in the $z_0$ axis to obtain a figure similar to a sphere.

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/171591572-baeda6b3-5f88-4bf5-b26d-5ec7674c94f4.png" width="400px">
</p>

### Inverse kinematics solvers in robotics toolbox by Peter Corke

We found six methods for determining inverse kinematics with roboticstoolbox in python, its methods are going to describe one by one:

- `ikine_6s`: inverse kinematics for 6-axis spherical wrist revolute robot, for more information you can make clic [here](https://petercorke.github.io/robotics-toolbox-python/arm_dh.html#roboticstoolbox.robot.DHRobot.DHRobot.ikine_6s).

- `ikine_LM`: This method use the numerical inverse kinematics by Levenberg-Marquadt optimization and return the inverse kinematic solution of the robot, also this method can be used for robots with any number of degrees of freedom. For more information you can make clic [here](https://petercorke.github.io/robotics-toolbox-python/arm_dh.html#roboticstoolbox.robot.DHRobot.DHRobot.ikine_LM).

- `ikine_LMS`: This method return the inverse kinematic solution using numerical inverse kinematics by Levenberg-Marquadt optimization. For more information you can make clic [here](https://petercorke.github.io/robotics-toolbox-python/arm_dh.html#roboticstoolbox.robot.DHRobot.DHRobot.ikine_LMS).

- `ikine_global`: In this moment the method _ikine_global_ is for using SciPy global optimizers. For more information you can make clic [here](https://petercorke.github.io/robotics-toolbox-python/arm_dh.html#roboticstoolbox.robot.DHRobot.DHRobot.ikine_global).
- `ikine_min`: This method allows to find Inverse kinematics by optimization with joint limits.For more information you can make clic [here](https://petercorke.github.io/robotics-toolbox-python/arm_dh.html#roboticstoolbox.robot.DHRobot.DHRobot.ikine_min).

- `ikine_a`: This method use an analytic inverse kinematic solution to return a joint angle vector in radians. For more information you can make clic [here](https://petercorke.github.io/robotics-toolbox-python/arm_dh.html#roboticstoolbox.models.DH.Puma560.ikine_a).

### Analysis

#### Robot degrees of freedom

The robot has 4 DOF, 3 of them are used for position but the measurement of the fourth DOF is the angle that rotates around the open axis.

#### Possible ikine solutions

The phantom x robot has two possible solutions, the first is denominate "elbow up" and the second is called "elbow down". We can see the two configurations in the image of [2R mechanism](#2r-mechanism).

#### Dextrous workspace

The reachable workspace is the volume whereby the end effector is capable of reaching each point within the space in at least one orientation while the dextrous workspace has the end effector capable of reaching all points in all orientations[^ws].

## Pick and place

For the development of a pick and place routine, the following parts were 3D printed:

<div align="center" >

|                                                             Part 1                                                              |                                                             Part 2                                                              |
| :-----------------------------------------------------------------------------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------------------------: |
| <img src="https://user-images.githubusercontent.com/30636259/171552916-f8f9c989-5833-4d76-bfa7-de4d779c29cc.png" width="200px"> | <img src="https://user-images.githubusercontent.com/30636259/171552921-b20710ea-d566-410c-99c1-4f4cb88f67c7.png" width="200px"> |

</div>

Part type 1 is initially located to the right of the robot, and part type 2 to the left. The challenge is to pick piece type 1 and place it in front, then take piece 2 and insert it into piece 1 which is now in front.

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/171556851-b70b97c3-bac8-474d-9f53-304555ab0223.png" alt="pick_and_place" width="400px" >

[^guide]

</p>

For this we developed a code in python, using the robotics toolbox by Peter Corke where initially the robot was created with the `DHrobot` class, and then 3 homogeneous transformation matrices where the robot is going to move, these are:

- $T_{p_1}$: The transformation matrix that locates the TCP in Part 1.
- $T_{p_2}$: The transformation matrix that locates the TCP in Part 2.
- $T_{p_g}$: The transformation matrix that locates the TCP in the goal pose (front of the robot).

One constraint is that the movements must be vertical for ascending and descending, and horizontal for moving, therefore, a safe zone height ($hsz=0.085~m$) was defined where the robot moves without colliding with any object. With the above, 3 other transformation matrices were defined equal to the previous ones but with a change in the height to reach the safe zone.

- $T_{p_1h}$: The transformation matrix that locates the TCP above Part 1 at the height of the safe zone.
- $T_{p_2h}$: The transformation matrix that locates the TCP above Part 2 at the height of the safe zone.
- $T_{p_gh}$: The transformation matrix that locates the TCP above Part 1 at the height of the safe zone.

Once these transformation matrices are defined, the robot [inverse kinematics method](scripts/px100_ikine/PXrobot.py) is used to know the state of the joints corresponding to each matrix, this in order to use the `jtraj` function of the toolbox to interpolate in the joint space, this function has the following syntax:

```python
q = rtb.jtraj(q0, qg, n)
```

Where $q_0$ is the source configuration, $q_g$ is the target configuration and $n$ is the number of points in the interpolation. In our algorithm we use 10 steps for each interpolation, therefore, the complete movement including the start at home and arrival at home has 130 configurations.

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/171560516-5c9d8cf1-2198-480a-aa15-846119d251bc.gif" alt="jtraj" width="500px" >
</p>

1. $T_{home}\to T_{p_1h}$
1. $T_{p_1h}\to T_{p_1}$
1. $T_{p_1}\to T_{p_1h}$
1. $T_{p_1h}\to T_{p_gh}$
1. $T_{p_gh}\to T_{p_g}$
1. $T_{p_g}\to T_{p_gh}$
1. $T_{p_gh}\to T_{p_2h}$
1. $T_{p_2h}\to T_{p_2}$
1. $T_{p_2}\to T_{p_2h}$
1. $T_{p_2h}\to T_{p_gh}$
1. $T_{p_gh}\to T_{p_g}$
1. $T_{p_g}\to T_{p_gh}$
1. $T_{p_gh}\to T_{home}$

Finally, to operate the griper, closing commands are sent after moves 2 and 8 (grasp), and opening commands after moves 5 and 11 (release). This is done with the service `dynamixel_workbench/dynamixel_command`.

## Motion in workspace

Now, an algorithm is performed to move the robot in the workspace. As the pincher x100 is a 4 DOF robot, a pure motion is performed on its 4 axes:

1. Translation in X - `trax`
1. Translation in Y - `tray`
1. Translation in Z - `traz`
1. Rotation about the O axis of TCP - `rot`

These movements are performed from keyboard commands using the _pynput_ library. The available commands are:

- <kbd>w</kbd>: Select next axis.
- <kbd>s</kbd>: Select previous axis.
- <kbd>a</kbd>: Move the establish step in positive direction along the selected axis.
- <kbd>d</kbd>: Move the establish step in negative direction along the selected axis.

To calculate the movements in the workspace, first the step is defined, this is:

- $0.02~m$ for translational motion.
- $0.002~rad$ for rotational motion

Then, the current state of the joints is read using the _/joint_states_ topic and with the `fkine` function of the toolbox it is possible to calculate the TCP transformation matrix. Depending on the axis of motion that is being controlled the transformation matrix goal is calculated, if they are the translational motion axes the step is added (or subtracted in case it moves in positive direction) in the corresponding translation component:

$$
\begin{gather*}
    T_{trax} =
    \begin{bmatrix}
        r_{11} & r_{12} & r_{13} & d_x + step\\
        r_{21} & r_{22} & r_{23} & d_y \\
        r_{31} & r_{32} & r_{33} & d_z \\
        0 & 0 & 0 & 1
    \end{bmatrix}
\end{gather*}
$$

$$
\begin{gather*}
    T_{tray} =
    \begin{bmatrix}
        r_{11} & r_{12} & r_{13} & d_x \\
        r_{21} & r_{22} & r_{23} & d_y + step \\
        r_{31} & r_{32} & r_{33} & d_z \\
        0 & 0 & 0 & 1
    \end{bmatrix}
\end{gather*}
$$

$$
\begin{gather*}
    T_{traz} =
    \begin{bmatrix}
        r_{11} & r_{12} & r_{13} & d_x \\
        r_{21} & r_{22} & r_{23} & d_y \\
        r_{31} & r_{32} & r_{33} & d_z+step \\
        0 & 0 & 0 & 1
    \end{bmatrix}
\end{gather*}
$$

While for the rotation, the transformation matrix is multiplied by a pure rotation on the y-axis.

$$
\begin{gather*}
    T_{rot} =
    \begin{bmatrix}
        r_{11} & r_{12} & r_{13} & d_x \\
        r_{21} & r_{22} & r_{23} & d_y \\
        r_{31} & r_{32} & r_{33} & d_z \\
        0 & 0 & 0 & 1
    \end{bmatrix}
    \cdot  \text{troty}(step)
\end{gather*}
$$

With the origin and goal matrices established, we must interpolate between the two, for this we use the function `ctraj` which interpolates in Cartesian space (unlike `jtraj` which interpolates in joint space). This function receives three parameters just like `jtraj`:

```python
T = rtb.ctraj(T0,Tg,n)
```

the difference is that this one receives the transformation matrices but keeps the $n$ parameter to define the number of intermediate matrices.

Finally, to each of the interpolation matrices we do inverse kinematics to find q and with the `sendJoints` function we send the target position to each motor.

### RViz visualization

in order to view the robot in rviz the same configuration file of the px100_description package is used and the state of the joints are mapped from the topical **/dynamixel_workbench/joint_states** to **/joint_states**.

## Conclusions
The correct elaboration of the inverse kinematics is of utmost importance because in a simulation the robot can cross its own links, but in reality if this happens while using the robot in a physical way it can suffer serious damage. On the other hand, although the inverse kinematics yields the exact joint configuration to achieve a given pose, care must be taken as the quality of the sensors affects the measurement and therefore arrive at an approximate pose but not the real one as was the case in the pick and place task.

## Video
<a href="https://youtu.be/_0eVXJhujo8">YouTube video</a>

https://user-images.githubusercontent.com/30636259/171556641-a57b44b1-683a-42c6-8f7a-a546d67eda81.mp4



## Acknowledgments

- [Felipe Gonzales (Professor)](https://felipeg17.github.io/index.html)
- [Manuel Lugo (Monitor)](https://github.com/mlugom)

## References

[^px100]: https://www.trossenrobotics.com/pincherx-100-robot-arm.aspx
[^rvc]: https://petercorke.com/toolboxes/robotics-toolbox/
[^modules]: http://docs.ros.org/en/lunar/api/catkin/html/howto/format2/installing_python.html
[^guide]: https://drive.google.com/file/d/1tYA0gQ9XO5WdSqxvGoNDuhaGTOsUNHZ_/view?usp=sharing
[^ws]: https://www.sciencedirect.com/topics/engineering/reachable-workspace
