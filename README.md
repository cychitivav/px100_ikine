# PincherX 100 inverse kinematics and simple path planning 
This repository contains the development of the inverse kinematics of the pincher x100 robot[^px100], as well as a simple trajectory to perform a pick and place task.

### Requirements
* ROS Noetic
* ROS toolbox for MATLAB
* Robotics toolbox for MATLAB [^rvc]
* [Type 1](https://drive.google.com/file/d/14Xp9oiURIpeZDVdC0CXu595G7Bunqgh5/view) and [type 2](https://drive.google.com/file/d/1FriMnXi1eF2S0NBVbNXHXbg8Je8QTHQi/view) parts
* Pynput library for python (`pip install pynput` or `pip3 install pynput`)
* PincherX 100 Robot arm [^px100]
* [Pincher X100 description package](https://github.com/cychitivav/px100_description)

## Inverse kinematics
To find the inverse kinematics of the pincher x100 a geometrical approach with a wrist decoupling is used, the following diagram is used for this purpose:

<p align="center">
    <img src="https://user-images.githubusercontent.com/30636259/170734625-8a467f70-3a1b-4563-915e-150ae77759cf.svg#gh-light-mode-only" alt="px100_ik_white" width="600px" >
    <img src="https://user-images.githubusercontent.com/30636259/170617707-bd4c1035-94aa-473a-a24e-09973a84d942.svg#gh-dark-mode-only" alt="px100_ik_black" width="600px" >
</p>

> __Note__: For a more detailed view of the location of the frames and the dimensions of the robot see the [README](https://github.com/cychitivav/px100_description) file of the px100_description package.

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
    <img src="https://user-images.githubusercontent.com/30636259/170735251-66cb82c2-ebd4-4d85-a0af-1e61a278ce6c.svg#gh-light-mode-only" alt="2r_white" width="100%" >
    <img src="https://user-images.githubusercontent.com/30636259/170735526-51a6dcb6-269d-4443-9556-7a7a2073c52a.svg#gh-dark-mode-only" alt="2r_black" width="100%" >
</p>

the equations of the 2R mechanism are:

$$
\begin{gather*}
    r = \sqrt{x_w^2+y_w^2}\\
    h = z_w-L_1\\
    \\
    hyp = \sqrt{r^2+h^2}\\
    \\
    \beta = \arctan2{(L_m,L_2)}\\
    \psi = \frac{pi}{2}-\beta\\
    Lr = sqrt(Lm^2+L2^2)\\
    \\
    \phi = \arccos{\frac{hyp^2-L_3^2-L_r^2}{-2L_rL_3}}\\
    \\
    \gamma = \arctan2{(h,r)}\\
    \alpha =  \arccos{\frac{L_3^2-L_r^2-hyp^2}{-2L_rhyp}}
\end{gather*}
$$

#### Second and third joints (Shoulder and elbow)
To define the elbow up and elbow down solution, the following equations are used:

<div align="center">

| Joint |             Elbow up                |               Elbow down              |
| :---: | :---------------------------------: | :-----------------------------------: |
| $q_2$ | $\frac{\pi}{2}-\beta-\alpha-\gamma$ | $\frac{\pi}{2}-(\gamma-\alpha+\beta)$ |
| $q_3$ |         $\pi-\psi-\phi$             |          $-\pi+(\phi-\psi)$           |

</div>

### Wrist Joint

$$
q_4=



> __Note__: The development of this equations is done in [ikine function](matlab/ikine.m).


# References
[^px100]: https://www.trossenrobotics.com/pincherx-100-robot-arm.aspx
[^rvc]: https://petercorke.com/toolboxes/robotics-toolbox/
