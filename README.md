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

|     Joint      |             Elbow up                |               Elbow down              |
| :------------: | :---------------------------------: | :-----------------------------------: |
| $\mathbf{q_2}$ | $\frac{\pi}{2}-\beta-\alpha-\gamma$ | $\frac{\pi}{2}-(\gamma-\alpha+\beta)$ |
| $\mathbf{q_3}$ |         $\pi-\psi-\phi$             |          $-\pi+(\phi-\psi)$           |

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

### Workspace
### Inverse kinematics solvers in robotics toolbox by Peter Corke
### Analysis
#### Robot degrees of freedom
#### Possible ikine solutions
The phantom x robot has two possible solutions, the first  is denominate "elbow up" and the second  is called "elbow down". We can see the two configurations in the next image: 
#### Espacio diestro
* __Espacio de trabajo alcanzable__: Está compuesto por el conjunto de puntos alcanzables por el manipulador.
* __Espacio de trabajo diestro__: Está compuesto por el conjunto de puntos que el manipulador puede alcanzar con una orientación arbitraria de su efector final.

## Pick and place

## Motion in task space

## RViz visualization

## Conclusions



> __Note__: The development of this equations is done in [ikine](matlab/ikine.m) function.


# References
[^px100]: https://www.trossenrobotics.com/pincherx-100-robot-arm.aspx
[^rvc]: https://petercorke.com/toolboxes/robotics-toolbox/
