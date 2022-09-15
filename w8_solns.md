---
marp: true
theme: uncover
style: |
    :root {
    --color-background: #FFFFFF;
    --color-foreground: #8B2781;
    --color-highlight: #8B2781;
    --color-dimmed: #888888;
    }
    h1 {
      color: #51247A
    }
    h2 {
      color: #8B2781
    }
    h3 {
      color: #220033
    }
    section {
      font-size: 30px;
    }
paginate: true

---

# METR4202
## Robotics & Automation
### Week 8: [TUT] - Dynamics
#### (Solutions)

---

## What are we covering for this tutorial?
- Dynamics
  - Determining the Equations of Motion
    - Euler-Lagrange Approach
    - Newton-Euler Approach


---

# Euler-Lagrange
## RP Robot Example

For Euler-Lagrange, we need to calculate the kinetic ($\mathcal{K}$) energy and potential ($\mathcal{P}$) energy, with respect to the states, $\theta_{1}$, $\theta_{2}$, and their time derivatives $\dot{\theta}_{1}$, $\dot{\theta}_{2}$
This gives us the Lagrangian ($\mathcal{L}$)
$$
\mathcal{L}(\theta, \dot{\theta}) = \mathcal{K}(\theta, \dot{\theta}) - \mathcal{P}(\theta)
$$
We can also calculate the total energy, called the Hamiltonian ($
\mathcal{H}$)
$$
\mathcal{H}(\theta, \dot{\theta}) = \mathcal{K}(\theta, \dot{\theta}) + \mathcal{P}(\theta)
$$

---

# Potential Energy

For any rigid-body, we can calculate the potential energy from the centre of mass. We know that the vector field $\mathfrak{g}$, is proportional to the negative gradient of the potiential energy $\mathcal{P}$.
$$
\begin{aligned}
m\mathfrak{g}=-\nabla \mathcal{P}
\end{aligned}
$$

It's fair to assume that gravity is constant, which means that we can calculate the potiential as:

$$
\begin{aligned}
\mathcal{P} &= -\mathfrak{m}\mathbf{x}^{\top} \mathfrak{g} \\
&= -\mathfrak{m}\begin{bmatrix}x & y\end{bmatrix}\begin{bmatrix}0 \\ -g\end{bmatrix} \\
& = \mathfrak{m}gy
\end{aligned}
$$

---

# Kinetic Energy

For any rigid-body, we can calculate the kinetic energy by looking at the velocity and angular velocity about the centre of mass.

$$
\mathcal{K} = \frac{1}{2}\begin{bmatrix} \dot{\mathbf{x}}^{\top} \\ \dot{\boldsymbol{\omega}}^{\top}\end{bmatrix} \begin{bmatrix}mI & 0 \\ 0 & \mathcal{I}\end{bmatrix}\begin{bmatrix} \dot{\mathbf{x}} & \dot{\boldsymbol{\omega}}\end{bmatrix},\quad \dot{\mathbf{x}},\boldsymbol{\omega}\in\R^{3},\quad \mathcal{I}\in\R^{3\times 3}
$$
In the case of a planar mechanism, this reduces to:
$$
\mathcal{K}=\frac{1}{2}m\left(\dot{x}^{2}+\dot{y}^{2}\right)+\frac{1}{2}\mathcal{I}\omega^{2}
$$

---

## RP - Position + Velocity
### Question 1a)

First we need to calculate the positions of the frames.
$$
\begin{aligned}
x_{1} &= L_{1}\cos{\theta_{1}} \\
x_{2} &= \theta_{2}\cos{\theta_{1}} \\
y_{1} &= L_{1}\sin{\theta_{1}} \\
y_{2} &= \theta_{2} \sin{\theta_{1}} \\
\end{aligned}
$$
---

We can take the derivatives of these to get the frame velocities.
$$
\begin{aligned}
\dot{x}_{1} &= -L_{1}\dot{\theta}_{1}\sin{\theta_{1}} \\
\dot{x}_{2} &= \dot{\theta}_{2}\cos{\theta_{1}}-\theta_{2}\dot{\theta}_{1}\sin{\theta_{1}} \\
\dot{y}_{1} &= L_{1}\dot{\theta}_{1}\cos{\theta_{1}} \\
\dot{y}_{2} &= \dot{\theta}_{2}\sin{\theta_{1}} + \theta_{2}\dot{\theta}_{1}\cos{\theta_{1}} \\
\end{aligned}
$$
We can also work out the 
$$
\begin{aligned}
\omega_{1} &= \dot{\theta}_{1} \\
\omega_{2} &= \dot{\theta}_{1}
\end{aligned}
$$

---

## RP - Potential Energy
### Question 1b)

For our case, we have

$$
\begin{aligned}
\mathcal{P}_{1} &= \mathfrak{m}_{1}gy_{1} \\
&= \mathfrak{m}_{1}gL_{1}\sin{\theta_{1}} \\
\mathcal{P}_{2} &= \mathfrak{m}_{2}gy_{2} \\
&= \mathfrak{m}_{2}g\theta_{2}\sin{\theta_{1}} \\
\mathcal{P} &= \mathcal{P}_{1} + \mathcal{P}_{2} \\
&= \left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\sin{\theta_{1}}
\end{aligned}
$$

---

## RP - Kinetic Energy
### Question 1c)

Then we can calculate the kinetic energy of each rigid-body.

$$
\begin{aligned}
\mathcal{K}_{1} &= \frac{1}{2}m_{1}\left(\dot{x}_{1}^{2}+\dot{y}_{1}^{2}\right)+\frac{1}{2}\mathcal{I}_{1}\omega_{1}^{2} \\
\mathcal{K}_{2} &= \frac{1}{2}m_{2}\left(\dot{x}_{2}^{2}+\dot{y}_{2}^{2}\right)+\frac{1}{2}\mathcal{I}_{2}\omega_{2}^{2}
\end{aligned}
$$

---

## Link 1:
$$
\begin{aligned}
\mathcal{K}_{1} &= \frac{1}{2}m_{1}\left(\dot{x}_{1}^{2}+\dot{y}_{1}^{2}\right)+\frac{1}{2}\mathcal{I}_{1}\omega_{1}^{2} \\
&= \frac{1}{2}\mathfrak{m}_{1}\left(L_{1}^{2}\dot{\theta}_{1}^{2}\sin^{2}{\theta_{1}}+L_{1}^{2}\dot{\theta}_{1}^{2}\cos^{2}{\theta_{1}}\right)+\frac{1}{2}\mathcal{I}_{1}\dot{\theta}_{1}^{2} \\
&= \frac{1}{2}\left(\mathfrak{m}_{1}L_{1}^{2}+\mathcal{I}_{1}\right)\dot{\theta}_{1}^{2}
\end{aligned}
$$

---

## Link 2:
$$
\begin{aligned}
\mathcal{K}_{2} &= \frac{1}{2}\mathfrak{m}_{2}\left(\dot{x}_{2}^{2}+\dot{y}_{2}^{2}\right)+\frac{1}{2}\mathcal{I}_{2}\omega_{2}^{2} \\
\dot{x}_{2}^{2} &= \left(\dot{\theta}_{2}\cos{\theta_{1}}-\theta_{2}\dot{\theta}_{1}\sin{\theta_{1}}\right)^{2} \\

&= \dot{\theta}_{2}^{2}\cos^{2}{\theta_{1}}-\dot{\theta}_{1}\dot{\theta}_{2}\theta_{2}\cos{\theta_{1}}\sin{\theta_{1}} + \theta_{2}^{2}\dot{\theta}_{1}^{2}\sin^{2}{\theta_{1}} \\

\dot{y}_{2}^{2} &= \left(\dot{\theta}_{2}\sin{\theta_{1}}+\theta_{2}\dot{\theta}_{1}\cos{\theta_{1}}\right)^{2} \\

&= \dot{\theta}_{2}^{2}\sin^{2}{\theta_{1}}+\dot{\theta}_{1}\dot{\theta}_{2}\theta_{2}\cos{\theta_{1}}\sin{\theta_{1}} + \theta_{2}^{2}\dot{\theta}_{1}^{2}\cos^{2}{\theta_{1}} \\
\mathcal{K}_{2} &= \frac{1}{2}\mathfrak{m}_{2}\left(\dot{\theta}_{2}^{2}+\theta_{2}^{2}\dot{\theta}_{1}^{2}\right)+\frac{1}{2}\mathcal{I}_{2}\dot{\theta}_{1}^{2} \\
 &= \frac{1}{2}\mathfrak{m}_{2}\dot{\theta}_{2}^{2}+\frac{1}{2}\left(\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{2}\right)\dot{\theta}_{1}^{2}
\end{aligned}
$$


---

$$
\begin{aligned}
\mathcal{K} &= \mathcal{K}_{1} + \mathcal{K}_{2} \\
&= \frac{1}{2}\left(\mathfrak{m}_{1}L_{1}^{2}+\mathcal{I}_{1}\right)\dot{\theta}_{1}^{2} + \frac{1}{2}\mathfrak{m}_{2}\dot{\theta}_{2}^{2}+\frac{1}{2}\left(\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{2}\right)\dot{\theta}_{1}^{2} \\
&=\frac{1}{2}\mathfrak{m}_{2}\dot{\theta}_{2}^{2} + \frac{1}{2}\left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\dot{\theta}_{1}^{2}
\end{aligned}
$$

---

## RP: The Lagrangian
### Question 1d)

Putting it all together we have
$$
\begin{aligned}
\mathcal{L} &= \mathcal{K} - \mathcal{P} \\
&= \frac{1}{2}\mathfrak{m}_{2}\dot{\theta}_{2}^{2} + \frac{1}{2}\left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\dot{\theta}_{1}^{2}  -\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\sin{\theta_{1}}
\end{aligned}
$$

---

# Equations of Motion
From the Lagrangian, we can derive the equations of motion:
$$
\begin{aligned}
f_{i} &= \frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial \dot{q}_{i}}\right) - \frac{\partial \mathcal{L}}{\partial q_{i}}
\end{aligned}
$$
$f_{i}$ and $q_{i}$ refer to the generalised 'force' and generalised coordinates, which can apply to any system (not just mechanical).
They are defined such that $f_{i}^{\top}\dot{q}_{i}$ is power.
E.g. $P=\tau\cdot\omega=\tau\cdot \dot{\theta}$, or $P=f\cdot v=f\cdot \dot{p}$
For robots defined in this convention, it becomes:
$$
\begin{aligned}
\tau_{i} &= \frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial \dot{\theta}_{i}}\right) - \frac{\partial \mathcal{L}}{\partial \theta_{i}}
\end{aligned}
$$
$\tau_{i}$ are the joint torques/forces and $
\theta_{i}$ are the joint angles/distances.

---

## RP: Equations of Motion
### Question 1e)
- Let's work out the equations of motion for this system which should be in this form:
$$
\tau = M(\theta)\ddot{\theta} + c(\theta, \dot{\theta}) + g(\theta)
$$
---

## Joint 1:
$$
\begin{aligned}
\frac{\partial \mathcal{L}}{\partial \theta_{1}} &= \frac{\partial}{\partial \theta_{1}}\left(\frac{1}{2}\mathfrak{m}_{2}\dot{\theta}_{2}^{2} + \frac{1}{2}\left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\dot{\theta}_{1}^{2}  -\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\sin{\theta_{1}}\right) \\
&=-\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\cos{\theta_{1}} \\
\frac{\partial \mathcal{L}}{\partial \dot{\theta}_{1}}&=\frac{\partial}{\partial \dot{\theta}_{1}}\left(\frac{1}{2}\mathfrak{m}_{2}\dot{\theta}_{2}^{2} + \frac{1}{2}\left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\dot{\theta}_{1}^{2}  -\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\sin{\theta_{1}}\right) \\
 
 &= \left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\dot{\theta}_{1} \\
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{\theta}_{1}}\right) &= \left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\ddot{\theta}_{1} + 2\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1}\dot{\theta}_{2} \\
\tau_{1} &= \left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\ddot{\theta}_{1} + 2\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1}\dot{\theta}_{2} + \left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\cos{\theta_{1}}
\end{aligned}
$$

---
## Joint 2:
$$
\begin{aligned}
\frac{\partial \mathcal{L}}{\partial \theta_{2}} &= \frac{\partial}{\partial \theta_{2}}\left(\frac{1}{2}\mathfrak{m}_{2}\dot{\theta}_{2}^{2} + \frac{1}{2}\left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\dot{\theta}_{1}^{2}  -\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\sin{\theta_{1}}\right) \\

&= \mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1}^{2} - \mathfrak{m}_{2}g\sin{\theta_{1}} \\

\frac{\partial \mathcal{L}}{\partial \dot{\theta}_{2}}&=\frac{\partial}{\partial \dot{\theta}_{2}}\left(\frac{1}{2}\mathfrak{m}_{2}\dot{\theta}_{2}^{2} + \frac{1}{2}\left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\dot{\theta}_{1}^{2}  -\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\sin{\theta_{1}}\right) \\

 &= \mathfrak{m}_{2}\dot{\theta}_{2} \\

\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{\theta}_{2}}\right) &= \mathfrak{m}_{2}\ddot{\theta}_{2} \\

\tau_{2} &= \mathfrak{m}_{2}\ddot{\theta}_{2} - \mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1}^{2} + \mathfrak{m}_{2}g\sin{\theta_{1}} 
\end{aligned}
$$

---

$$
\begin{cases}
\tau_{1} &= \left(\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}\right)\ddot{\theta}_{1} + 2\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1}\dot{\theta}_{2} + \left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\cos{\theta_{1}} \\
\tau_{2} &= \mathfrak{m}_{2}\ddot{\theta}_{2} - \mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1}^{2} + \mathfrak{m}_{2}g\sin{\theta_{1}}
\end{cases}
$$

We can also write this in matrix-vector form:
$$
\begin{bmatrix}\tau_{1} \\ \tau_{2}\end{bmatrix} = \begin{bmatrix}\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2} & 0 \\ 0 & \mathfrak{m}_{2}\end{bmatrix}\begin{bmatrix}\ddot{\theta}_{1} \\ \ddot{\theta}_{2}\end{bmatrix}+\begin{bmatrix}2 \mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1}\dot{\theta}_{2} \\ -\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{2}^{2}\end{bmatrix} + \begin{bmatrix}\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\cos{\theta_{1}} \\ \mathfrak{m}_{2}g\sin{\theta_{1}} \end{bmatrix}
$$

---
$$
\tau = M(\theta)\ddot{\theta} + c(\theta, \dot{\theta}) + g(\theta)
$$
We have the mass matrix:
$$
M(\theta_{1}, \theta_{2}) = \begin{bmatrix}\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2} & 0 \\ 0 & \mathfrak{m}_{2}\end{bmatrix}
$$
The coriolis + centripetal terms:
$$
c(\theta_{1}, \theta_{2}, \dot{\theta}_{1}, \dot{\theta}_{2}) =
\begin{bmatrix}2 \mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1}\dot{\theta}_{2} \\ -\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{2}^{2}\end{bmatrix}
$$
and the gravity terms:
$$
g(\theta_{1},\theta_{2}) = \begin{bmatrix}\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\cos{\theta_{1}} \\ \mathfrak{m}_{2}g\sin{\theta_{1}} \end{bmatrix}
$$

---

## Properties of the Dynamics Equations
### Mass Matrix
The mass matrix is positive semi-definite and only dependent on the configuration $\theta$.
$$
M(\theta)\succeq 0: \dot{\theta}^{\top}M(\theta)\dot{\theta}\geq 0,\forall\dot{\theta}
$$
Since this is similar to the energy term, this is stating that the kinetic energy is never negative.
Additionally, this matrix must be symmetric.
$$
M(\theta) = M^{\top}(\theta)
$$

---

#### RP Example:
We can see from this that the terms in this are all positive, which means that it is positive
$$
M(\theta_{1}, \theta_{2}) = \begin{bmatrix}\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2} & 0 \\ 0 & \mathfrak{m}_{2}\end{bmatrix}
$$
Also, since it is diagonal (in this case), it is also symmetric.

---

### Coriolis + Centripetal Terms
These can be split into **coriolis** terms which include cross terms between $\dot{\theta}_{i}$ and $\dot{\theta}_{j}$, and the **centripetal** terms, which include quadratic velocity terms, e.g. $\dot{\theta}_{i}^{2}$.
We can also write this out as:
$$
c(\theta, \dot{\theta}) = C(\theta,\dot{\theta})\dot{\theta}
$$
From this, we can see that if the velocity is zero, this term also goes to zero.
$$
\dot{\theta} = 0 \Rightarrow c(\theta, \dot{\theta}) = 0
$$

---

#### RP Example:
$$
\begin{aligned}
c(\theta_{1}, \theta_{2}, \dot{\theta}_{1}, \dot{\theta}_{2}) &=
\begin{bmatrix}2 \mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1}\dot{\theta}_{2} \\ -\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{2}^{2}\end{bmatrix} \\
&=\begin{bmatrix}2\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{2} & 0 \\
0 & -\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{2}\end{bmatrix}\begin{bmatrix}\dot{\theta}_{1} \\ \dot{\theta}_{2}\end{bmatrix} \\
&=\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{2}\begin{bmatrix}2 & 0 \\
0 & -1\end{bmatrix}\begin{bmatrix}\dot{\theta}_{1} \\ \dot{\theta}_{2}\end{bmatrix} \\
&=\begin{bmatrix}0 & 2\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{1} \\
0 & -\mathfrak{m}_{2}\theta_{2}\dot{\theta}_{2}\end{bmatrix}\begin{bmatrix}\dot{\theta}_{1} \\ \dot{\theta}_{2}\end{bmatrix} \\
&=\mathfrak{m}_{2}\theta_{2}\begin{bmatrix}0 & 2\dot{\theta}_{1} \\
0 & -\dot{\theta}_{2}\end{bmatrix}\begin{bmatrix}\dot{\theta}_{1} \\ \dot{\theta}_{2}\end{bmatrix} \\
\end{aligned}
$$
There are multiple ways you can factorise the terms into the $C$ matrix.

---

### Gravity Terms
Since the potiential energy is only dependent on the configuration, so are the gravity terms.
The gravity terms represent the generalised force on the joints due to gravity.

---

#### RP Example

$$
g(\theta_{1},\theta_{2}) = \begin{bmatrix}\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\cos{\theta_{1}} \\ \mathfrak{m}_{2}g\sin{\theta_{1}} \end{bmatrix}
$$

We can interpret the first entry $g_{1}$ as a moment, about the first joint.
The force due to gravity on the two masses is $\mathfrak{m}_{1}g\cos{\theta_{1}}$ and $\mathfrak{m}_{2}g\cos{\theta_{1}}$, which generates the moments at a distance $L_{1}$ and $
\theta_{1}$ respepctively.
This gives a total moment:
$$
g_{1}=\left(\mathfrak{m}_{1}L_{1}+\mathfrak{m}_{2}\theta_{2}\right)g\cos{\theta_{1}}
$$
The second entry is the force along the prismatic joint, given by the dot product of the force due to gravity, and the joint axis.
$$
g_{2}=\begin{bmatrix}0 \\ \mathfrak{m}_{2}g\end{bmatrix}\cdot\begin{bmatrix}\cos{\theta_{1}} \\ \sin{\theta_{1}}\end{bmatrix} = \mathfrak{m}_{2}g\sin{\theta_{1}}
$$

---

## Mass Matrix cont.

What happens when the velocity and gravity is set to zero?
$$
\begin{aligned}
\tau &= M(\theta)\ddot{\theta} + C(\theta, \dot{\theta})\dot{q} + g(\theta) \\
\dot{\theta}=0\Rightarrow \tau &= M(\theta)\ddot{\theta}
\end{aligned}
$$
Now, the mass matrix ($M$) describes how a force/torque on the directly affects the joint acceleration.
I.e. $M_{ij}$ is the inertia of joint $i$ with respect to a torque $j$, and also the inertia of joint $j$ with respect to torque $i$, since it is symmetric.

---

## RP - Mass Matrix cont.
### Question 1f)
Consider the mass matrix from the example:
$$
M(\theta_{1}, \theta_{2}) = \begin{bmatrix}\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2} & 0 \\ 0 & \mathfrak{m}_{2}\end{bmatrix}
$$
At rest, the inertia about joint 1 and torque 1, is given by
$$
  M_{11}(\theta_{1}, \theta_{2}) = \mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}
$$
This is the sum of the inertias $\mathcal{I}_{1}$ and $\mathcal{I}_{2}$, and the inertias due to the masses $\mathfrak{m}_{1}$ at a constant distance $L_{1}$, and $\mathfrak{m}_{2}$ at a distance $\theta_{2}$.
When $\theta_{2}$ increases, the inertia increases, requiring more torque to produce an acceleration.

---

# End-effector Mass Matrix
While the mass matrix $M$ describes the relationship between joint torques and joint accelerations, we can also describe the end-effector effective inertia, using the mass matrix $M(\theta)$ and the jacobian inerverse $J^{-1}(\theta)$, both of which are configuration dependent.
$$
  \Lambda(\theta)=(J^{-1})^{\top}M(J^{-1})
$$

---
If we have some cartesian coordinates $u_{i}$, where
$$
u_{1}=x,\ u_{2}=y, 
$$
$$
J_{ij}(\theta) = \frac{\partial u_{i}}{\partial \theta_{j}}(\theta)
$$
$$
\dot{u}_{i}=\sum_{j}J_{ij} \dot{\theta}_{j}
$$
Note that $u_{1}=x_{n}$ and $u_{2}=y_{n}$, for the end-effector.

---

## RP: End effector Mass Matrix
### Question 1g)
We want to get this:
$$
\begin{aligned}
\dot{x}_{2} &= \dot{\theta}_{2}\cos{\theta_{1}}-\theta_{2}\dot{\theta}_{1}\sin{\theta_{1}} \\
\dot{y}_{2} &= \dot{\theta}_{2}\sin{\theta_{1}} + \theta_{2}\dot{\theta}_{1}\cos{\theta_{1}} \\
\end{aligned}
$$
to this form:
$$
\begin{cases}
\dot{x}_{2}&=J_{11} \dot{\theta}_{1} + J_{12} \dot{\theta}_{2} \\
\dot{y}_{2}&=J_{21} \dot{\theta}_{1} + J_{22} \dot{\theta}_{2}
\end{cases}
$$

---
With some rearranging we get:
$$
\begin{aligned}
\dot{x}_{2} &= \left(-\theta_{2}\sin{\theta_{1}}\right)\dot{\theta}_{1} + \left(\cos{\theta_{1}}\right)\dot{\theta}_{2} \\
\dot{y}_{2} &= \left(\theta_{2}\cos{\theta_{1}}\right)\dot{\theta}_{1} + \left(\sin{\theta_{1}}\right)\dot{\theta}_{2} \\
\end{aligned}
$$
So the Jacobian becomes:
$$
J(\theta_{1},\theta_{2})=
\begin{bmatrix}
-\theta_{2}\sin{\theta_{1}} & \cos{\theta_{1}} \\
\theta_{2}\cos{\theta_{1}} & \sin{\theta_{1}}
\end{bmatrix}
$$
And the Jacobian inverse is:
$$
J^{-1}(\theta_{1},\theta_{2})=
\begin{bmatrix}
-\frac{\sin{\theta_{1}}}{\theta_{2}} & \frac{\cos{\theta_{1}}}{\theta_{2}} \\
\cos{\theta_{1}} & \sin{\theta_{1}}
\end{bmatrix}
$$

---

$$
M(\theta_{1}, \theta_{2}) = \begin{bmatrix}\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2} & 0 \\ 0 & \mathfrak{m}_{2}\end{bmatrix}
$$

$$\Lambda = (J^{-1})^{\top}MJ^{-1}$$


$$
\begin{aligned}
\Lambda &= 
\begin{bmatrix}
  -\frac{\sin{\theta_{1}}}{\theta_{2}} & \frac{\cos{\theta_{1}}}{\theta_{2}} \\
  \cos{\theta_{1}} & \sin{\theta_{1}}
\end{bmatrix}^{\top}
\begin{bmatrix}
  M_{1} & 0 \\
  0 & M_{2}
\end{bmatrix}
\begin{bmatrix}
  -\frac{\sin{\theta_{1}}}{\theta_{2}} & \frac{\cos{\theta_{1}}}{\theta_{2}} \\
  \cos{\theta_{1}} & \sin{\theta_{1}}
\end{bmatrix} \\
&=\begin{bmatrix}
  -\frac{\sin{\theta_{1}}}{\theta_{2}} & \cos{\theta_{1}} \\
  \frac{\cos{\theta_{1}}}{\theta_{2}} & \sin{\theta_{1}}
\end{bmatrix}
\begin{bmatrix}
  M_{1} & 0 \\
  0 & M_{2}
\end{bmatrix}
\begin{bmatrix}
  -\frac{\sin{\theta_{1}}}{\theta_{2}} & \frac{\cos{\theta_{1}}}{\theta_{2}} \\
  \cos{\theta_{1}} & \sin{\theta_{1}}
\end{bmatrix} \\
&=\begin{bmatrix}
  -\frac{M_{1}\sin{\theta_{1}}}{\theta_{2}} & M_{2}\cos{\theta_{1}} \\
  \frac{M_{1}\cos{\theta_{1}}}{\theta_{2}} & M_{2}\sin{\theta_{1}}
\end{bmatrix}
\begin{bmatrix}
 -\frac{\sin{\theta_{1}}}{\theta_{2}} & \frac{\cos{\theta_{1}}}{\theta_{2}} \\
  \cos{\theta_{1}} & \sin{\theta_{1}}
\end{bmatrix} \\
&=
\begin{bmatrix}
M_{1}\frac{\sin^{2}{\theta_{1}}}{\theta_{2}^{2}}+M_{2}\cos^{2}{\theta_{1}} & \left(M_{2}-\frac{M_{1}}{\theta_{2}^{2}}\right)\sin{\theta_{1}}\cos{\theta_{1}} \\
\left(M_{2}-\frac{M_{1}}{\theta_{2}^{2}}\right)\sin{\theta_{1}}\cos{\theta_{1}} & M_{1}\frac{\cos^{2}{\theta_{1}}}{\theta_{2}^{2}}+M_{2}\sin^{2}{\theta_{1}}
\end{bmatrix}
\end{aligned}
$$

---

So the end-effector mass matrix becomes:
$$
\begin{aligned}
\Lambda(\theta_{1},\theta_{2})&=
\begin{bmatrix}
M_{1}\frac{\sin^{2}{\theta_{1}}}{\theta_{2}^{2}}+M_{2}\cos^{2}{\theta_{1}} & \left(M_{2}-\frac{M_{1}}{\theta_{2}^{2}}\right)\sin{\theta_{1}}\cos{\theta_{1}} \\
\left(M_{2}-\frac{M_{1}}{\theta_{2}^{2}}\right)\sin{\theta_{1}}\cos{\theta_{1}} & M_{1}\frac{\cos^{2}{\theta_{1}}}{\theta_{2}^{2}}+M_{2}\sin^{2}{\theta_{1}}
\end{bmatrix} \\
\end{aligned}
$$
We can see that it depends on the angle $\theta_{1}$, but due to symmetry we should be able to analyse the behaviour from one angle $\theta_{1}=0$.

---

Now let $\theta_{1}=0$
$$
\begin{aligned}
\Lambda(0,\theta_{2})&=
\begin{bmatrix}
M_{2} & 0 \\
0 & \frac{M_{1}}{\theta_{2}^{2}}
\end{bmatrix} \\
&=
\begin{bmatrix}
\mathfrak{m}_{2} & 0 \\
0 & \frac{\mathfrak{m}_{1}L_{1}^{2}+\mathfrak{m}_{2}\theta_{2}^{2}+\mathcal{I}_{1}+\mathcal{I}_{2}}{\theta_{2}^{2}}
\end{bmatrix} \\
&=
\begin{bmatrix}
\mathfrak{m}_{2} & 0 \\
0 & \frac{\mathcal{I}_{\mathrm{eq}} + \mathfrak{m}_{2}\theta_{2}^{2}}{\theta_{2}^{2}}
\end{bmatrix} \\
&= \begin{bmatrix}
\mathfrak{m}_{2} & 0 \\
0 & \frac{\mathcal{I}_{\mathrm{eq}}}{\theta_{2}^{2}} + \mathfrak{m}_{2}
\end{bmatrix}
\end{aligned}
$$
Note the units of these, which should be a mass e.g. $\mathrm{kg}$.
Finally, notice that this approaches a point mass $\mathfrak{m}_{2}$ when $\theta_{2}\rightarrow \infty$.

---

# The Newton-Euler approach
---

# Notation
Some important notation
$$
\begin{aligned}
\hat{\mathcal{A}}_{i} &= (\hat{\omega}^{\hat{\mathcal{A}}}_{i},\ \hat{v}^{\hat{\mathcal{A}}}_{i}) \\
\hat{\mathcal{S}}_{i} &= (\hat{\omega}^{\hat{\mathcal{S}}}_{i},\ \hat{v}^{\hat{\mathcal{S}}}_{i}) \\
\mathcal{V}_{i} &= (\omega_{i},\ v_{i}) \\
\dot{\mathcal{V}}_{i} &= (\dot{\omega}_{i},\ \dot{v}_{i}) \\
T_{i,i-1} &=
\begin{bmatrix}
R_{i,i-1} & p_{i,i-1} \\
0 & 1
\end{bmatrix}
\end{aligned}
$$
---

## Given Conditions
Given $\mathcal{F}_{n+1}=\mathcal{F}_{\mathrm{ext}}$ and $\tau$
## Constants
These can be pre-calculated before the simulation.
$$
\begin{aligned}
M_{ij} &= M_{0,i}^{-1}M_{0,j} \\
M_{i,i-1} &= M_{0,i}^{-1}M_{0,i-1} \\
\hat{\mathcal{A}}_{i} &= \mathrm{Ad}_{M_{0,i}^{-1}}(\mathcal{S}_{i})\\
T_{n+1,n} &= M_{n,n+1}^{-1}
\end{aligned}
$$

---

## NE Algorithm
### Forward Propagation of Acceleration/Velocity
For $i=1$ ton $n$ do
$$
\begin{aligned}
T_{i,i-1} &= e^{-[\hat{\mathcal{A}}_{i}]\theta_{i}}M_{i,i-1} \\
\mathcal{V}_{i} &= \mathrm{Ad}_{T_{i,i-1}}(\mathcal{V}_{i-1})+\hat{\mathcal{A}}_{i}\dot{\theta} \\
\dot{\mathcal{V}}_{i} &= \mathrm{Ad}_{T_{i,i-1}}(\dot{\mathcal{V}}_{i-1})+\hat{\mathcal{A}}_{i}\ddot{\theta} + \mathrm{ad}_{\mathcal{V}_{i}}(\hat{\mathcal{A}}_{i})\dot{\theta}_{i}
\end{aligned}
$$

---
## Backward Propagation of Forces
For $i=n$ to $1$ do
$$
\begin{aligned}
\mathcal{F}_{i} &=\mathrm{Ad}^{\top}_{T_{i+1,i}}(\mathcal{F}_{i+1})+\mathcal{G}_{i}\dot{\mathcal{V}}_{i} - \mathrm{ad}_{\mathcal{V}_{i}}^{\top}(\mathcal{G}_{i}\mathcal{V}_{i}) \\
\tau_{i} &= \mathcal{F}_{i}^{\top}\hat{\mathcal{A}}_{i}
\end{aligned}
$$

---

# Decoupled Form for Angular Components
$$
\begin{aligned}
R_{i,i-1}&=e^{-[\hat{\omega}^{\hat{\mathcal{A}}}_{i}]\theta_{i}}\hat{\omega}^{\hat{\mathcal{S}}}_{i} \\
\omega_{i}&=R_{i,i-1}\omega_{i-1}+\hat{\omega}^{\hat{\mathcal{A}}}_{i}\dot{\theta} \\
\dot{\omega}_{i}&=R_{i,i-1}\dot{\omega}_{i-1}+\hat{\omega}^{\hat{\mathcal{A}}}_{i}\ddot{\theta}+(\mathcal{\omega_{i}}\times\hat{\omega}^{\hat{\mathcal{A}}}_{i})\dot{\theta}_{i}
\end{aligned}
$$

---

# Using the NE-ID Algorithm
- How can we use the Newton-Euler Inverse Dynamics algorithm to calculate the Mass matrix $M(\theta)$, as well as $c(\theta,\dot{\theta})$ and $g(\theta)$?

---

# Forward Dynamics Simulation
Given a differential equation, derived from forward dynamics, how can we simulate a system's dynamics?

$$
\begin{aligned}
\ddot{\theta}&=f(\theta, \dot{\theta}) \\
\end{aligned}
$$
Recall that we can approximate the derivatives as:
$$
\begin{aligned}
\ddot{\theta}&\approx\frac{\Delta \dot{\theta}}{\Delta t}= \frac{\dot{\theta}[k+1]-\dot{\theta}[k]}{t[k+1]-t[k]} \\
\dot{\theta}&\approx\frac{\Delta \theta}{\Delta t}= \frac{\theta[k+1]-\theta[k]}{t[k+1]-t[k]}
\end{aligned}
$$