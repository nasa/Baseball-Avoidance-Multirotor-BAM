## Controller

[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)

The BAM simulation comes with a baseline geometric quad-rotor controller.  Relevant reference papers include <cite> Lee et al. [1] </cite>,<cite>Ackerman et al. [2] </cite>.

The basic structure of the controller consists of the following cascaded elements:
- Outer loop: Path-following control
- Inner loop: Proportional integral and feed forward rate controller
- Control allocation: Pseudo-inverse (single matrix) allocator
- Motor Model: quadratic model with saturation (converts desired thrust to motor RPM)

The output of the outer loop are desired body rates and a total thrust desired.  The inner loop takes only the desired body rates from the outer loop and converts them into desired rotational moments.  The control allocation subsystem takes the desired moments and the desired total thrust and allocates the thrust to each of the four motors.  Finally, the Thrust_to_RPM_MDL subsystem uses a non-linear model to convert the desired thrust for each motor into an RPM command.

<center>
<img src=".graphics/Bam_Cont.png" width="550" height="250" />
</center>

---

### **Outer Loop**

The controller outer loop takes the following inputs:
1. Actual position and velocity, expressed in NED inertial frame
2. Actual body rates (`p,q,r`) and quaternion (inertial to body)
3. Desired position, velocity and acceleration, expressed in NED inertial Frame
4. Desired heading angle

The outputs of the outer loop are a total desired thrust and desired body rates.  The total desired thrust is computed as: (see Eq. 15 of <cite> [1] </cite>)

$$
   \begin{align*}
   f = -(k_x e_x-k_v e_v-mg e_3+m\ddot{x}_d) \cdot Re_3
   \end{align*}
$$

where $k_x$ and $k_v$ are the gains on the position error $e_x=p_{act}-p_{des}$ and the velocity error $e_v=v_{act}-v_{des}$ respectively. The vector $e_3$ is a unit vector in the z-direction, and $\ddot{x}$ is the NED inertial frame actual acceleration. Details on computation of the desired body rate are found in <cite> [1],[2] </cite>.   

---

### **Inner Loop**

Details on the inner loop proportional, integral and feed-forward controller design are found in <cite>[2]</cite>.  Recall that the inner loop converts desired body rates $\omega_{des}$ into desired rotational moments.  Details on the terms used to compute $\omega_{des}$ can be seen in Eq. 16 of <cite>[1]</cite>.   
<center>
<img src=".graphics/Inner_Loop.png" width="550" height="250" />
</center>

---

### **Control Allocation**

The input into the control allocation subsystem is a concatenated vector of desired rotational moments and the desired total thrust.  Conversion of this vector into desired thrusts for each motor is performed using a simple weighted pseudo-inverse single matrix control allocation algorithm. In general, for a given controls effectiveness matrix $B$ and specified weighting matrix $W$, the weighted pseudo-inverse is found using:

$$
   P = B^TW^T(BW^TB^T)^{-1}
$$

In this application, the specified weighting matrix is the identity matrix, and thus the pseudo-inverse used is $P=B^T(BB^T)^{-1}$ in the BAM simulation.

---

### **Motor Model (Thrust to RPM Model)**

The output of the control allocation is a vector of 4 desired thrusts (one for each motor).  The thrust to RPM subsystem converts each desired thrust into a commanded motor RPM using a quadratic polynomial model of the form:

$$ T = ar^2+br+c$$

with the thrust $T$, the commanded RPM $r$ and the polynomial coefficients $a,b,c$.  Rearranging the polynomial fit and solving for the (real) zeros gives:

$$ 
r = \frac{-b+(b^2-4a(c-T))^{1/2}}{2a}
$$

Note, saturation (maximum and minimum RPM limits) are enforced after the quadratic model determination.

---

[1] *Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE3*, T. Lee, M. Leok, H. McClamroch, revised 9 Sep 2011 (https://arxiv.org/abs/1003.2005)

[2] *Flight Control Methods for Multirotor UAS*, K Ackerman, I. Gregory, and N. Hovakimyan, 2019 International Conference on Unmanned Aircraft Systems (ICUAS), Atlanta GA., June 11-14, 2019


[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)
