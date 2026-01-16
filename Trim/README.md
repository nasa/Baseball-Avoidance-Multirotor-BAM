# Trim

[Back..](../README.md#api-documentation)

This folder includes scripts and functions for simulating and trimming multirotor dynamics.  The trim solutions are obtained for both the polynomial aero-propulsive model, and the simplified aero-propulsive model.  The folder contains the following:

1. Top level scripts for solving the multirotor trim problem over a range of $u$, $v$, and $w$ body frame velocities. The top level scripts are the `Gen_Trim_Map.m` and `Gen_Trim_Map_Poly.m` for the simplified and polynomial models respectively. These generate trim map scripts make use of MATLAB `fmincon` and corresponding cost, and non-linear constraint m-files.

2. MATLAB `.mat` files which contain the trim solutions over the range of body frame velocities produced from the generate trim map scripts.  These files are `T_map.mat` and `Poly_T_map.mat` for the simplified and polynomial aero-propulsive models respectively.

3. The cost function scripts (used with `fmincon`) `mycost.m` and `mycost_poly.m` for the simplified and polynomial aero-propulsive models respectively.  The cost utilized for the optimization is merely the square of the state derivatives for Euler angles, body velocities, and body rates.  

4. Scripts that compute the state derivatives using an aero-propulsive model.  These scripts are `nldyn_sim_eqn.m` and `nldyn_sim_eqn_poly.m` for the respective aero-propulsive models.

Below, the corresponding m-files for the **simplified aeropropulsive model** are described.  The polynomial model m-files are not described in detail, but are similar enough to the **simplified aeropropulsive model** versions as to be self-explanatory to users.

**Version:** 1.0

## 1. `Gen_Trim_Map`

The function `Gen_Trim_Map` generates a trim map for multirotor configurations, which determines the state and actuator settings required to achieve desired orientations without accelerating.
It systematically explores different velocity states to produce a comprehensive trim map.

### Inputs

- `tmap_fname`: Path/filename of the output .mat data file containing trim maps.

### Outputs

- `t_struc`: Structure containing arrays for pitch (`phi`), roll (`theta`), and propeller rotations per minute (`p1`, `p2`, `p3`, `p4`) across different velocity states.

### Parameters
- `c`, `units`, `rho` (implicitly defined within this script): Multirotor configuration parameters, units, and atmospheric density, respectively.


### Key Process

The function iterates over pre-defined velocity vectors, solving an optimization problem to minimize deviation from desired trimmed conditions using the `fmincon` function:

$$ \text{Objective: } \min_{\text{eu}} \| x_d(1:9) \|^2 $$

where $x_d$ is the state derivative vector calculated by `nldyn_sim_eqn` where the objective (cost) is found in `mycost`. Similar functions for the polynomial model are utilized `nldyn_sim_eqn_poly` and `mycost_poly`. There is a difference in how the use of `fmincon` equality constraints are handled between the polynomial and the simplified aeropropulsive `Gen_Trim_Map` scripts.  In particular, the `Gen_Trim_Map_Poly` uses the state derivatives directly as equality constraints in `fmincon` while `Gen_Trim_Map` (simplified model) does not.

## 2. `nldyn_sim_eqn`

The function `nldyn_sim_eqn` computes the state derivatives based on the aircraft's rigid body motion equations of motion. Users are referred to the polynomial version `nldyn_sim_eqn_poly` to note the slight differences in the polynomial model version of this script.

### Description

This script calculates the translational and rotational dynamics of a multirotor system, taking into account gravitational forces, aerodynamic drag, and propulsion inputs. The function is primarily used to evaluate how changes in control inputs affect the multirotor's state over time.

### Inputs

- `param`: A placeholder for parameter estimates (not used).
- `u`: Control inputs vector.
- `x`: State vector consisting of Euler angles (`φ`, `θ`, `ψ`), body frame velocities (`u`, `v`, `w`), angular rates (`p`, `q`, `r`), and inertial position (`x`, `y`, `z`).
- `c`: Structure containing multirotor configuration parameters.
- `g_fps2`: Gravitational acceleration in ft/s².
- `rho_slugpft3`: Atmospheric density in slug/ft³.
- `aero_p`: Drag force-related multirotor properties.
- `units`: Units structure.

### Outputs

- `xd`: Time derivative of the state vector.
- `accel`: Vector containing translational and rotational accelerations.

### Key Equations

1. **Rotational Kinematics:**
   $$
   \begin{align*}
   \dot{\phi} &= p + \tan(\theta) (q \sin(\phi) + r \cos(\phi)) \\
   \dot{\theta} &= q \cos(\phi) - r \sin(\phi) \\
   \dot{\psi} &= \sec(\theta) (q \sin(\phi) + r \cos(\phi))
   \end{align*}
    $$
2. **Translational Dynamics:**
   $$
   \begin{align*}
   \dot{u} &= -q w + r v - g \sin(\theta) + \frac{X}{m} \\
   \dot{v} &= -r u + p w + g \cos(\theta) \sin(\phi) + \frac{Y}{m} \\
   \dot{w} &= -p v + q u + g \cos(\theta) \cos(\phi) + \frac{Z}{m}
   \end{align*}
   $$

3. **Rotational Dynamics:**
   $$
   \begin{align*}
   \dot{p} &= -(I_z L + I_{xz} N - I_{xz}^2 q r - I_z^2 q r + I_x I_{xz} p q - I_{xz} I_{y} p q + I_{xz} I_{z} p q + I_y I_z q r)/(I_{xz}^2 - I_x I_z) \\
   \dot{q} &= (M - I_{xz} p^2 + I_{xz} r^2 - I_x p r + I_z p r)/I_y; \\
   \dot{r} &= -(I_{xz} L + I_x N + I_x^2 p q + I_{xz}^2 p q - I_x I_y p q - I_x I_{xz} q r + I_{xz} I_y q r - I_{xz} I_z q r)/(I_{xz}^2 - I_x I_z);
   \end{align*}
   $$

4. **Rotation Matrix**
   $$
   R = \begin{bmatrix} \cos(\psi) \cos(\theta) & \cos(\psi) \sin(\phi) \sin(\theta) - \cos(\phi) \sin(\psi) & \sin(\phi) \sin(\psi) + \cos(\phi) \cos(\psi) \sin(\theta)\\
         \cos(\theta) \sin(\psi) & \cos(\phi) \cos(\psi) + \sin(\phi) \sin(\psi) \sin(\theta) & \cos(\phi) \sin(\psi) \sin(\theta) - \cos(\psi) \sin(\phi)\\
                  -\sin(\theta) & \cos(\theta) \sin(\phi) & \cos(\phi) \cos(\theta)
      \end{bmatrix}
      $$

Users are referred to the polynomial version `nldyn_sim_eqn_poly` to note the slight differences in the polynomial model version of this script.

### Dependencies

Simplified aeropropulsive model:
- **`compute_multirotor_FM`:** Computes forces and moments given rotor speeds, velocities, and densities for the simplified aero-propulsive model. This script is located in `AeroProp` folder.
- **`quad_drag`:** Estimates aerodynamic drag forces acting on the quadrotor for the simplified aero-propulsive model. This script is located in `AeroProp` folder.

Polynomial aeropropulsive model:
- **`IMPACT_AeroProp_RSE_V1_10Jun25_180239`:** Computes the forces and moments for the polynomial aero-propulsive model.  This script is located in the `AeroProp` folder.

## Additional Functions

- **`nlinCon_poly`:** Solves the non-linear equations and uses them as equality constraints for use with fmincon in `mycost_poly`.

[Back..](../README.md#api-documentation)
