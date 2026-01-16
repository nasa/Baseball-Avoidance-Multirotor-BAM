# Plots

[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)

This module includes basic visualization capabilities native in MATLAB and
Simulink.

## `Sim_Plots` simulink subsystem

A subsystem at the top level of the BAM simulation that contains scopes for
visualizing multirotor states and command inputs at runtime.

## MATLAB plotting routines

A collection of MATLAB functions for visualizing BAM simulation data in
post-processing. By default, figures are saved to
`Ref_Models/Plots/Saved_Figures/`.

- `plot_trajectories`: Generate and save plots of one or more multirotor and
    baseball trajectories in 3D, ground track, and altitude.

- `plot_multirotor_states`: Generate and save plots of multirotor states (body 
    frame velocity, Euler angles, angular rates) and controls (commanded and 
    actual motor speeds).

### Example usage

After setting up a desired simulation, we can run the simulation and plotting
functions with the commands
```matlab
simout = sim('BAM.slx');
plot_trajectories(simout.logsout);
plot_multirotor_states(simout.logsout);
```

These functions can also plot multiple trajectories in one figure. To do this,
they expect an input which can be indexed by curly braces (`{}`) and with each
item having a `Values` attribute. For example,
```matlab
logsout_data = cell(1, 2);
simout = sim('BAM.slx');
logsout_data{1} = simout.logsout{1}
% Make changes to SimPar
...
simout = sim('BAM.slx');
logsout_data{2} = simout.logsout{1}

plot_trajectories(simout.logsout);
plot_multirotor_states(simout.logsout);
```

To plot the trajectory of a baseball using `plot_trajectories`, construct a
piecewise Bezier curve of the baseball trajectory as described in 
[Bez_Functions](../../Bez_Functions/README.md)
```matlab
wpts = ...  % x, y, and z waypoints
tint = ...  % time values for each waypoint
bball_pwcurve = genPWCurve(wpts, tint);
plot_trajectories(simout.logsout, bball_pwcurve);
```

These functions also accept certain keyword arguments. For a full list, see the
relevant function documentation. As an example, we can change the directory
where plots are saved to:
```matlab
my_save_dir = 'path/to/save/figures';
plot_trajectories(simout.logsout, save_dir=my_save_dir);
plot_multirotor_states(simout.logsout, save_dir=my_save_dir);
```

[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)
