[Back..](../README.md)

# SimPar Variable Usage in BAM.slx

| Block Path | Property Name | Variable Name | Property Value | Value Type | Value Dimensions |
|:-----------|:--------------|:--------------|:--------------|:-----------|:-----------------|
| Model Configuration | StopTime | SimPar.model_params.stop_time | 80 | double | [1 1] |
| BAM/BAM Controller/PI&FF Rate Controller/Constant1 | Value | SimPar.vehicle.mass.I | [0.029262 0 0;0 0.031578 0;0 0 0.047887] | double | [3 3] |
| BAM/BAM Controller/PI&FF Rate Controller/Gain Kff/Constant | Value | SimPar.vehicle.mass.I | [0.029262 0 0;0 0.031578 0;0 0 0.047887] | double | [3 3] |
| BAM/BAM Controller/PI&FF Rate Controller/Gain Kff/Constant1 | Value | SimPar.FCS.Cont.Inner.Kff_1 | [0.1;0.1;0.1] | double | [3 1] |
| BAM/BAM Controller/PI&FF Rate Controller/Gain Ki/Constant | Value | SimPar.vehicle.mass.I | [0.029262 0 0;0 0.031578 0;0 0 0.047887] | double | [3 3] |
| BAM/BAM Controller/PI&FF Rate Controller/Gain Ki/Constant1 | Value | SimPar.FCS.Cont.Inner.Ki_1 | [0.8;0.8;0.8] | double | [3 1] |
| BAM/BAM Controller/PI&FF Rate Controller/Gain Kp/Constant | Value | SimPar.vehicle.mass.I | [0.029262 0 0;0 0.031578 0;0 0 0.047887] | double | [3 3] |
| BAM/BAM Controller/PI&FF Rate Controller/Gain Kp/Constant1 | Value | SimPar.FCS.Cont.Inner.Kp_1 | [7;7;7] | double | [3 1] |
| BAM/BAM Controller/Path-following control/Constant1 | Value | SimPar.IC.Q_i2b | [0.996028827442064;0.00603286149235858;0.0166034758370229;0.0872611258048479] | double | [4 1] |
| BAM/BAM Controller/Path-following control/Constant18 | Value | SimPar.FCS.Cont.Outer.KL_1 | 1 | double | [1 1] |
| BAM/BAM Controller/Path-following control/Constant2 | Value | SimPar.FCS.Cont.Outer.KR_1 | 1.25 | double | [1 1] |
| BAM/BAM Controller/Path-following control/Constant3 | Value | SimPar.FCS.Cont.Outer.KX_1 | 2.75 | double | [1 1] |
| BAM/BAM Controller/Path-following control/Constant4 | Value | SimPar.FCS.Cont.Outer.KV_1 | 2.75 | double | [1 1] |
| BAM/BAM Controller/Path-following control/Constant5 | Value | SimPar.FCS.mass_nominal | 1.9593 | double | [1 1] |
|  |  |  | [] |  | [] |
|  |  |  | [] |  | [] |
|  |  |  | [] |  | [] |
| BAM/Environment/Turbulence  Model/Dryden Model | longseed | SimPar.Environment.Turbulence.RandomSeedLong | 3366 | double | [1 1] |
| BAM/Environment/Turbulence  Model/Dryden Model | latseed | SimPar.Environment.Turbulence.RandomSeedLat | 23 | double | [1 1] |
| BAM/Environment/Turbulence  Model/Dryden Model | vertseed | SimPar.Environment.Turbulence.RandomSeedVert | 1369 | double | [1 1] |
| BAM/Environment/Turbulence  Model/Dryden Model | Mask: longseed | SimPar.Environment.Turbulence.RandomSeedLong | 3366 | double | [1 1] |
| BAM/Environment/Turbulence  Model/Dryden Model | Mask: latseed | SimPar.Environment.Turbulence.RandomSeedLat | 23 | double | [1 1] |
| BAM/Environment/Turbulence  Model/Dryden Model | Mask: vertseed | SimPar.Environment.Turbulence.RandomSeedVert | 1369 | double | [1 1] |
| BAM/Environment/Turbulence Level | Value | SimPar.Environment.Turbulence.intensity | 3 | double | [1 1] |
| BAM/Environment/WindDir (True, deg) | Value | SimPar.Environment.Turbulence.WindDirectionAt5kft | 60 | double | [1 1] |
| BAM/Environment/WindSpd (kts) | Value | SimPar.Environment.Turbulence.WindAt5kft | 0 | double | [1 1] |
| BAM/FCS Params/Constant4 | Value | SimPar.FCS.prop.ParamsNom | [45 0.25 -0.028 -1 1 0.01 0 -1.8248e-06 0 0 0 10000 0 0 0 0.833333333333333 0;225 0.25 -0.028 -1 1 0.01 0 -1.5221e-06 0 0 0 10000 0 0 0 0.833333333333333 0;315 0.25 -0.028 1 1 0.01 0 -1.8699e-06 0 0 0 10000 0 0 0 0.833333333333333 0;135 0.25 -0.028 1 1 0.01 0 -1.6581e-06 0 0 0 10000 0 0 0 0.833333333333333 0] | double | [4 17] |
| BAM/FCS Params/Constant5 | Value | SimPar.FCS.CM_Nom | [0;0;0.001] | double | [3 1] |
| BAM/FCS Params/Constant6 | Value | SimPar.FCS.mass_nominal | 1.9593 | double | [1 1] |
| BAM/FCS Params/Constant7 | Value | SimPar.FCS.J_nominal | [0.029262 0 0;0 0.031578 0;0 0 0.047887] | double | [3 3] |
| BAM/OwnShip Traj/Reference Inputs/BEZIER/Constant3 | Value | SimPar.RefInputs.trajectory.wptVect | [0;120;240;3;3;0;0;0;0;0;0;0;0;0;0;0;0;0;0;-60;0;0;-3;3;0;0;0] | double | [27 1] |
| BAM/OwnShip Traj/Reference Inputs/BEZIER/Constant4 | Value | SimPar.RefInputs.trajectory.timeVect | [0;40;80;0;40;80;0;40;80] | double | [9 1] |
| BAM/OwnShip Traj/Reference Inputs/BEZIER/Constant5 | Value | SimPar.RefInputs.trajectory.parseVect | [3;3;3;3;3;3;3] | double | [7 1] |
|  |  |  | [] |  | [] |
|  |  |  | [] |  | [] |
|  |  |  | [] |  | [] |
|  |  |  | [] |  | [] |
| BAM/Subsystem Reference/First Order Rate and Position Limited & Time Delay | Bias | SimPar.IC.rpm_init | [6907.25852698139;7848.37583483524;7213.7441106667;8204.77236112714] | double | [4 1] |
| BAM/Subsystem Reference/First Order Rate and Position Limited & Time Delay | Mask: Bias | SimPar.IC.rpm_init | [6907.25852698139;7848.37583483524;7213.7441106667;8204.77236112714] | double | [4 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Constant | Value | SimPar.vehicle.mass.mass | 1.9593 | double | [1 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Constant1 | Value | SimPar.vehicle.mass.I | [0.029262 0 0;0 0.031578 0;0 0 0.047887] | double | [3 3] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Create STARS Initial Conditions/Constant | Value | SimPar.IC.Pos_bii | [0;0;0] | double | [3 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Create STARS Initial Conditions/Constant1 | Value | SimPar.IC.Vel_bIb | [2.99846112989938;0.00143303980263507;-0.096066637702588] | double | [3 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Create STARS Initial Conditions/Constant2 | Value | SimPar.IC.Omega_BIb | [0;0;0] | double | [3 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Create STARS Initial Conditions/Constant3 | Value | SimPar.IC.Q_i2b | [0.996028827442064;0.00603286149235858;0.0166034758370229;0.0872611258048479] | double | [4 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Rigid Body Dynamics | Accel_i_IC | SimPar.EOM.Accel_bIi | [0;0;0] | double | [3 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Rigid Body Dynamics | ASensed_IC | SimPar.EOM.Asensed_bIb | [0.314030630875362;-0.146270930230427;-9.80052926123592] | double | [3 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Rigid Body Dynamics | OmegaDot_IC | SimPar.EOM.OmegDtI_BIb | [0;0;0] | double | [3 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Rigid Body Dynamics | Mask: Accel_i_IC | SimPar.EOM.Accel_bIi | [0;0;0] | double | [3 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Rigid Body Dynamics | Mask: ASensed_IC | SimPar.EOM.Asensed_bIb | [0.314030630875362;-0.146270930230427;-9.80052926123592] | double | [3 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Rigid Body Dynamics | Mask: OmegaDot_IC | SimPar.EOM.OmegDtI_BIb | [0;0;0] | double | [3 1] |
| BAM/Vehicle EOM/Equations of Motion/Simple/Gravity/mass | Value | SimPar.vehicle.mass.mass | 1.9593 | double | [1 1] |
|  |  |  | [] |  | [] |
| BAM/Vehicle EOM/Equations of Motion/Simple/inertia | Value | SimPar.vehicle.mass.I | [0.029262 0 0;0 0.031578 0;0 0 0.047887] | double | [3 3] |
| BAM/Vehicle EOM/Equations of Motion/Simple/mass | Value | SimPar.vehicle.mass.mass | 1.9593 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Constant3 | Value | SimPar.IC.GrndAltMSL | 0 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Topodetic Position | Pos_bee | SimPar.EOM.Pos_bee | [1198760.75527728;-4950035.93475557;3826495.56582697] | double | [3 1] |
| BAM/Vehicle EOM/World-Relative Data/Topodetic Position | Q_e2h | SimPar.EOM.Q_e2h | [1;0;0;0] | double | [4 1] |
| BAM/Vehicle EOM/World-Relative Data/Topodetic Position | Mask: Pos_bee | SimPar.EOM.Pos_bee | [1198760.75527728;-4950035.93475557;3826495.56582697] | double | [3 1] |
| BAM/Vehicle EOM/World-Relative Data/Topodetic Position | Mask: Q_e2h | SimPar.EOM.Q_e2h | [1;0;0;0] | double | [4 1] |
