[Back..](../README.md###SimIn)

# SimIn Variable Usage in BAM.slx

| Block Path | Property Name | Variable Name | Property Value | Value Type | Value Dimensions |
|:-----------|:--------------|:--------------|:--------------|:-----------|:-----------------|
| BAM/BAM Controller/Control Allocation/Constant | Value | SimIn.FCS.UppThrustLimit | 143.735127271207 | double | [1 1] |
| BAM/BAM Controller/Control Allocation/Constant1 | Value | SimIn.FCS.PseudoInv | [-1.41421356237309 1.4142135623731 16.6666666666667 0.25;1.41421356237309 -1.4142135623731 16.6666666666667 0.25;1.4142135623731 1.41421356237309 -16.6666666666667 0.25;-1.4142135623731 -1.41421356237309 -16.6666666666667 0.25] | double | [4 4] |
| BAM/BAM Controller/Control Allocation/Constant2 | Value | SimIn.FCS.LwrThrustLimit | 10.8187730204134 | double | [1 1] |
| BAM/BAM Controller/Control Allocation/Rate Transition | OutPortSampleTime | SimIn.FCS.t_control | 0.005 | double | [1 1] |
| BAM/BAM Controller/Mapping thrust to rpm and saturation/Convert lbf | Gain | SimIn.Units.lbf | 4.4482216152605 | double | [1 1] |
| BAM/BAM Controller/Mapping thrust to rpm and saturation/Convert to standard units | Gain | SimIn.Units.ftlbf | 1.3558179483314 | double | [1 1] |
| BAM/BAM Controller/Mapping thrust to rpm and saturation/Convert to standard units | Gain | SimIn.Units.ftlbf | 1.3558179483314 | double | [1 1] |
| BAM/BAM Controller/Mapping thrust to rpm and saturation/Convert to standard units | Gain | SimIn.Units.ftlbf | 1.3558179483314 | double | [1 1] |
| BAM/BAM Controller/Mapping thrust to rpm and saturation/Convert to standard units | Gain | SimIn.Units.lbf | 4.4482216152605 | double | [1 1] |
| BAM/BAM Controller/Mapping thrust to rpm and saturation/Gain1 | Gain | SimIn.Units.ft | 0.3048 | double | [1 1] |
| BAM/BAM Controller/Mapping thrust to rpm and saturation/Rate Transition | OutPortSampleTime | SimIn.FCS.t_control | 0.005 | double | [1 1] |
| BAM/BAM Controller/PI&FF Rate Controller/Discrete-Time Integrator | SampleTime | SimIn.FCS.t_control | 0.005 | double | [1 1] |
| BAM/BAM Controller/PI&FF Rate Controller/Saturation | UpperLimit | SimIn.FCS.maxRateCmd | [6.28318530717959;6.28318530717959;2.0943951023932] | double | [3 1] |
| BAM/BAM Controller/PI&FF Rate Controller/Saturation | LowerLimit | SimIn.FCS.maxRateCmd | [6.28318530717959;6.28318530717959;2.0943951023932] | double | [3 1] |
| BAM/BAM Controller/Path-following control/Constant6 | Value | SimIn.Units.g0 | 9.80665 | double | [1 1] |
| BAM/BAM Controller/Path-following control/Constant7 | Value | SimIn.FCS.e3 | [0;0;1] | double | [3 1] |
| BAM/BAM Controller/Path-following control/Desired Frame/Attitude Protection/Constant3 | Value | SimIn.FCS.tan_angle_max | 1 | double | [1 1] |
| BAM/BAM Controller/Path-following control/IC | Value | SimIn.IC.Q_i2b | [0.999843945276841;-0.007456992901778;-0.0160144960744764;-0.000119438622514093] | double | [4 1] |
| BAM/BAM Controller/Path-following control/IC1 | Value | SimIn.IC.Omega_BIb | [0;0;0] | double | [3 1] |
| BAM/BAM Controller/Path-following control/Path/Heading Hold Logic prevents badness when trajectory is vertical/Constant1 | Value | SimIn.FCS.e3 | [0;0;1] | double | [3 1] |
| BAM/BAM Controller/Path-following control/Scope | SampleTime | SimIn.model_params.sim_rate | 0.005 | double | [1 1] |
| BAM/BAM Controller/Path-following control/Scope1 | SampleTime | SimIn.model_params.sim_rate | 0.005 | double | [1 1] |
| BAM/BAM Controller/Path-following control/pos | SampleTime | SimIn.model_params.sim_rate | 0.005 | double | [1 1] |
| BAM/BAM Controller/Path-following control/speed | SampleTime | SimIn.model_params.sim_rate | 0.005 | double | [1 1] |
| BAM/BAM Controller/Path-following control/vel | SampleTime | SimIn.model_params.sim_rate | 0.005 | double | [1 1] |
| BAM/Environment/Turbulence  Model/Dryden Model | dt | SimIn.Environment.Turbulence.dT | 0.01 | double | [1 1] |
| BAM/Environment/Turbulence  Model/Dryden Model | Mask: dt | SimIn.Environment.Turbulence.dT | 0.01 | double | [1 1] |
| BAM/Environment/m2ft 3 | Gain | SimIn.Units.m | 1 | double | [1 1] |
| BAM/Plots/Subsystem/r2d | Gain | SimIn.Units.deg | 0.0174532925199433 | double | [1 1] |
| BAM/Plots/Subsystem/r2d1 | Gain | SimIn.Units.deg | 0.0174532925199433 | double | [1 1] |
| BAM/Plots/Subsystem/r2d2 | Gain | SimIn.Units.deg | 0.0174532925199433 | double | [1 1] |
| BAM/Plots/r2d | Gain | SimIn.Units.deg | 0.0174532925199433 | double | [1 1] |
| BAM/Plots/r2d1 | Gain | SimIn.Units.deg | 0.0174532925199433 | double | [1 1] |
| BAM/Plots/r2d2 | Gain | SimIn.Units.deg | 0.0174532925199433 | double | [1 1] |
| BAM/Propulsion and Aerodynamic Forces and Moments/Convert to  ft//sec | Gain | SimIn.Units.ft | 0.3048 | double | [1 1] |
| BAM/Propulsion and Aerodynamic Forces and Moments/Convert to N  | Gain | SimIn.Units.lbf | 4.4482216152605 | double | [1 1] |
| BAM/Propulsion and Aerodynamic Forces and Moments/Convert to N-m | Gain | SimIn.Units.ftlbf | 1.3558179483314 | double | [1 1] |
| BAM/Rate Transition2 | OutPortSampleTime | SimIn.model_params.sim_rate | 0.005 | double | [1 1] |
| BAM/Rate Transition3 | OutPortSampleTime | SimIn.model_params.sim_rate | 0.005 | double | [1 1] |
| BAM/Subsystem Reference2/Band-Limited White Noise | Ts | SimIn.model_params.sim_rate | 0.005 | double | [1 1] |
| BAM/Subsystem Reference2/Band-Limited White Noise | Mask: Ts | SimIn.model_params.sim_rate | 0.005 | double | [1 1] |
| BAM/User Signal Logging/Data Output/Code Generation Output | lengthUserOut | SimIn.lengthUserOut | 336 | double | [1 1] |
| BAM/User Signal Logging/Data Output/Code Generation Output | Mask: lengthUserOut | SimIn.lengthUserOut | 336 | double | [1 1] |
| BAM/Vehicle EOM/Equations of Motion/STARS/Constant2 | Value | SimIn.Units.g0 | 9.80665 | double | [1 1] |
| BAM/Vehicle EOM/Equations of Motion/Simple/Create Inertial Bus/acc due to gravity | Value | SimIn.Units.g0 | 9.80665 | double | [1 1] |
| BAM/Vehicle EOM/Equations of Motion/Simple/Gravity/acc due to gravity | Value | SimIn.Units.g0 | 9.80665 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Angular Rate of NED Frame | a | SimIn.Environment.Earth.RadiusEquator | 6378137 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Angular Rate of NED Frame | e | SimIn.Environment.Earth.Eccentricity | 0.0818191908426215 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Angular Rate of NED Frame | Mask: a | SimIn.Environment.Earth.RadiusEquator | 6378137 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Angular Rate of NED Frame | Mask: e | SimIn.Environment.Earth.Eccentricity | 0.0818191908426215 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Flat Earth to Geodetic | lat | SimIn.IC.LatGeod | 0.647565480656884 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Flat Earth to Geodetic | lon | SimIn.IC.Lon | -1.33319852961523 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Flat Earth to Geodetic | a | SimIn.Environment.Earth.RadiusEquator | 6378137 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Flat Earth to Geodetic | e | SimIn.Environment.Earth.Eccentricity | 0.0818191908426215 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Flat Earth to Geodetic | Mask: lat | SimIn.IC.LatGeod | 0.647565480656884 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Flat Earth to Geodetic | Mask: lon | SimIn.IC.Lon | -1.33319852961523 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Flat Earth to Geodetic | Mask: a | SimIn.Environment.Earth.RadiusEquator | 6378137 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Local NED Flattened/Flat Earth to Geodetic | Mask: e | SimIn.Environment.Earth.Eccentricity | 0.0818191908426215 | double | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Pressure Altitude | u | SimIn.Units | <non-displayable> | struct | [1 1] |
| BAM/Vehicle EOM/World-Relative Data/Pressure Altitude | Mask: u | SimIn.Units | <non-displayable> | struct | [1 1] |
