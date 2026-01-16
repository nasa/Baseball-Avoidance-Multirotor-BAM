# Environment System

[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)

This environmental model simulates atmospheric conditions and local wind velocities. The model integrates static wind patterns, turbulence effects, and standard atmospheric properties to create realistic environmental conditions for simulation scenarios.  Users who wish to include atmospheric winds (steady-state) and turbulence should use the `userStruct.simulation_defaults.Environment.Turbulence` fields. Note that the winds and turbulence are always active in the simulation, but the default values in the `userStruct.simulation_defaults.Environment.Turbulence` winds and turbulence intensity fields are set to zero.

## Subsystems

### 1. Static Winds

The static wind component represents a constant wind field throughout the simulation environment.  The associated `userStruct.simulation_defaults.Environment.Turbulence` fields are: `WindAt5kft` (kts) and `WindDirectionAt5kft` (deg True).

**Features:**
- Configurable wind magnitude and direction parameters
- Horizontal wind components fully customizable
- Vertical wind component currently zeroed out by default.  
- Wind vectors are applied in the Earth-fixed reference frame

### 2. Turbulence Model

The turbulence model simulates atmospheric disturbances that affect aircraft motion.

**Configuration Options:**
- **Model Selector:** This is manually selected in the `Dryden Model` subsystem 
  - 1 = TM-4511 specification
  - 2 = MIL-1797 specification (Military standard)
- **Intensity Levels:** This is specified using the `userStruct.simulation_defaults.Environment.Turbulence.intensity` field.
  - 1 = Light
  - 2 = Moderate
  - 3 = Severe

**Implementation Details:**
- Based on the Dryden turbulence model
- Default configuration uses TM-4511 specification
- Gusts are generated as body-frame velocities (`u, v, w`)
- Severe turbulence (level 3) is scaled by the following factors for each intensity level:
  - None: `[0, 0, 0]`
  - Light: `[0.2, 0.2, 0.17]` for (u, v, w) respectively
  - Moderate: `[0.4, 0.4, 0.34]` for (u, v, w) respectively
  - Severe: `[0.4, 1.152, 0.34]` for (u, v, w) respectively
- Only linear turbulence effects are currently implemented

**Note:** The turbulence implementation focuses on translational disturbances without rotational components.

### 3. Atmospheric Model

A single-layer atmospheric model based on the U.S. Standard Atmosphere 1976.

**Key Constants:**
- Gas constant (r): 1716.55915670803
- Ratio of specific heats (gamma): 1.4
- Gravitational acceleration (g): 32.17405 ft/s²
- Earth radius (re): 20855531.5 ft
- Sea level temperature (tsl): 518.67°R (288.15 K × 1.8)
- Sea level pressure (psl): 2116.22 psf
- Sea level altitude (hsl): 0 ft
- Sea level density (sldens): 0.0023769 slugs/ft³
- Sea level pressure (slpr): 2116.220 psf

**Features:**
- Calculates air density, pressure, and temperature as functions of altitude
- Provides standard atmospheric properties for flight simulation


## Limitations

- Turbulence model only includes linear effects
- Atmospheric model uses a simplified single-layer approach
- No terrain effects on wind patterns are currently modeled

---


[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)