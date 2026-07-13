# PMSM dq SVPWM Simulink Model

This folder contains a programmatically generated Simulink model for a surface-mounted PMSM position step response.

## Files

- `build_pmsm_svpwm_model.m` builds `pmsm_dq_svpwm_cascade.slx` and can optionally run the simulation.
- `run_pmsm_svpwm_step_response.m` rebuilds the model, runs the step response, and exports plots.
- `pmsm_dq_svpwm_cascade.slx` is the generated Simulink model.
- `pmsm_step_response.png` and `pmsm_step_response.fig` are the latest simulation plots.
- `tests/test_pmsm_svpwm_model.m` verifies model generation and the step response.

## Model

The plant uses the rotor dq equations for a surface PMSM:

```text
v_d = R_s i_d + L_d di_d/dt - omega_e L_q i_q
v_q = R_s i_q + L_q di_q/dt + omega_e (L_d i_d + psi_f)
T_e = 1.5 p (psi_f i_q + (L_d - L_q) i_d i_q)
J d omega_m/dt = T_e - B omega_m - T_load_motor
```

For the surface PMSM case, `L_d = L_q` and `i_d_ref = 0`.

The control structure is:

```text
output position PI -> output speed PI -> dq current PI -> SVPWM voltage limiting -> PMSM dq plant -> gearbox output
```

The DC bus voltage is `Vdc = 36 V`. The voltage vector is limited to `Vdc / sqrt(3)`, and SVPWM duty cycles are generated with common-mode injection.

The gearbox has two reduction stages:

```text
stage 1 output speed = input speed / 3.71428571428
stage 2 output speed = input speed / 14
total output speed = motor speed / 52
```

The position command and plotted position are output-shaft quantities. Motor-shaft position and speed are logged separately as `theta_m` and `omega_rpm`; first-stage output is logged as `theta_stage1` and `omega_stage1_rpm`; final output is logged as `theta_out` and `omega_out_rpm`.

Output load torque is reflected to the motor shaft as:

```text
T_load_motor = T_load_output / GearTotalRatio
```

## Current Motor Parameters

The model currently uses these motor values:

| Parameter | Value | Note |
| --- | ---: | --- |
| `Vdc` | `36 V` | Fixed DC bus voltage |
| `Rs` | `0.6 ohm` | Assumed phase resistance |
| `Ld`, `Lq` | `0.82 mH` | Assumed phase dq inductance |
| `PolePairs` | `14` | 28 magnetic poles |
| `MaxSpeedRpm` | `1500 rpm` | Motor-shaft maximum speed |
| `GearStage1Ratio` | `3.71428571428` | First reduction stage denominator |
| `GearStage2Ratio` | `14` | Second reduction stage denominator |
| `GearTotalRatio` | `52` | Total reduction denominator |
| `OutputMaxSpeedRpm` | `28.85 rpm` | Output-shaft speed limit |
| `RatedCurrent` | `3.8 A` | Stored for reference |
| `PeakCurrent` | `19.5 A` | Stored as drive/current capability reference |
| `RatedTorqueMotor` | `0.7 N*m` | Motor-shaft torque, before gearbox |
| `StallTorqueMotor` | `2.2 N*m` | Motor-shaft torque, before gearbox |
| `RatedTorqueOutput` | `36.4 N*m` | Output torque after 52:1 reduction, ideal efficiency |
| `StallTorqueOutput` | `114.4 N*m` | Output torque after 52:1 reduction, ideal efficiency |
| `KvRpmPerVolt` | `6.71` | Raw user value, kept for reference |
| `PsiF` | `0.00945 Wb` | Estimated from 36 V and 1500 rpm |
| `Kt` | `0.198 N*m/A` | Equivalent q-axis torque constant |
| `RatedCurrentFromTorque` | `3.53 A` | Calculated from `0.7 / Kt` |
| `StallCurrentFromTorque`, `IqLimit` | `11.08 A` | Calculated from `2.2 / Kt`; used as q-axis current limit |
| `J` | `8e-5 kg*m^2` | Estimate until measured data is available |
| `B` | `1e-4 N*m*s/rad` | Estimate until measured data is available |

Important: the `6.71` value is not currently used to calculate `PsiF`, because it conflicts with the 36 V / 1500 rpm constraint unless its unit is clarified.

The simplified plant clamps `i_q` to `IqLimit`, clamps `i_d` to the peak current value, and clamps motor speed to `MaxSpeedRpm`. These limits keep near-stall load tests bounded and represent drive/mechanical protection boundaries.

The mechanical load is a step torque input:

```text
T_load_output = 0 N*m before 1.80 s
T_load_output = 102.96 N*m after 1.80 s
```

This is `90%` of the ideal output-shaft stall torque:

```text
0.9 * StallTorqueOutput = 0.9 * 114.4 = 102.96 N*m
```

Reflected to the motor shaft, this is:

```text
102.96 / 52 = 1.98 N*m
```

The simulation logs `iq`, `omega_rpm`, `theta_m`, `omega_stage1_rpm`, `theta_stage1`, `omega_out_rpm`, `theta_out`, `id`, `iq_ref`, `omega_ref_rpm`, `vd`, `vq`, `duty_a`, `duty_b`, `duty_c`, `load_torque`, `load_torque_motor`, `torque_e`, `torque_motor`, `torque_stage1`, and `torque_output` as timeseries workspace outputs.

Torque signals use these meanings:

```text
torque_motor  = motor-shaft electromagnetic torque before gearbox
torque_stage1 = torque_motor * GearStage1Ratio
torque_output = torque_motor * GearTotalRatio
load_torque   = external load torque at output shaft
```

## Run

In MATLAB, run:

```matlab
run_pmsm_svpwm_step_response
```

To run the tests:

```matlab
runtests("tests/test_pmsm_svpwm_model.m")
```

## Parameters Still Estimated

For a closer match to your motor, replace these values in `getDefaultParameters()` when you know them:

- `J`: rotor plus coupled-load inertia.
- `B`: viscous friction coefficient.
- `LoadTorqueFinal`: load step torque.
