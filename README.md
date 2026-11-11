# Automatic Plant Watering Based on Soil Moisture Sensor and PID Control

Code and data for the article *Automatic Plant Watering Based on Soil Moisture Sensor and PID Control*,
Science, Engineering and Technology (SET Journal), 2026.

The system is an Arduino Uno with a soil moisture sensor, a relay module and a DC pump, built and checked in
Tinkercad. Its closed-loop behaviour is analysed in Scilab with a first-order pump/motor model that drives a first-order
soil water balance. Running the Scilab scripts reproduces every value in Tables 3–6 and the data behind Figures 4–7.

## Contents

| File | Purpose | Paper |
| --- | --- | --- |
| `pid_model.sci` | Shared model: parameters (Table 2), PID and ON/OFF simulators, performance metrics | Sections 4.4–4.5, Eqs. (3)–(5) |
| `PID_Ziegler_Nichols_Tuning.sce` | Ziegler–Nichols (ZN) starting point and the six trial gain sets | Table 3 |
| `PID_Main_Simulation.sce` | Baseline PID, scenario S1 (30% → 60%) | Table 4, Figures 4–5, text values |
| `PID_Validation_Comparison.sce` | Scenarios S1–S4, and PID versus hysteresis ON/OFF | Tables 5–6, Figures 6–7 |
| `make_figures.py` | Figure-generation script: redraws Figures 4–7 from the CSV files | Figures 4–7 |
| `arduino.ino` | Arduino firmware (PID, anti-windup, 2 s relay window, interlock) | Algorithm 1, Eq. (6) |
| `tinkercad.txt` | Link to the Tinkercad circuit | Figure 2 |
| `results/` | Console outputs (`*_console.txt`), exported data (`*.csv`) and figures (`figures/`) | — |

## How to run

1. Install Scilab (tested with Scilab 2024.0.0).
2. Run each script from the Scilab console, or from a terminal:

   ```
   scilab-cli -nb -quit -f PID_Ziegler_Nichols_Tuning.sce
   scilab-cli -nb -quit -f PID_Main_Simulation.sce
   scilab-cli -nb -quit -f PID_Validation_Comparison.sce
   ```

   Each script loads `pid_model.sci` from its own folder and writes its CSV files to `results/`.
3. Optional: `python make_figures.py` (Python 3, numpy, matplotlib) redraws Figures 4–7 into `results/figures/`.

## Model and metrics

- Pump/motor: τm·dm/dt = Km·u(t) − m(t), with Km = 1.20 and τm = 2.50 s.
- Soil water balance: dθ/dt = KI·m(t) − KE·θ(t) − KD·θ(t), with KI = 0.080 s⁻¹, KE = 0.015 s⁻¹ and KD = 0.005 s⁻¹.
- Forward Euler with a step of 0.01 s over a 60 s horizon. The integral state is limited to ±100 and the command to 0–255.
- Baseline gains: Kp = 5.0, Ki = 0.8, Kd = 0.1. The ON/OFF baseline uses a ±1 pp hysteresis band and a 180/255 command.
- Sensor disturbance (S4 and Table 3): a ±1 pp sinusoid at 0.8 Hz added to the measurement.
- Metrics:
  - rise time: to 90% of the step;
  - overshoot: as a percentage of the setpoint;
  - settling time: ±2% band;
  - final error at 60 s, e60: |setpoint − θ(60 s)|;
  - IAE: Σ|e|·Δt over 0–60 s;
  - pump switching events: transitions of the simulated pump-active state, i.e. a command above 10/255;
  - σu: sample standard deviation of the command for t ≥ 45 s.

The coefficients are nominal, illustrative values, not identified from a real pump or soil. The time axis is therefore
illustrative too.

**Simulation versus firmware.** The Scilab model applies the saturated PID command directly as a continuous average
drive. It does not model the firmware's 2 s relay window (Eq. 6), the interlock that keeps the relay OFF when
θ ≥ θsp, or the 1% resolution of the integer sensor mapping. Those features are implemented in `arduino.ino` and were
checked only qualitatively in Tinkercad. The switching counts reported here are transitions of the simulated
pump-active state, not relay operations.

## Expected output (scenario S1, baseline gains)

```
Rise time t_r            : 4.13 s
Peak value               : 77.43 % at 9.60 s
Overshoot                : 29.06 %
Settling time t_s (2%)   : 42.83 s
Final moisture at 60 s   : 60.10 %
Final error at 60 s e60  : 0.10 pp
IAE                      : 415.9 %.s
Pump switching events    : 5
```

The complete outputs are in `results/*_console.txt`.
