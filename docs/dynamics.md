# Fast moves: where the current goes, heat and bus voltage

Evidence tags: [MEAS] measured, [calc] derived, [HYP] estimate. Parameters — [servo_model.md](servo_model.md).
Output angles and speeds are at the servo output; R = 0.505 Ω cold, ~0.66 Ω hot; N = 206; Kt ≈ 7.1 mN·m/A.

## 1. In fast unloaded moves the current accelerates the rotor
Equation of motion at the output (α in rad/s²):

    Kt·N·(I − I₀) = J_rotor·N²·α + (J_load·α + T_load)/η

- The rotor inertia sits **before** the gearbox: J_rotor·N² ≈ 1.0·10⁻⁶ × 206² ≈ **0.042 kg·m²** at the output [calc] —
  like 1 kg at a 20 cm radius. The gearbox efficiency η does not apply to this term.
- A light payload on the output is often 0.005–0.02 kg·m² [HYP] → in a fast repositioning move without an external load
  **70–90 % of the current goes into accelerating and braking the motor itself**.
- Unloaded: 15 A → ~28 000 °/s² at the output, 8 A → ~14 000 °/s² [calc; matches the measured 0 → 90 % speed in 65–69 ms at 8 A].

## 2. Peaks are fine, RMS is what heats
- Peak current: switches 90 A, a 0.1 s burst of 25 A (~300 W in the winding) is short for the thermal mass.
- Heating follows the RMS current over the whole duty cycle: I_rms = I_peak·√(fraction of time at peak).
  Example: 25 A for 2 × 0.1 s every second → I_rms ≈ 11 A → 60–80 W in the winding — not sustainable.
- Continuous current is **not measured**; estimate 4–6 A RMS with the housing bolted to an aluminium frame with thermal paste [HYP].
  Any motion plan should be checked for I_rms per servo over the cycle, not only for peak current.

## 3. Bus voltage matters for high current at high speed
To drive current I at output speed ω (°/s) at full duty:

    V ≥ I·R + ω/39.0          (39.0 °/s per volt = 6·Kv/N)

Output speed up to which current I is still available [calc]:

| Vbus | 15 A (cold / hot) | 20 A | 25 A |
|---|---|---|---|
| 24 V | 641 / 550 °/s | 543 / 422 | 444 / 293 |
| 36 V | 1109 / 1019 | 1011 / 890 | 912 / 761 |
| 48 V | 1578 / 1487 | 1479 / 1358 | 1381 / 1229 |

- Only accelerating at speed is voltage-limited. Braking is not: the back-EMF adds to the applied voltage. The braking energy
  goes back to the supply (a battery takes it; a bench PSU does not — see bench_log.md).
- Bus current = winding current × duty: at low speed a large winding current draws little from the bus.

## 4. Levers that reduce the current of fast moves
For a move dominated by inertia (amplitude θ, time T):

- **Move time.** Current ∝ θ/T², copper heat per move ∝ θ²/T³. +25 % of move time → −36 % peak current, ~−49 % heat.
- **Trajectory shape.** Heat per move ∝ ∫α²dt; rest-to-rest move, in units of θ²/T³ [calc]:

  | Profile | ∫α²dt | Peak α (θ/T²) |
  |---|---|---|
  | Cubic (linear acceleration) — minimum heat | 12.0 | 6.0 |
  | Bang-bang (full acceleration / full braking) | 16.0 | 4.0 |
  | Quintic (minimum jerk) | 17.1 | 5.8 |
  | Cycloidal (sinusoidal acceleration) | 19.7 | 6.3 |

  The cubic costs 25 % less heat than bang-bang but steps the current at the start and end; minimum jerk is smoother at +7 % heat.
- **Linkage ratio** between the servo output and the driven member, r = servo angle / member angle: the torque needed for
  inertia is J_motor·r·α + J_load·α/r, minimal at **r* = √(J_load / 0.042)**. For a light payload r* < 1 — the servo should
  turn less than the member. The price: for a static load the servo torque is T_static/r, i.e. more current when holding a load.
  Good for members that mostly move unloaded, bad for members that mostly hold weight.

## 5. What to check in a model
- Rotor inertia on the motor side (not divided by η), 0.042 kg·m² at the output.
- Voltage constraint V ≥ I·R + ω/39 with the hot winding resistance.
- I_rms per servo over the cycle against a continuous limit (to be measured), not just the peak.
- Gearbox efficiency η is not measured (0.7–0.9) — it scales the static torque terms, not the rotor inertia.
