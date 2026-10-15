# Where next

What we take from the best in the niche, what is already there (v1.5), what is planned.

## What was borrowed from whom
| Who | What is valuable | Status here |
|---|---|---|
| **moteus (mjbots)** | one command for all modes: position, velocity, max torque, feedforward, kp/kd on the fly; cascade down to the current loop | impedance command done; current loop — model-based + slow integrator |
| **MIT Mini Cheetah / CubeMars** | impedance in one frame p, v, Kp, Kd, τ_ff | done, plus timestamps and cubic interpolation |
| **Dynamixel Protocol 2.0** | Fast Sync Read, indirect addressing, time-based profiles, CRC16 | SYNC_READ with slots done; CRC16 and 2.0 — no |
| **VESC** | separate limits (motor, bus, regen, rpm) with soft rollback, energy counters | limits and rollback done; the unipolar shunt cannot see regen |
| **ODrive / industrial drives** | two encoders across the gearbox: torque from wind-up, backlash compensation | waiting for an output encoder |
| **CiA 402** | formal state machine, fault latch | done (STATE, LATCHED) |

## Next steps
1. **Magnetic encoder on the output** (MT6701 / AS5600): first analog output into the potentiometer channel, then I2C on two free
   pads. Position sensor layer with 4095 → 0 wrap, turn counter, 360° and multi-turn.
2. **Two sensors across the gearbox** (Halls + encoder): backlash ≈ 12° on reversal is measured — compensate from the angle difference,
   torque estimate from wind-up, collision detection.
3. **Bidirectional bus with a PC**: SYNC_WRITE/SYNC_READ at 1–2 Mbit/s, TIME_SYNC, ID assignment by UID.
4. **Thermal model with an anchor**: die temperature already available; I2C temperature sensor on the motor can → honest continuous torque
   as a function of time (SOA map).
5. **48 V**: board replacements (electrolytic, buck diode, bus ceramics), Vbus divider, rpm limit; 60 V MOSFETs → bus ≤ 48 V.
6. **Single-shunt FOC** — only after scoping the shunt path: if the RC filter smooths pulses by more than 1–2 µs,
   phase currents cannot be reconstructed from one shunt. On an M0 without FPU this is Q15 in registers: two sampling windows per PWM period,
   reconstruction by SVPWM sector, ~40–65 % CPU.

## Deliberately not doing
Anti-cogging (gearbox ≈ 1:220 hides it), a second shunt (board), field weakening (nothing to weaken at Kv 1475).
