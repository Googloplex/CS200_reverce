# TODO

Project checklist. Plan details — [docs/roadmap.md](docs/roadmap.md), registers — [docs/registers_v1.md](docs/registers_v1.md).

## Reverse engineering and clone
- [x] SWD, dump of Flash/bootloader/info area, memory map
- [x] Stock firmware analysis: clocking, TIM1 → predriver, Halls, ADC, UART+DMA, protocol, journal, bootloader
- [x] GCC clone under the stock bootloader, A/B against stock 1:1
- [ ] What pad F outputs (currently a position PWM output, as in stock) and what is on PA1 (~1.5 V)
- [ ] Shunt amplifier and RC filter schematic (6-pin SOT next to the shunt?) — scope on PF6

## Firmware
- [x] Halls as a sensor, telemetry, protections, IWDG, SYNC_READ
- [x] Torque in amperes: model-based duty ceiling + measurement as a safety net, I²t, SOA percentage
- [x] Impedance command, TIME_SYNC, cubic interpolation by timestamps
- [x] Soft limits, stall/contact detection, failsafe policy, state machine, latch, TEST_UNLIMITED
- [x] Identity by UID, ID by UID, die temperature, counters, flight recorder, persistent config
- [x] Self-test/identification: direction, steps/unit, Kv, friction, backlash
- [x] Sinusoidal commutation with Hall interpolation, offset and phase advance tuning
- [x] No soft-float; review before the bench
- [ ] Position sensor layer: potentiometer / analog encoder / MT6701, AS5600 over I2C on PA10 + PA0
- [ ] 360° and multi-turn: filter with 4095 → 0 wrap, turn counter, remove the 900..2100 µs limit in the stock loop
- [ ] Two sensors across the gearbox: backlash compensation, torque from wind-up, collision detection
- [ ] Measured current loop (CFG_ILOOP) — check bandwidth under load
- [ ] Protocol: CRC16 / Dynamixel 2.0, indirect addressing — as needed
- [ ] Single-shunt FOC — only after scoping the shunt path (item above)
- [ ] Slim down: drop the stock controller and the PWM input once compatibility is no longer needed (~1.5 KB)

## Bench and measurements
- [x] pp = 1, Kv ≈ 1475 rpm/V, R = 505 mΩ, friction, breakaway duty
- [x] Backlash on reversal 38–48 Hall steps (~12°)
- [ ] Compare backlash on another servo (self-test, `ST_BACKLASH`): gearbox or potentiometer gear
- [ ] Winding L (pulse test or RLC meter), thermal time constants under load
- [ ] Two-way bus with a PC: 470 Ω resistor in the adapter's TX; SYNC_WRITE/READ at 1–2 Mbit, TIME_SYNC, 0x87/0x88 over the wire
- [ ] Torque feed-forward under load; sinusoidal under load (torque smoothness)
- [ ] Coast via MOE = 0 + OSSI instead of clearing CCxE — compare on a scope
- [ ] Self-test and identity on several servos, summary table per batch (`tools/dump_one.py`)

## 48 V
- [ ] Board changes: input electrolytic → 63 V, buck diode SS34 → SS310, bus ceramics → 100 V, input TVS — by test
- [ ] Vbus divider and calibration, rotor speed limit, test 24 → 30 → 36 → 42 → 48 V on a sacrificial servo
- [ ] Brake chopper on the common bus (the unipolar shunt does not see regeneration)
