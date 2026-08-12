# Force-feedback contact inspection

After the part is placed in the nest, the tool probes five points on its top
face and measures the height at each. Five points give a plane, and the plane
says whether the part is seated: a part sitting on a chip of swarf, or dropped
into the nest at an angle, shows up as a tilt of a few tenths of a degree —
which no camera at this standoff resolves and which a probe measures in a
couple of seconds.

Reproduce with:

```bash
make inspection
```

## The loop

Velocity-based admittance at 500 Hz, one law throughout:

```
v_z = (F_measured − F_target) / damping
```

In free space `F` is zero, so the probe descends at `F_target / damping`, and
the damping is chosen to make that the intended 5 mm/s. As the force rises the
descent slows, and the loop settles where `F = F_target`.

It is worth writing it as one law rather than as "approach, then switch to
force control on contact". The switched version needs a mode flag; the version
that keeps the approach term *and* adds the force term — which is the obvious
reading of the admittance equation, and what the first draft did — never stops
descending, because at `F = F_target` it is still commanding the full approach
speed. Every probe drove to the 45 N abort limit.

Contact is declared when the *filtered* force crosses 2 N. That declaration
affects only the measurement, not the control, so a late or early trigger costs
accuracy and cannot cost the part.

Stability: the contact time constant is damping/stiffness = 50 ms against a
2 ms control period, so the margin is generous.

## The error budget, measured

40 repeats per row, error in microns (measured minus true):

**Force-sensor noise**

| noise (N RMS) | mean | std | \|err\| p95 |
|---:|---:|---:|---:|
| 0.1 | −11.0 | 2.4 | 15.0 |
| 0.35 | −7.7 | 6.9 | 18.1 |
| 1.0 | 86.2 | 517.5 | 27.1 |
| 2.0 | 4542 | 536 | 5079 |

**Contact stiffness**

| stiffness (N/m) | mean | std | \|err\| p95 |
|---:|---:|---:|---:|
| 5 000 | 32.1 | 25.8 | 77.1 |
| 20 000 | −7.7 | 6.9 | 18.1 |
| 80 000 | −11.5 | 2.2 | 15.4 |

**Approach speed**

| speed (mm/s) | mean | std | \|err\| p95 |
|---:|---:|---:|---:|
| 2 | 2.7 | 7.0 | 15.0 |
| 5 | −7.7 | 6.9 | 18.1 |
| 10 | −20.8 | 7.4 | 32.1 |
| 20 | −42.3 | 9.6 | 56.3 |

At the settings the cell runs — 0.35 N sensor noise, 20 kN/m contact, 5 mm/s —
the probe is good to about **8 µm mean, 18 µm at the 95th percentile.**

## Reading the budget

**The trigger threshold against the stiffness dominates, and it is corrected
for.** Declaring contact at 2 N into a 20 kN/m contact means the probe is
already 100 µm in. That is more than every other term combined, so it is
subtracted rather than ignored — and the correction needs a stiffness estimate,
which `estimate_stiffness` measures from the force ramp during the settle
rather than assuming. The nominal contact stiffness of a part in a nest is a
guess: it depends on the part, the nest, and how much of the robot's own
compliance is in series.

**Noise breaks the trigger, not the measurement.** Between 0.35 N and 1.0 N the
error goes from tens of microns to hundreds, with a standard deviation of half
a millimetre. That is not the measurement degrading smoothly — it is filtered
noise crossing the 2 N threshold before the tip touches anything, on some runs
and not others. The fix on a noisy sensor is to raise the threshold in
proportion, which costs proportionally more deflection to correct for. A UR's
built-in force estimate sits near 1 N; a proper wrist FT sensor sits near 0.1.

**Approach speed is a straight trade.** Roughly 2 µm of extra depth per mm/s,
because the loop cannot react inside one 2 ms period. Probing four times faster
costs five times the error.

## Seating detection

A part with one corner lifted, 20 repeats per row, acceptance threshold 0.35°:

| corner lift | detected | measured tilt |
|---:|---:|---:|
| 0.00 mm | 0% | 0.021° |
| 0.05 mm | 0% | 0.069° |
| 0.10 mm | 0% | 0.141° |
| 0.20 mm | 0% | 0.270° |
| 0.30 mm | **100%** | 0.399° |
| 0.50 mm | **100%** | 0.662° |
| 1.00 mm | **100%** | 1.310° |

**False positives on a correctly seated part: 0%.**

The transition is sharp because the measurement noise (tens of microns) is far
below the feature being measured (hundreds), so the threshold is what decides,
not the noise. On a 62 × 38 mm part the 0.35° limit corresponds to a corner
lift of 0.27 mm, which is exactly where detection switches on.

That is the specification this routine can be held to: **it catches anything
over about 0.3 mm and nothing under 0.2, and it does not cry wolf.** Lowering
the threshold to catch 0.2 mm would work — the measurement supports it — and
would start reporting the nest's own flatness. A seating check that never
misses and stops the line twice a shift is worse than no seating check.

## Faults

The routine raises the same codes the fieldbus publishes: `NO_CONTACT` (21)
when the probe travels its 12 mm allowance without touching anything, which is
what an empty nest looks like; `OVERFORCE` (22) at 45 N; `NOT_SEATED` (23) when
the fitted plane exceeds the tilt limit. Same numbers on both sides of the
bridge, asserted by `tests/test_fieldbus.py`.

## Cost

Five probes at about 1.4 s each, plus repositioning: roughly 8 s of added cycle
time. That is most of a cycle again, which is why the inspection is not run on
every part in the benchmark — `analysis/inspection_accuracy.py` measures it
standalone, and the cycle-time figures in the README are for pick and place
without it. Stated rather than folded in.
