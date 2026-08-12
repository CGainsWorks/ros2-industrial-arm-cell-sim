# Cycle time

Where the time goes in one pick-and-place cycle, and what each part of it
responds to. Reproduce with:

```bash
make grasp
make cycle-time
```

## How it is measured

Not by a stopwatch. Every stage that consumes time contributes a named
`CycleSegment`, and the cycle time is their sum. That distinction matters: the
simulation does not run in real time, so a wall-clock measurement of it would
be a statement about the machine it ran on.

Each motion segment's duration comes from the time parameterisation in
`cell_motion/core/trajectory.py`, applied to the actual planned path. So the
figure is "how long this trajectory would take on a UR5e held to these
velocity and acceleration limits" — which is a real claim about the robot,
subject to the assumptions below.

## The assumptions, stated up front

**Limits are scaled to 50% velocity and 35% acceleration.** Not 100%. A UR5e
running a pick and place at its datasheet limits is not something anyone
commissions in a cell with a person near it, and quoting a cycle time obtained
at full limits would be quoting a number nobody would run. Both the offline
parameterisation and the MoveIt `joint_limits.yaml` use the same two scalars,
so a trajectory planned either way takes the same time.

**No corner blending is generated.** The profile bounds the speed at each
waypoint so the direction change fits inside a 60 ms blend window at the
acceleration limit, but it does not produce the blend geometry. A real
controller rounds the corner; this reports the time as if it tracked the
polyline exactly.

**Fixed costs are from datasheets, not invented.** Camera settle 150 ms,
capture and transfer 90 ms at 640×480, PLC handshake 40 ms (two 20 ms scans),
gripper close scaled from the Robotiq 2F-85's 0.03–0.07 m/s jaw speed.

**Inspection is not included.** The five-point probe adds about 8 s, which is
most of a cycle again. `docs/inspection.md` reports it separately rather than
folding it into a headline that would then not be comparable to anything.

## The breakdown

Mean over the 192 successful cycles of the 200-trial nominal run. Median total
9.88 s, mean 10.39 s, p05 9.39 s, p95 14.79 s, range 9.19–15.32 s.

| segment | mean | p95 | share | kind |
|---|---:|---:|---:|---|
| `to_scan` | 2.60 s | 2.60 s | 25.1% | motion |
| `to_home` | 1.89 s | 2.56 s | 18.2% | motion |
| `transfer` | 1.79 s | 3.42 s | 17.2% | motion |
| `to_pregrasp` | 1.33 s | 3.65 s | 12.8% | motion |
| `descend` | 0.57 s | 0.62 s | 5.5% | motion |
| `lift` | 0.57 s | 0.62 s | 5.5% | motion |
| `place` | 0.45 s | 0.46 s | 4.4% | motion |
| `retract` | 0.45 s | 0.46 s | 4.4% | motion |
| `open_gripper` | 0.33 s | 0.33 s | 3.2% | gripper |
| `camera_settle` | 0.15 s | 0.15 s | 1.4% | sensor |
| `camera_capture` | 0.09 s | 0.09 s | 0.9% | sensor |
| `close_gripper` | 0.08 s | 0.08 s | 0.8% | gripper |
| `cycle_start_handshake` | 0.04 s | 0.04 s | 0.4% | fieldbus |
| `cycle_done_handshake` | 0.04 s | 0.04 s | 0.4% | fieldbus |

By kind: motion 9.66 s (93.0%), gripper 0.41 s (4.0%), sensor 0.24 s (2.3%),
fieldbus 0.08 s (0.8%).

Run `make cycle-time` for the exact figures from your own run; the JSON in
`results/cycle_time.json` carries them to three decimals.

## What to do with the split

A cycle time is only actionable once it is split by what produced it.

**Motion is 93% of it, and 43% of the cycle is travelling empty.** The two
free moves — `to_scan` at 2.60 s and `to_home` at 1.89 s — are 4.49 s of a
10.39 s mean, and neither does anything with the part. Three things would
attack that, in descending order of payoff:

1. **Delete the home pose.** Nothing requires the cycle to start and end there;
   it exists so that every trial is independent, which is a benchmark
   requirement, not a cell requirement. A production cycle would go straight
   from the retract to the next scan.
2. **Move the scan pose.** It sits 0.50 m above the belt because that is the
   standoff at which the camera's field of view covers the taught region. A
   narrower region, or a wider-angle lens, brings it closer to the pick and
   shortens both the approach and the retreat.
3. **Overlap the capture with the previous retreat.** The 240 ms of sensor time
   is small, but the *motion to get there* is not, and a cell that scans part
   N+1 while placing part N removes the whole `to_scan` segment from the
   critical path.

**Sensor time responds to nothing but a faster camera.** 240 ms of settle and
transfer is 2.3% of the cycle and there is no clever way to shorten it.

**Gripper time responds to commanding a smaller opening**, which trades
directly against catching the part on the way down — see `JAW_CLEARANCE` in
`docs/grasp-planning.md`. Not worth 0.3 s.

**Fieldbus time is 80 ms — 0.8%.** It is reported because it is the term
everyone assumes is large. It is not.

## The spread

The p95 (14.79 s) sits 50% above the median (9.88 s), and it is not noise. The
two segments with a p95 far above their mean are `to_pregrasp` (1.33 → 3.65 s)
and `transfer` (1.79 → 3.42 s), which is where it comes from:

- **RRT-Connect is randomised.** A run that takes an unlucky route through
  free space produces a longer path, and the shortcut pass does not always
  recover it. The gap between the median and the p95 is essentially the cost of
  having a sampling planner rather than an optimising one.
- **Part position changes the transfer distance.** A part at the far corner of
  the belt is 0.2 m further from the nest than one at the near corner.

The first is fixable — a fixed seed makes the benchmark reproducible but does
not make the planner deterministic across part poses, and a
shortest-path-refining planner would tighten the distribution. The second is
the cell.

## The regression worth recording

The first time parameterisation gave each segment its velocity-limited duration
and then iteratively stretched any segment whose implied acceleration was too
high. That diverges on a finely sampled path: stretching one segment lowers its
speed, which *creates* a velocity discontinuity with its neighbour, which
demands stretching that one, and so on.

A 100 mm Cartesian approach sampled every 5 mm came out at **nine seconds**,
and the whole cycle at 34 s — for a pick and place a UR5e does in under ten.
The number was wrong in the direction that looks conservative, which is the
easiest kind of wrong number to publish and the hardest to notice.

`tests/test_cycle_and_outcome.py::test_a_finely_sampled_short_move_does_not_take_forever`
guards it.
