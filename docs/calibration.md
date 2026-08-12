# Eye-in-hand calibration

The camera is bolted to the wrist. Everything the vision system says is in the
camera's frame, the robot only takes commands in its own, and the transform
between them — `X`, flange to camera optical frame — is the single number that
decides whether a grasp computed from an image lands on the part. A millimetre
of error in `X` is a millimetre of error at every grasp, forever, and it does
not average out.

Reproduce everything here with:

```bash
make calibration
make calibration-sensitivity
```

## The procedure

1. Move the arm to a set of poses that all see a board clamped in the cell.
2. At each pose, project the board's 63 corners through the pinhole model, add
   0.15 px of detection noise, and recover the board's pose in the camera frame
   by solving PnP with Levenberg–Marquardt.
3. Form the motion pairs and solve `A X = X B` by Tsai–Lenz.
4. Report the residual.

The PnP front end is real rather than a "true pose plus Gaussian noise"
shortcut, because the error PnP actually makes is anisotropic — depth is far
less certain than lateral position at these angles — and that anisotropy is
what determines how many poses the hand-eye solve needs.

## The result

Twenty-five randomised pose sets of each kind, seed 20260101:

| pose set | poses | **true** translation error | **true** rotation error | AX−XB residual | board spread | reprojection |
|---|---:|---:|---:|---:|---:|---:|
| thorough | 20 | **0.38 mm** | **0.049°** | 0.81 mm | 0.19 mm | 0.146 px |
| adequate | 10 | **0.61 mm** | **0.143°** | 0.85 mm | 0.21 mm | 0.146 px |
| hurried | 6 | **4.51 mm** | **0.791°** | 1.03 mm | 0.62 mm | 0.146 px |

Medians. The left pair needs the simulator's ground truth and exists only here.
The right three are what a real cell can compute.

## The point of the table

**The hurried calibration is twelve times worse and looks 27% worse.**

Its reprojection error is identical to three decimal places. Its AX−XB
consistency residual is 1.03 mm against 0.81. Its reconstructed board positions
agree to 0.62 mm. Every diagnostic available on a real robot says it is fine.
It is out by four and a half millimetres.

The reason is conditioning, not noise. `A X = X B` determines the rotation of
`X` only from motions whose rotation axes are not parallel, and the translation
only from motions that rotate at all. A set of nearly-overhead poses rotates
about one axis, so the solve is ill-conditioned — and an ill-conditioned
least-squares problem does not announce itself. It returns a confident answer
with a small residual.

This is why "we calibrated it and the residual was under a millimetre" is not
evidence of anything on its own.

## What the node does about it

`cell_calibration/nodes/hand_eye_node.py` **refuses to solve** when the largest
angle between any two motion axes is under 25°, and says which way to rotate
the wrist. Refusing is the useful behaviour; reporting a residual and letting
someone commission it is not.

`generate_poses` therefore rolls every pose about its view axis and tilts it
off vertical. A set that translates the camera over the board without turning
it produces beautiful board detections every time and leaves the translation of
`X` completely unobservable.

## What it costs downstream

`analysis/calibration_sensitivity.py` injects a known hand-eye translation
error and runs the grasp benchmark at each level. The error is applied mostly
laterally in the camera frame — an error along the *optical axis* is the benign
one, since it only rescales the apparent depth of a plane whose height the cell
already knows, while an error across it moves every part sideways by the same
amount.

| hand-eye error | grasp success | 95% CI | attempted and lost | declined |
|---:|---:|---|---:|---:|
| 0 mm | 100% | 92.9–100 | 0% | 0% |
| 2 mm | 100% | 92.9–100 | 0% | 0% |
| 5 mm | 100% | 92.9–100 | 0% | 0% |
| 10 mm | 98% | 89.5–99.7 | 0% | 2% |
| 16 mm | 90% | 78.6–95.7 | 2% | 8% |
| 22 mm | 76% | 62.6–85.7 | 6% | 18% |
| 30 mm | 54% | 40.4–67.0 | 22% | 24% |

50 trials each. So: the cell tolerates about a centimetre, degrades gracefully
to two, and falls apart past that. Note the *shape* of the degradation as well
as its size — up to 16 mm the failures are overwhelmingly grasps the cell
declined to attempt, which cost a re-present; by 30 mm nearly half of them are
parts it tried to pick and dropped, which cost a stopped line. Two cells at the
same headline rate are not equally good.

The 4.5 mm a hurried calibration produces costs nothing measurable on this
cell — which is a real finding, and one that is specific to parts 26 to 78 mm
wide grasped with several millimetres of jaw clearance. On a cell picking 10 mm
pins it would be the whole error budget.

That is the number worth having. "As accurate as possible" is not an
engineering answer; "the twenty-pose routine buys you nothing on this cell and
everything on a finer one" is.

## Why Tsai–Lenz

It is the method every industrial hand-eye routine has shipped since 1989: a
closed form, rotation first from the modified Rodrigues vectors of each motion
pair, then translation by linear least squares. It is not the most accurate
option — a simultaneous nonlinear solve over all pairs does better on noisy
data — but it has no initialisation and no local minima, which is worth more on
a shop floor than the last few hundredths of a degree.

`calibrate` drops motion pairs whose rotation is under 8°. Tsai–Lenz divides by
the sine of half the motion angle in effect, so a nearly-pure translation
contributes almost nothing but its noise, and including them makes the answer
*worse the more data you collect* — which is a genuinely confusing thing to
debug on hardware.

## Assumptions this inherits

Hand-eye calibration assumes the robot's own forward kinematics is exact. It is
not: joint encoder offsets, gearbox backlash and link length tolerances all
exist, and every one of those errors is absorbed into `X` as though it were a
camera mounting error. `collect_samples` takes an `encoder_noise_rad` argument
so this can be measured rather than assumed. Correcting it properly means
calibrating the robot's kinematic parameters as well, which is a different and
much larger job.
