# Grasp planning

From a depth image to a ranked list of top-down parallel-jaw grasps, and the
reasoning behind each stage. Reproduce everything here with:

```bash
make grasp
```

## The pipeline

```
  depth image
      |  median filter (3x3, NaN-aware)
      v
  point cloud            <- deprojected with the CALIBRATED extrinsic
      |  crop to the taught region of interest
      v
  RANSAC plane fit       <- seeded near the known belt height
      |  discard inliers
      v
  Euclidean clustering   <- KD-tree + union-find, 6 mm radius
      |  largest cluster is the part; the rest become obstacles
      v
  minimum-area rectangle <- rotating calipers on the trimmed convex hull
      |
      v
  candidate generation   <- 2 principal closing directions, x offsets, x depths
      |
      v
  six-term scoring       <- with vetoes, best first
```

## Three things that were wrong, and what they cost

These are the decisions worth reading, because each was made after measuring
what the obvious choice did.

### 1. The median filter is not cosmetic

Without it, the plane fit leaves the belt's noise residue behind as a sparse
but *connected* field of outliers — about 3% of 60,000 belt points, roughly
9 mm apart. Euclidean clustering at any radius large enough to hold a part face
together then percolates straight through it, and the largest cluster becomes
the whole region of interest. The pipeline reported 400–490 mm "parts".

The fix is two changes together: a NaN-aware 3×3 median filter, which knocks
the residue down by about an order of magnitude, and a cluster radius of 6 mm
rather than 12. The radius has to sit between two spacings — the ~1.5 mm
between samples on a part face, which must connect, and the residue, which must
not. Six millimetres is comfortably between them. Twelve is not.

### 2. PCA yaw is undefined on a square, and a third of the catalogue is square

The first version took the part's heading from the principal axis of the
footprint. For a rectangle that is exactly right. For a 55 mm cube it is
meaningless: the two eigenvalues are equal, the eigenvectors are whatever the
dropout pattern says, and the planner confidently closed the jaws across the
78 mm diagonal. Every failure on the two square parts in the catalogue came
from that one effect.

The replacement is a **minimum-area enclosing rectangle** by rotating calipers
over the convex hull. It has no degeneracy: a square's minimum-area rectangle
is the square, in one of four orientations that are all the same grasp. Median
yaw error over the randomised set fell from 0.36° to 0.33°, which sounds like
nothing — but the 95th percentile fell from **24.3° to 5.1°**, and the tail is
where the failures were.

The eigenvalue ratio is still computed, because it remains a useful diagnostic:
it says how *elongated* the footprint is, which is what the planner uses to
decide whether to sweep alternative yaws. It just no longer decides the answer.

Trimming matters here. A minimum-area rectangle is defined by the convex hull,
so it is maximally sensitive to the few stray points the dropout model produces
at grazing incidence. The estimator trims by percentile along the principal
axes before fitting; without that the fit reads several millimetres wide.

### 3. "Jaw clearance" is two numbers, not one

The planner asks for 6 mm of clearance each side of the part before closing —
below about 4 mm the tips catch on the way down whenever the width estimate is
a couple of millimetres light, which it routinely is.

Using that same 6 mm as the *feasibility* test rejects every part wider than
73 mm on an 85 mm stroke. The 78 mm plate in the catalogue then had no grasps
at all, when 3.5 mm a side is a tight but perfectly ordinary industrial grasp.
The preferred clearance and the minimum clearance are different numbers, and
conflating them costs the widest part everything.

## Candidate generation

A box lying on a face has two sensible closing directions: across its short
horizontal axis and across its long one. For each, the planner sweeps three
offsets along the perpendicular axis (centre, ±25% of the extent) and two
insertion depths (50% and 75% of the part height, measured down from the top
face). Twelve candidates for a well-conditioned part.

When the eigenvalue ratio says the footprint has no usable principal axis, it
additionally sweeps yaw in 15° steps over a quadrant. That is brute force, and
it is the honest response to having no information: there is nothing better to
do, and the alternative — trusting a yaw derived from which corner happened to
drop out — is worse.

## Scoring

Six terms, each in [0, 1], combined with fixed weights:

| Term | Weight | What it measures |
|---|---:|---|
| `contact` | 0.24 | Alignment of the closing direction with the face normal it will land on, scaled against a 30° friction cone |
| `width` | 0.22 | Margin between the jaw stroke and the width to span |
| `balance` | 0.16 | How far the grasp line sits from the estimated centre of mass |
| `clearance` | 0.16 | Distance from the fingers to everything that is not the part |
| `approach` | 0.12 | Clearance along the vertical descent into the grasp |
| `posture` | 0.10 | Yoshikawa manipulability of the arm configuration that reaches it |

Any term can also **veto**. A candidate wider than the stroke is not a
low-scoring grasp, it is not a grasp; a candidate whose fingers intersect the
belt is not a grasp either. Vetoes are counted by name, so the benchmark can
report *why* a part had no candidates rather than only that it had none.

**The weights are not tuned against the benchmark.** They were set once, from
the reasoning above, and left alone. Tuning them on the same 200 poses the
result is reported over would make the headline number a measure of
overfitting.

## What the planner is not told

The part's identity, its true dimensions, its mass, its friction coefficient,
and its true pose. It sees a cluster of points and a calibrated camera
transform. The friction cone in the `contact` term is therefore a fixed 30°,
while the *evaluator* uses each part's true coefficient — which means a grasp
the planner considers marginal on a slippery part is one it will lose. That
asymmetry is deliberate and it is where several of the remaining failures come
from.

## Clutter

Every cluster the perception finds that is not the part becomes a collision
obstacle for the duration of the cycle, sized by its own minimum-area
rectangle. The cell has no idea what those objects are — but it can see that
something is there, and planning an approach through a neighbouring part it can
plainly see would be a modelling convenience rather than a cell.

This is what makes the benchmark's `clutter` condition a test of the *planner*
rather than a test of whether obstacles were declared to it.

## Where it would break

Stated as failure modes rather than caveats, because each is predictable:

- **A part with a hole or a slot.** The rectangle fit reads the outline, so the
  grasp is placed correctly and the width is right. Fine.
- **A part with a draft angle.** The rectangle reads the *top* face, which is
  narrower than the base. The commanded opening is then too small and the
  fingers catch — reported as `finger_collision`.
- **Two parts touching.** Clustering cannot separate them and should not be
  expected to. They fit one rectangle covering both, and the grasp is planned
  for an object that does not exist.
- **A cylinder.** The minimum-area rectangle of a circle is a square of the
  diameter, which is the correct grasp width, and the yaw is arbitrary — which
  for a cylinder is also correct. This one works by accident, and would stop
  working the moment the part stopped being rotationally symmetric.
