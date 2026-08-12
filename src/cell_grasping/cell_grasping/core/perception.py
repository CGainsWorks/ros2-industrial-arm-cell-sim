"""From a depth cloud to an estimated part pose.

Three stages, all of them the ordinary industrial ones:

1. **Plane removal.** RANSAC-fit the dominant plane and discard its inliers.
   The belt height is known to within a few millimetres from the cell drawing,
   so a fixed height threshold would work; RANSAC is used anyway because a belt
   that is not quite level, or a camera whose hand-eye transform is a
   millimetre out, moves the apparent plane, and a fixed threshold then either
   keeps a skin of belt points or eats the bottom of the part. The fit is
   seeded with the nominal height so it cannot latch onto a large part face.

2. **Clustering.** Euclidean clustering on the remaining points, via a KD-tree
   and union-find. The largest cluster is the part; the rest are handed back as
   clutter, and the cycle puts them into the collision scene.

3. **Pose estimation.** A minimum-area enclosing rectangle over the trimmed
   convex hull of the cluster's footprint, plus the height above the support
   plane.

WHERE THE ERROR COMES FROM, AND WHY IT MATTERS

The estimate is systematically biased in two ways that no amount of averaging
removes, and both of them are real:

* Only the top face and at most two sides are visible, so the centroid of the
  visible points sits above and towards the camera relative to the part's true
  centroid. The estimator corrects for the top-face bias using the measured
  height, which is the standard fix, and the residual is what is left.
* Yaw is well defined for a rectangle and only defined modulo 90 degrees for a
  square, which is a property of the part rather than of the estimator. The
  estimator reports the PCA eigenvalue ratio alongside the fit so the grasp
  planner can tell "the long axis is that way" from "there is no long axis" and
  decide whether sweeping alternative yaws is worth the candidates. Using PCA
  to produce the yaw itself, which is what the first version did, is a
  different matter: on a square footprint the eigenvectors are noise, and the
  planner grasped 55 mm cubes across their 78 mm diagonal.

Both show up in ``analysis/perception_error.py``, which reports the
distribution of centroid, yaw and extent error over the same randomised part
set the grasp benchmark uses.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class PlaneFit:
    """A fitted plane and the points that did not belong to it."""

    normal: np.ndarray
    offset: float
    inlier_count: int
    outliers: np.ndarray

    def height_at(self, points: np.ndarray) -> np.ndarray:
        return np.atleast_2d(points) @ self.normal - self.offset


@dataclass
class PartEstimate:
    """The pose and size the perception stage believes the part has."""

    centre: np.ndarray  # World-frame centre of the part body.
    yaw: float  # Heading of the longer horizontal axis.
    extents: np.ndarray  # (long, short, height) in metres.
    axis_ratio: float  # Longer horizontal extent over shorter. 1.0 means square.
    support_height: float  # Height of the surface the part is sitting on.
    point_count: int
    eigen_ratio: float  # PCA eigenvalue ratio; near 1.0 means the yaw is meaningless.

    @property
    def yaw_is_meaningful(self) -> bool:
        """Whether the footprint is elongated enough for PCA yaw to mean anything.

        The threshold is on the *eigenvalue* ratio, not the extent ratio: a
        square face with one corner occluded has a perfectly respectable extent
        ratio and a yaw that points at the occlusion.
        """
        return self.eigen_ratio > 1.35


def fit_plane_ransac(
    points: np.ndarray,
    rng: np.random.Generator,
    tolerance: float = 0.004,
    iterations: int = 60,
    nominal_height: Optional[float] = None,
    nominal_tolerance: float = 0.03,
) -> PlaneFit:
    """RANSAC plane fit, biased towards a horizontal plane near a known height.

    ``nominal_height`` restricts the sample set to points that could plausibly
    belong to the support surface. Without it, RANSAC on a scene where the part
    is the largest single planar patch in view -- which happens with the
    100 x 70 mm coupon at close range -- fits the *part's top face* and
    discards it, leaving the belt as the "object".
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 3:
        return PlaneFit(np.array([0.0, 0.0, 1.0]), 0.0, 0, points)

    if nominal_height is not None:
        candidates = points[np.abs(points[:, 2] - nominal_height) < nominal_tolerance]
        if len(candidates) < 3:
            candidates = points
    else:
        candidates = points

    best_normal = np.array([0.0, 0.0, 1.0])
    best_offset = float(np.median(candidates[:, 2]))
    best_count = int(np.sum(np.abs(points[:, 2] - best_offset) < tolerance))

    for _ in range(iterations):
        sample = candidates[rng.choice(len(candidates), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal = normal / norm
        if normal[2] < 0:
            normal = -normal
        if normal[2] < 0.9:
            continue  # Not a support surface; a wall or the side of a fixture.
        offset = float(normal @ sample[0])
        count = int(np.sum(np.abs(points @ normal - offset) < tolerance))
        if count > best_count:
            best_normal, best_offset, best_count = normal, offset, count

    residual = points @ best_normal - best_offset
    return PlaneFit(best_normal, best_offset, best_count, points[residual > tolerance])


def euclidean_clusters(
    points: np.ndarray,
    radius: float = 0.006,
    min_points: int = 60,
) -> List[np.ndarray]:
    """Cluster points by connectivity, returning index arrays, largest first.

    The default radius is deliberately small. It has to sit between two
    spacings: the ~1.5 mm between neighbouring samples on a part face at this
    standoff, which must connect, and the residue the plane fit leaves behind
    on the belt, which must not. Six millimetres is comfortably between them.
    Twelve is not -- the residue percolates and the largest cluster becomes the
    entire region of interest.

    A KD-tree pair query plus union-find. The alternative -- a per-point
    breadth-first search -- is the textbook formulation and is an order of
    magnitude slower at the ~10k points a single part produces, which matters
    when the benchmark runs the pipeline two hundred times.
    """
    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        return []

    tree = cKDTree(points)
    pairs = tree.query_pairs(radius, output_type="ndarray")

    parent = np.arange(len(points))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    for left, right in pairs:
        root_left, root_right = find(left), find(right)
        if root_left != root_right:
            parent[root_right] = root_left

    labels = np.array([find(index) for index in range(len(points))])
    clusters = []
    for label in np.unique(labels):
        members = np.flatnonzero(labels == label)
        if len(members) >= min_points:
            clusters.append(members)
    clusters.sort(key=len, reverse=True)
    return clusters


def minimum_area_rectangle(points: np.ndarray) -> Tuple[np.ndarray, float, np.ndarray]:
    """Smallest-area enclosing rectangle of a 2D point set.

    Rotating calipers over the convex hull edges: the minimum-area rectangle
    always has a side flush with a hull edge, so testing one axis-aligned
    bounding box per edge is exhaustive, not a heuristic. Returns the centre,
    the heading of the longer side, and the (long, short) extents.

    WHY THIS AND NOT PCA

    PCA gives the direction of greatest variance, which for a rectangle is its
    long axis -- until the rectangle is square, at which point the two
    eigenvalues are equal and the eigenvectors are whatever the noise says. A
    55 mm cube's estimated yaw is then uniformly distributed, the planner grasps
    it across a diagonal 78 mm wide, and the jaws hit it on the way down. In the
    200-trial benchmark that single effect accounted for every failure on the
    two square parts in the catalogue.

    A minimum-area rectangle has no such degeneracy: a square's minimum-area
    rectangle is the square, in one of four orientations that are all the same
    grasp. The eigenvalue ratio is still computed and reported, because it is a
    genuinely useful diagnostic -- it says how *elongated* the footprint is,
    which the planner uses to decide whether to sweep alternative yaws -- but it
    no longer decides the answer.
    """
    from scipy.spatial import ConvexHull

    hull = ConvexHull(points)
    vertices = points[hull.vertices]
    edges = np.diff(np.vstack([vertices, vertices[:1]]), axis=0)
    angles = np.arctan2(edges[:, 1], edges[:, 0])
    angles = np.unique(np.mod(angles, np.pi / 2.0))

    best = None
    for angle in angles:
        rotation = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
        rotated = vertices @ rotation.T
        low = rotated.min(axis=0)
        high = rotated.max(axis=0)
        extents = high - low
        area = float(extents[0] * extents[1])
        if best is None or area < best[0]:
            centre_rotated = (low + high) / 2.0
            best = (area, angle, extents, centre_rotated @ rotation)

    _, angle, extents, centre = best
    if extents[0] >= extents[1]:
        return centre, float(angle), np.array([extents[0], extents[1]])
    return centre, float(angle + np.pi / 2.0), np.array([extents[1], extents[0]])


def estimate_part(
    points: np.ndarray,
    support_height: float,
    percentile: float = 2.0,
) -> Optional[PartEstimate]:
    """Fit an oriented box to a cluster of surface points.

    Outliers are trimmed by percentile along the principal axes *before* the
    rectangle fit. That order matters: a minimum-area rectangle is defined by
    the convex hull, so it is maximally sensitive to exactly the few stray
    points the dropout model produces at grazing incidence, and an untrimmed
    fit reads several millimetres wide. Percentile trimming is the cheap robust
    estimator here; a full RANSAC rectangle fit would be the thorough one and
    is not worth its runtime at this point count.
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 20:
        return None

    horizontal = points[:, :2]
    centred = horizontal - horizontal.mean(axis=0)
    covariance = centred.T @ centred / max(len(centred) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigen_ratio = float(np.sqrt(eigenvalues[0] / max(eigenvalues[1], 1e-12)))

    low, high = percentile, 100.0 - percentile
    keep = np.ones(len(horizontal), dtype=bool)
    for axis in range(2):
        projection = centred @ eigenvectors[:, order[axis]]
        keep &= (projection >= np.percentile(projection, low)) & (projection <= np.percentile(projection, high))
    trimmed = horizontal[keep] if np.count_nonzero(keep) >= 12 else horizontal

    try:
        centre_xy, yaw, extents = minimum_area_rectangle(trimmed)
    except Exception:
        # A degenerate hull -- every point collinear, which happens when a part
        # is seen almost edge-on through a narrow strip of surviving pixels.
        # Fall back to the principal axes rather than losing the detection.
        long_axis = eigenvectors[:, order[0]]
        projection = centred @ long_axis
        centre_xy = horizontal.mean(axis=0)
        yaw = float(np.arctan2(long_axis[1], long_axis[0]))
        extents = np.array([float(projection.max() - projection.min()), 0.001])

    top = float(np.percentile(points[:, 2], 98.0))
    height = max(top - support_height, 0.002)

    return PartEstimate(
        centre=np.array([centre_xy[0], centre_xy[1], support_height + height / 2.0]),
        yaw=float(yaw),
        extents=np.array([extents[0], extents[1], height]),
        axis_ratio=float(extents[0] / max(extents[1], 1e-6)),
        support_height=support_height,
        point_count=int(len(points)),
        eigen_ratio=eigen_ratio,
    )


def detect_part(
    cloud: np.ndarray,
    rng: np.random.Generator,
    nominal_support_height: float,
    cluster_radius: float = 0.006,
    workspace: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None,
) -> Tuple[Optional[PartEstimate], PlaneFit]:
    """The primary detection only. See :func:`detect_scene` for the rest."""
    primary, _, plane = detect_scene(cloud, rng, nominal_support_height, cluster_radius, workspace)
    return primary, plane


def detect_scene(
    cloud: np.ndarray,
    rng: np.random.Generator,
    nominal_support_height: float,
    cluster_radius: float = 0.006,
    workspace: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None,
    max_clutter: int = 4,
) -> Tuple[Optional[PartEstimate], List[PartEstimate], PlaneFit]:
    """The whole perception pipeline: cloud in, part estimate and clutter out.

    ``workspace`` crops to the pick region in x and y before clustering. That
    is not cheating: a real cell has a taught region of interest, and without
    one the largest cluster in this scene is the camera pillar.

    The *other* clusters are returned too, and the cycle puts them into the
    collision scene as obstacles. This is the honest handling of clutter: the
    cell has no idea what those objects are, but it can see that something is
    there, and planning an approach through a neighbouring part it can plainly
    see would be a modelling convenience rather than a cell. It is also what
    makes the clutter condition in the grasp benchmark a test of the *planner*
    rather than a test of whether obstacles were declared to it.
    """
    cloud = np.asarray(cloud, dtype=float)
    if workspace is not None:
        (x_low, x_high), (y_low, y_high) = workspace
        inside = (
            (cloud[:, 0] >= x_low)
            & (cloud[:, 0] <= x_high)
            & (cloud[:, 1] >= y_low)
            & (cloud[:, 1] <= y_high)
            & (cloud[:, 2] < nominal_support_height + 0.25)
        )
        cloud = cloud[inside]

    plane = fit_plane_ransac(cloud, rng, nominal_height=nominal_support_height)
    clusters = euclidean_clusters(plane.outliers, radius=cluster_radius)
    if not clusters:
        return None, [], plane

    estimates = [estimate_part(plane.outliers[members], support_height=plane.offset) for members in clusters]
    estimates = [estimate for estimate in estimates if estimate is not None]
    if not estimates:
        return None, [], plane
    return estimates[0], estimates[1 : 1 + max_clutter], plane
