"""A simulated eye-in-hand depth camera.

Ray-casts a pinhole depth image against the cell's boxes, then degrades it the
way a stereo depth camera degrades: range-dependent Gaussian noise, quantised
output, a working range outside which it returns nothing, and dropout at
grazing incidence.

WHY SIMULATE THE SENSOR AT ALL

The grasp planner could be handed the part's true pose. Every number that came
out of it would then be a statement about the planner's arithmetic and nothing
else, and the grasp success rate would be whatever the geometry allowed --
close to 100%, and worthless as evidence. Almost all of the interesting failure
modes in a real vision-guided cell come from the perception being *nearly*
right: a yaw estimate off by four degrees, an extent underestimated because
only the top face is visible, a centroid pulled sideways because half the part
is in shadow. Those only appear if the pipeline has to start from pixels.

The noise model is the D400-series shape: sigma grows with the square of range
because stereo disparity error maps to depth error quadratically. The constants
are in ``simulation/cell.yaml``.

WHAT IS NOT MODELLED

No colour, no texture, no ambient IR, no multi-path. The parts here are matte
boxes on a matte belt, which is the case where a depth camera behaves well;
shiny or transparent parts break stereo depth in ways that no amount of
Gaussian noise represents, and pretending otherwise would flatter the result.
``docs/limitations.md`` says so.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from cell_common.cell_model import CameraSpec
from cell_motion.core.collision import Box


@dataclass
class DepthImage:
    """A rendered depth frame and the pose it was taken from.

    ``depth`` is metres along the optical axis, with NaN where the camera
    returned nothing. ``pose`` is the ground-truth optical frame in the world;
    the perception pipeline is given an *estimate* of it, not this.
    """

    depth: np.ndarray
    pose: np.ndarray
    camera: CameraSpec
    stride: int = 1

    @property
    def valid_fraction(self) -> float:
        return float(np.mean(np.isfinite(self.depth)))

    def pixel_grid(self) -> Tuple[np.ndarray, np.ndarray]:
        """Pixel coordinates of each rendered sample, honouring the stride."""
        rows = np.arange(0, self.camera.height, self.stride)
        columns = np.arange(0, self.camera.width, self.stride)
        return np.meshgrid(columns, rows)


def ray_directions(camera: CameraSpec, stride: int = 1) -> np.ndarray:
    """Unit-z ray directions in the optical frame, shape (H', W', 3).

    The third component is exactly 1, so the ray parameter *is* the depth along
    the optical axis. That is what a depth camera reports, and normalising the
    directions here would silently turn the output into range instead --
    a 4% error at the edge of this field of view, which is enough to tilt an
    estimated part plane by a couple of degrees.
    """
    columns, rows = np.meshgrid(
        np.arange(0, camera.width, stride, dtype=float),
        np.arange(0, camera.height, stride, dtype=float),
    )
    x = (columns - camera.cx) / camera.fx
    y = (rows - camera.cy) / camera.fy
    return np.stack([x, y, np.ones_like(x)], axis=-1)


def _slab_intersection(origin: np.ndarray, directions: np.ndarray, box: Box) -> np.ndarray:
    """Entry distance from ``origin`` along each direction to ``box``.

    Standard slab test in the box's own frame. Returns +inf where the ray
    misses. ``directions`` is (N, 3) in world coordinates.
    """
    rotation = box.rotation
    local_origin = rotation.T @ (origin - box.centre)
    local_directions = directions @ rotation

    half = box.half_extents
    with np.errstate(divide="ignore", invalid="ignore"):
        inverse = 1.0 / local_directions
        t1 = (-half - local_origin) * inverse
        t2 = (half - local_origin) * inverse

    # A direction component of exactly zero gives +-inf here, which is the
    # right answer -- the ray never crosses that slab -- except when the origin
    # is already inside it, where numpy produces a NaN from 0 * inf. Replace
    # those explicitly rather than letting the NaN propagate into the minimum.
    parallel = np.abs(local_directions) < 1e-12
    inside = np.abs(local_origin) <= half
    t1 = np.where(parallel, np.where(inside, -np.inf, np.inf), t1)
    t2 = np.where(parallel, np.where(inside, np.inf, -np.inf), t2)

    near = np.max(np.minimum(t1, t2), axis=-1)
    far = np.min(np.maximum(t1, t2), axis=-1)
    hit = (far >= np.maximum(near, 0.0)) & (far > 0.0)
    entry = np.where(near > 0.0, near, far)  # Origin inside the box: exit face.
    return np.where(hit, entry, np.inf)


def render_depth(
    pose: np.ndarray,
    boxes: Sequence[Box],
    camera: CameraSpec,
    rng: Optional[np.random.Generator] = None,
    stride: int = 1,
    noise: bool = True,
) -> DepthImage:
    """Render a depth image of ``boxes`` from a camera at ``pose``.

    ``pose`` is the optical frame in the world: +z along the view axis.
    """
    pose = np.asarray(pose, dtype=float)
    origin = pose[:3, 3]
    directions = ray_directions(camera, stride)
    shape = directions.shape[:2]
    world_directions = directions.reshape(-1, 3) @ pose[:3, :3].T

    depth = np.full(world_directions.shape[0], np.inf)
    incidence = np.zeros_like(depth)
    for box in boxes:
        distances = _slab_intersection(origin, world_directions, box)
        closer = distances < depth
        if not np.any(closer):
            continue
        depth = np.where(closer, distances, depth)

        # Incidence angle at the hit, used for the dropout model. The face
        # normal is whichever box axis the hit point is most extreme along.
        # Computed only for the rays that actually hit this box: the misses
        # carry an infinite distance, and letting those through produces
        # inf - inf NaNs that then poison the sign and the minimum.
        index = np.flatnonzero(closer)
        hit_directions = world_directions[index]
        points = origin + distances[index, None] * hit_directions
        local = (points - box.centre) @ box.rotation
        axis = np.argmax(np.abs(local) / np.maximum(box.half_extents, 1e-9), axis=1)
        normals = box.rotation[:, axis].T * np.sign(np.take_along_axis(local, axis[:, None], axis=1))
        unit = hit_directions / np.linalg.norm(hit_directions, axis=1, keepdims=True)
        incidence[index] = np.abs(np.einsum("ij,ij->i", normals, unit))

    depth = np.where(np.isfinite(depth), depth, np.nan)
    depth = np.where((depth < camera.depth_min) | (depth > camera.depth_max), np.nan, depth)

    if noise:
        rng = rng if rng is not None else np.random.default_rng()
        sigma = camera.noise_sigma(np.nan_to_num(depth, nan=1.0))
        depth = depth + rng.normal(0.0, sigma)
        # Grazing-incidence dropout. A stereo camera loses a surface it sees
        # nearly edge-on; the box sides are exactly that, and keeping them
        # would hand the pose estimator a clean view of two faces it would not
        # have in practice.
        dropout = rng.random(depth.shape) < np.clip(0.35 - incidence, 0.0, 1.0)
        depth = np.where(dropout, np.nan, depth)
        # 1 mm output quantisation, as the 16-bit millimetre depth format.
        depth = np.round(depth * 1000.0) / 1000.0

    return DepthImage(depth=depth.reshape(shape), pose=pose, camera=camera, stride=stride)


def median_filter(image: DepthImage, size: int = 3) -> DepthImage:
    """NaN-aware median filter over the depth image.

    Every depth pipeline in production has one of these in front of it, and
    this one is not cosmetic. Without it the belt's noise residue survives the
    plane fit as a sparse but *connected* field of outliers: about 3% of
    60,000 belt points, which works out at roughly 9 mm apart, and Euclidean
    clustering at any radius large enough to hold a part face together then
    percolates straight through it and reports the whole region of interest as
    one 400 mm "part". The filter knocks the residue down by roughly an order
    of magnitude and removes the salt-and-pepper entirely.

    Implemented as a stack of shifted copies rather than with
    ``scipy.ndimage.median_filter``, which has no NaN-aware mode: it would
    propagate a single dropout across the whole window.
    """
    depth = image.depth
    half = size // 2
    padded = np.pad(depth, half, mode="edge")
    windows = [
        padded[row : row + depth.shape[0], column : column + depth.shape[1]]
        for row in range(size)
        for column in range(size)
    ]
    stacked = np.stack(windows)
    all_nan = np.all(np.isnan(stacked), axis=0)
    with warnings.catch_warnings():
        # A window that is entirely dropout is an all-NaN slice. That is a
        # legitimate state -- the camera saw nothing there -- and it is handled
        # on the next line, so the warning is noise.
        warnings.simplefilter("ignore", RuntimeWarning)
        filtered = np.nanmedian(stacked, axis=0)
    filtered = np.where(all_nan, np.nan, filtered)
    # A pixel the sensor did not return stays unreturned: filling it in from
    # its neighbours would invent surface where the camera saw none.
    filtered = np.where(np.isnan(depth), np.nan, filtered)
    return DepthImage(depth=filtered, pose=image.pose, camera=image.camera, stride=image.stride)


def deproject(image: DepthImage, extrinsic_pose: Optional[np.ndarray] = None) -> np.ndarray:
    """Depth image to an (N, 3) world point cloud.

    ``extrinsic_pose`` is the optical frame the caller *believes* the camera was
    at. Leaving it None uses the true pose, which is only correct for testing
    the geometry; the cycle passes the pose implied by the calibrated hand-eye
    transform, so a calibration error shows up as a systematically displaced
    cloud, which is precisely how it shows up in a real cell.
    """
    pose = image.pose if extrinsic_pose is None else np.asarray(extrinsic_pose, dtype=float)
    directions = ray_directions(image.camera, image.stride)
    depth = image.depth
    finite = np.isfinite(depth)
    points = directions[finite] * depth[finite][:, None]
    return points @ pose[:3, :3].T + pose[:3, 3]


def project(points: np.ndarray, pose: np.ndarray, camera: CameraSpec) -> np.ndarray:
    """World points to (u, v) pixels for a camera at ``pose``.

    Points behind the camera come back as NaN rather than as a mirrored
    projection through the optical centre.
    """
    from cell_common.transforms import invert

    local = np.atleast_2d(points) @ invert(pose)[:3, :3].T + invert(pose)[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = camera.fx * local[:, 0] / local[:, 2] + camera.cx
        v = camera.fy * local[:, 1] / local[:, 2] + camera.cy
    behind = local[:, 2] <= 1e-6
    return np.where(behind[:, None], np.nan, np.column_stack([u, v]))


def visible_boxes(scene_boxes: Sequence[Box], part_box: Optional[Box] = None) -> List[Box]:
    """The boxes a render should include: the cell plus the part."""
    boxes = list(scene_boxes)
    if part_box is not None:
        boxes = [box for box in boxes if box.name != part_box.name] + [part_box]
    return boxes
