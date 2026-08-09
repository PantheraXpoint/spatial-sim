"""
A synthetic observation source. Layer 3.

Reads config/sensors.yaml and config/scene.yaml, walks an avatar around a
circuit, and emits `Observation` objects with plausible shapes and payloads
that change as the avatar approaches. No simulator, no GPU, no stage.

WHAT THIS IS FOR. The simulator side of this project is the long pole: assets,
viewports, RTX profiles, semantic labels. Everything in Layers 3 and 4 -- the
memory module, the residual loop, the fusion code that is the actual research
-- would otherwise have to wait for it. It does not have to. This source
produces the same type against the same protocol, so all of that can be
written, run and tested on a laptop today, and repointed at the live simulator
when sim/observation_adapter.py lands (server task S11).

WHAT THIS IS NOT. It is not a renderer and not a physics engine. There is no
occlusion, no material response, no beam divergence, no multipath. What it
models is the one relationship the demo rests on: *readings change when the
avatar moves, and change more the closer it gets*. Anything that needs more
fidelity than that is testing the mock rather than the code under test.

Determinism: with `noise=0.0` (the default) two ticks with the world in the
same state produce bit-identical payloads. That is what lets a residual test
assert "nothing moved -> residual is zero" and mean it. Turn `noise` up to
shake out consumers that quietly assume exactness.

NOTHING IN THIS MODULE MAY IMPORT omni, pxr, OR isaacsim.
"""

from __future__ import annotations

import math
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from core.observation import (
    ANNOTATOR_DATA_KEYS,
    MODALITY_DATA_KEYS,
    Modality,
    Observation,
    Pose,
)
from core.registry import RANGE_MODALITIES, SensorRegistry, SensorSpec

_DEFAULT_DT = 1.0 / 30.0

# The avatar circuit. Not declared in scene.yaml, because it is a property of
# this fake and not of the scene: the real avatar is keyboard-driven and has no
# trajectory at all. Centre and radius are chosen so the path passes directly
# under INFRA_01 at (5, 0) at its nearest and ~8 m away at its farthest, and
# threads between the three robot platforms at (4, 2), (6, -1) and (8, 1.5).
# INFRA_02 is 60 m away in the second building and never sees the avatar --
# that is deliberate. It gives every consumer a genuinely static station to
# compare against, which is the control case for a change detector.
_PATH_CENTRE = (9.0, 0.0)
_PATH_RADIUS = 4.0

_CAMERA_HFOV_DEG = 70.0
# Cameras have a near plane. Without one, the avatar's own first-person camera
# sits inside the avatar and every frame is a wall of skin-coloured pixels.
_NEAR_CLIP_M = 1.0
_CAMERA_MAX_RANGE_M = 60.0

# Coarse beam geometry, in the ballpark of Example_Rotary / Example_Radar.
# Only the ratios matter: they set how fast returns fall off with distance.
_LIDAR = {
    "max_range_m": 30.0,
    "h_res_deg": 0.4,
    "v_res_deg": 1.0,
    "static_points": 1024,
}
_RADAR = {
    "max_range_m": 50.0,
    "h_res_deg": 1.5,
    "v_res_deg": 5.0,
    "static_points": 6,
}
# One object cannot return more than this many points, however close it gets.
# A real rotary lidar has a finite beam count too.
_MAX_OBJECT_POINTS = 4096

_AVATAR = "avatar"

# The warehouse's static contents: pallets and stacked crates near the circuit.
# Not cosmetic. Without them the avatar's own cameras see nothing but backdrop,
# every first-person frame is identical, and a broken avatar camera is
# indistinguishable from a working one pointed at an empty room. They are also
# what makes "static scene" mean something more than "empty scene" for a change
# detector. Positions are the mock's own -- scene.yaml declares no props.
_DEFAULT_PROPS: tuple[tuple[str, tuple[float, float, float]], ...] = (
    ("pallet_a", (12.5, 3.0, 0.0)),
    ("pallet_b", (13.0, -1.5, 0.0)),
    ("crate_stack_a", (5.5, -3.0, 0.0)),
    ("crate_stack_b", (5.0, 3.5, 0.0)),
)


# --- The little world --------------------------------------------------------


@dataclass(frozen=True)
class AvatarProfile:
    """The moving entity, as declared in config/scene.yaml."""

    prim_path: str
    speed: float = 1.4
    height: float = 1.75
    eye_height: float = 1.65
    radius: float = 0.35


@dataclass
class MockObject:
    """
    Something sensors can return off. The avatar is one of these, and so is
    anything a test puts in the world to make it change underneath a consumer.
    """

    name: str
    position: tuple[float, float, float]
    radius: float = 0.35
    half_height: float = 0.5
    label: str = "object"
    semantic_id: int = 2
    moving: bool = False

    @property
    def centre(self) -> np.ndarray:
        """Body centre. `position` is on the floor, like everything in USD."""
        x, y, z = self.position
        return np.array([x, y, z + self.half_height], dtype=np.float64)


@dataclass(frozen=True)
class CircuitPath:
    """
    A constant-speed circle. Constant speed matters more than an interesting
    shape: `move_speed` from scene.yaml is then honoured exactly, so distance
    over time is something a test can reason about in closed form.
    """

    centre: tuple[float, float] = _PATH_CENTRE
    radius: float = _PATH_RADIUS
    speed: float = 1.4

    def at(self, t: float) -> tuple[tuple[float, float, float], float]:
        """Floor position and heading (radians, +x is 0) at time `t`."""
        theta = (self.speed / self.radius) * t
        x = self.centre[0] + self.radius * math.cos(theta)
        y = self.centre[1] + self.radius * math.sin(theta)
        return (x, y, 0.0), theta + math.pi / 2.0  # heading follows the tangent


# --- Geometry ----------------------------------------------------------------


def _yaw_quaternion(heading: float) -> tuple[float, float, float, float]:
    """(w, x, y, z) for a rotation of `heading` about +z."""
    return (math.cos(heading / 2.0), 0.0, 0.0, math.sin(heading / 2.0))


def _heading_of(pose: Pose) -> float:
    w, _, _, z = pose.orientation
    return 2.0 * math.atan2(z, w)


def _quaternion_from_matrix(m: np.ndarray) -> tuple[float, float, float, float]:
    """(w, x, y, z) from a 3x3 rotation matrix, branching on the largest term."""
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s,
             (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s,
             (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
             0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
             (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _beams_on_target(distance: float, radius: float, half_height: float,
                     h_res_deg: float, v_res_deg: float) -> int:
    """
    How many beams land on an object of this size at this range.

    Angular width times angular height over the beam grid. Falls off as
    1/distance^2 for anything much further away than it is wide, which is the
    single property every consumer of this mock actually depends on.
    """
    if distance <= 1e-3:
        return 0
    width_deg = 2.0 * math.degrees(math.atan(radius / distance))
    height_deg = 2.0 * math.degrees(math.atan(half_height / distance))
    count = (width_deg / h_res_deg) * (height_deg / v_res_deg)
    return min(round(count), _MAX_OBJECT_POINTS)


@dataclass(frozen=True)
class _CameraRig:
    """A pinhole camera aimed at a point. Enough to project a blob."""

    position: np.ndarray
    right: np.ndarray
    up: np.ndarray
    forward: np.ndarray
    width: int
    height: int
    focal_px: float

    def project(self, point: np.ndarray) -> tuple[float, float, float] | None:
        """Pixel (u, v) and axial depth, or None if the point is behind us."""
        d = point - self.position
        depth = float(d @ self.forward)
        if depth <= _NEAR_CLIP_M:
            return None
        u = self.width / 2.0 + self.focal_px * float(d @ self.right) / depth
        v = self.height / 2.0 - self.focal_px * float(d @ self.up) / depth
        return u, v, depth

    @property
    def quaternion(self) -> tuple[float, float, float, float]:
        # USD camera convention: the camera looks down its local -Z with +Y up,
        # so the third basis column is -forward rather than forward.
        return _quaternion_from_matrix(
            np.column_stack([self.right, self.up, -self.forward])
        )


def _look_at(position: np.ndarray, target: np.ndarray,
             width: int, height: int) -> _CameraRig:
    forward = target - position
    norm = float(np.linalg.norm(forward))
    # A camera with nothing to look at points down +x rather than dividing by
    # zero. Only reachable if a mount sits exactly on the aim point.
    forward = forward / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0])
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(float(forward @ world_up)) > 0.999:
        # Straight down -- the ceiling station. +z is useless as a reference.
        world_up = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, world_up)
    right = right / float(np.linalg.norm(right))
    up = np.cross(right, forward)
    focal_px = (width / 2.0) / math.tan(math.radians(_CAMERA_HFOV_DEG) / 2.0)
    return _CameraRig(position, right, up, forward, width, height, focal_px)


# --- Reading the config ------------------------------------------------------


@dataclass(frozen=True)
class _Scene:
    """The three things this module needs out of config/scene.yaml."""

    mounts: dict[str, tuple[float, float, float]]   # by prim path
    robots: dict[str, tuple[float, float, float]]   # by id -- see _resolve_mounts
    avatar: AvatarProfile


def _load_scene(scene_path: Path) -> _Scene:
    raw = yaml.safe_load(scene_path.read_text()) or {}

    mounts: dict[str, tuple[float, float, float]] = {}
    for station in raw.get("stations") or []:
        if "prim_path" in station and "position" in station:
            mounts[station["prim_path"]] = _xyz(station["position"],
                                                station.get("id"))

    robots: dict[str, tuple[float, float, float]] = {}
    for robot in raw.get("robots") or []:
        if "id" in robot and "position" in robot:
            robots[robot["id"]] = _xyz(robot["position"], robot["id"])

    avatar_raw = raw.get("avatar") or {}
    if "prim_path" not in avatar_raw:
        raise ValueError(
            f"{scene_path}: avatar has no prim_path, so nothing can be attached "
            f"to the only moving entity in the scene."
        )
    height = float(avatar_raw.get("height", 1.75))
    avatar = AvatarProfile(
        prim_path=avatar_raw["prim_path"],
        speed=float(avatar_raw.get("move_speed", 1.4)),
        height=height,
        eye_height=float(avatar_raw.get("eye_height", height * 0.94)),
    )
    return _Scene(mounts, robots, avatar)


def _xyz(value: Any, name: object) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"position for '{name}' must be [x, y, z], got {value!r}")
    return (float(value[0]), float(value[1]), float(value[2]))


def _resolve_mounts(registry: SensorRegistry,
                    scene: _Scene) -> dict[str, tuple[float, float, float]]:
    """
    Give every parent Xform in the registry a world position.

    Station paths come straight out of scene.yaml. Robot mounts cannot: a
    robots: entry has an `id`, not a prim path, and hard rule 1 forbids
    inventing one. So a robot position is attached to a parent path only when
    that path was already written down by a human in sensors.yaml and its last
    component matches the robot id. The path is still the human's; only the
    position comes from the robots: block.

    Anything left unplaced is an error by name. A mock that quietly dropped
    un-placeable sensors to the origin would stack three stations on top of
    each other and every reading would still look plausible.
    """
    resolved = dict(scene.mounts)
    unplaced = []
    for spec in registry:
        parent = spec.parent
        if parent is None:
            raise ValueError(
                f"{spec.sensor_id}: no parent Xform, so the mock has nowhere to "
                f"put it. Give it a parent in config/sensors.yaml."
            )
        if parent == scene.avatar.prim_path or parent in resolved:
            continue
        robot_id = parent.rsplit("/", 1)[-1]
        if robot_id in scene.robots:
            resolved[parent] = scene.robots[robot_id]
        else:
            unplaced.append(f"{spec.sensor_id} (parent {parent})")
    if unplaced:
        raise ValueError(
            "config/scene.yaml has no position for: "
            + ", ".join(sorted(unplaced))
            + ". Add a stations: or robots: entry -- otherwise the mock would "
            "have to invent a pose."
        )
    return resolved


# --- The source --------------------------------------------------------------


class MockObservationSource:
    """
    Satisfies `core.observation.ObservationSource` with no simulator present.

    Used exactly the way the real thing will be::

        source = MockObservationSource.from_config()
        for _ in range(100):
            for obs in source.step():
                consume(obs)

    Beyond the protocol it offers `place_object`, `move_object`,
    `remove_object` and `reset`. Those exist so a consumer can be shown a world
    that changed without it having caused the change -- which is the whole
    content of a prediction residual (task M5). Nothing written against the
    protocol may use them, or it stops running against the live simulator.
    """

    def __init__(
        self,
        registry: SensorRegistry,
        mount_positions: dict[str, tuple[float, float, float]],
        avatar: AvatarProfile,
        *,
        path: CircuitPath | None = None,
        dt: float = _DEFAULT_DT,
        seed: int = 0,
        noise: float = 0.0,
        props: bool = True,
    ) -> None:
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")
        self._registry = registry
        self._avatar = avatar
        self._path = path or CircuitPath(_PATH_CENTRE, _PATH_RADIUS, avatar.speed)
        self._dt = dt
        self._seed = seed
        self._noise = float(noise)
        self._rng = np.random.default_rng(seed)
        self._t = 0.0
        self._mounts = dict(mount_positions)
        self._specs = list(registry)

        for spec in self._specs:
            if spec.parent != avatar.prim_path and spec.parent not in self._mounts:
                raise ValueError(
                    f"{spec.sensor_id}: no world position for its parent "
                    f"'{spec.parent}'."
                )

        self._objects: dict[str, MockObject] = {}
        self._rigs: dict[str, _CameraRig] = {}
        self._backdrops: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self._clutter: dict[str, np.ndarray] = {}
        self._scatter: dict[str, np.ndarray] = {}
        self._grids: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}
        self._place_avatar()
        if props:
            for name, position in _DEFAULT_PROPS:
                self.place_object(name, position, radius=0.6, half_height=0.6,
                                  label="prop")

    @classmethod
    def from_config(
        cls,
        sensors_path: str | Path = "config/sensors.yaml",
        scene_path: str | Path | None = None,
        **kwargs: Any,
    ) -> MockObservationSource:
        """Built from the same two files sim/ will build the real scene from."""
        sensors_path = Path(sensors_path)
        if scene_path is None:
            scene_path = sensors_path.parent / "scene.yaml"
        scene_path = Path(scene_path)
        if not scene_path.exists():
            raise FileNotFoundError(f"scene file not found: {scene_path}")
        registry = SensorRegistry.from_yaml(sensors_path, scene_path)
        scene = _load_scene(scene_path)
        return cls(registry, _resolve_mounts(registry, scene), scene.avatar, **kwargs)

    # --- ObservationSource ---------------------------------------------------

    @property
    def sensor_ids(self) -> tuple[str, ...]:
        return tuple(s.sensor_id for s in self._specs)

    @property
    def time(self) -> float:
        return self._t

    def step(self, dt: float | None = None) -> list[Observation]:
        self._t += self._dt if dt is None else dt
        self._place_avatar()
        return [self._observe(spec) for spec in self._specs]

    def close(self) -> None:
        """Nothing to release. Present so consumers can be written once."""

    # --- Beyond the protocol: driving the world ------------------------------

    @property
    def registry(self) -> SensorRegistry:
        return self._registry

    @property
    def avatar_pose(self) -> Pose:
        """Ground truth. The live source will not be able to offer this."""
        position, heading = self._path.at(self._t)
        return Pose(position, _yaw_quaternion(heading))

    def objects(self) -> dict[str, MockObject]:
        return dict(self._objects)

    def place_object(self, name: str, position: tuple[float, float, float],
                     **kwargs: Any) -> MockObject:
        """Put something in the world for sensors to start returning off."""
        if name == _AVATAR:
            raise ValueError(
                f"'{_AVATAR}' is driven by the trajectory -- use another name, "
                f"or pass a different CircuitPath."
            )
        obj = MockObject(name=name, position=position, **kwargs)
        self._objects[name] = obj
        return obj

    def move_object(self, name: str,
                    position: tuple[float, float, float]) -> MockObject:
        if name not in self._objects:
            raise KeyError(f"no object '{name}'. Known: {sorted(self._objects)}")
        self._objects[name].position = position
        return self._objects[name]

    def remove_object(self, name: str) -> None:
        if name == _AVATAR:
            raise ValueError("the avatar cannot be removed; it is the demo")
        self._objects.pop(name, None)

    def reset(self) -> None:
        """Back to t=0 with the seed re-drawn. Placed objects survive."""
        self._t = 0.0
        self._rng = np.random.default_rng(self._seed)
        self._place_avatar()

    # --- Internals -----------------------------------------------------------

    def _place_avatar(self) -> None:
        position, _ = self._path.at(self._t)
        self._objects[_AVATAR] = MockObject(
            name=_AVATAR,
            position=position,
            radius=self._avatar.radius,
            half_height=self._avatar.height / 2.0,
            label="person",
            semantic_id=1,
            moving=True,
        )

    def _is_avatar_mounted(self, spec: SensorSpec) -> bool:
        return spec.parent == self._avatar.prim_path

    def _sensor_pose(self, spec: SensorSpec) -> Pose:
        if self._is_avatar_mounted(spec):
            (x, y, _), heading = self._path.at(self._t)
            if spec.sensor_id.endswith("_TP"):
                # Third person trails the head. Co-location is a rule about
                # stations; the two cameras on the avatar are deliberately not
                # co-located, or third person would not be a viewpoint.
                x -= 2.0 * math.cos(heading)
                y -= 2.0 * math.sin(heading)
                return Pose((x, y, self._avatar.eye_height + 0.6),
                            _yaw_quaternion(heading))
            return Pose((x, y, self._avatar.eye_height), _yaw_quaternion(heading))

        position = self._mounts[spec.parent]
        if spec.modality in RANGE_MODALITIES:
            # A rotary sensor sweeps the whole azimuth, so it has no meaningful
            # heading to report: the mount frame is the sensor frame.
            return Pose(position, (1.0, 0.0, 0.0, 0.0))
        return Pose(position, self._static_rig(spec).quaternion)

    def _static_rig(self, spec: SensorSpec) -> _CameraRig:
        """
        A fixed camera, aimed at the middle of the avatar circuit.

        The aim is the mock's invention -- sensors.yaml declares no orientation,
        and a per-sensor orientation invented here would just be noise. Aiming
        every camera at one point at least makes the avatar traverse every
        frame, which is what a consumer needs to see.
        """
        rig = self._rigs.get(spec.sensor_id)
        if rig is None:
            width, height = spec.resolution or (640, 480)
            rig = _look_at(
                np.array(self._mounts[spec.parent], dtype=np.float64),
                np.array([_PATH_CENTRE[0], _PATH_CENTRE[1], 0.9]),
                width,
                height,
            )
            self._rigs[spec.sensor_id] = rig
        return rig

    def _grid(self, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
        """Cached pixel index grids -- the blob mask is the hot loop."""
        key = (width, height)
        if key not in self._grids:
            self._grids[key] = np.meshgrid(
                np.arange(width, dtype=np.float32),
                np.arange(height, dtype=np.float32),
            )
        return self._grids[key]

    def _visible(self, position: np.ndarray, max_range: float,
                 moving_only: bool = False) -> list[tuple[MockObject, float]]:
        """In range, nearest first. No occlusion model -- see the module header."""
        out = []
        for obj in self._objects.values():
            if moving_only and not obj.moving:
                continue
            distance = float(np.linalg.norm(obj.centre - position))
            if distance <= max_range:
                out.append((obj, distance))
        return sorted(out, key=lambda pair: pair[1])

    def _observe(self, spec: SensorSpec) -> Observation:
        pose = self._sensor_pose(spec)
        if spec.modality in RANGE_MODALITIES:
            data, intrinsics = self._range_payload(spec, pose)
        else:
            data, intrinsics = self._camera_payload(spec, pose)
        return Observation(
            sensor_id=spec.sensor_id,
            timestamp=self._t,
            modality=spec.modality,
            mount=spec.mount,
            pose=pose,
            intrinsics=intrinsics,
            data=data,
        )

    def _payload_keys(self, spec: SensorSpec) -> set[str]:
        """
        The arrays this sensor owes: its modality's minimum plus whatever its
        declared annotators add. Exactly the rule the live adapter has to
        follow, which is why the contract suite can check both the same way.
        """
        keys = set(MODALITY_DATA_KEYS[spec.modality])
        keys.update(ANNOTATOR_DATA_KEYS[a] for a in spec.annotators)
        return keys

    # --- Range payloads ------------------------------------------------------

    def _range_payload(self, spec: SensorSpec,
                       pose: Pose) -> tuple[dict[str, Any], dict[str, Any]]:
        radar = spec.modality is Modality.RADAR
        profile = _RADAR if radar else _LIDAR
        origin = np.array(pose.position, dtype=np.float64)

        clutter = self._static_cloud(spec, profile)
        clouds = [clutter]
        # Radar returns what moves. That is the honest difference between the
        # two range modalities here, and it is why a radar frame is tiny.
        for obj, distance in self._visible(origin, profile["max_range_m"],
                                           moving_only=radar):
            n = _beams_on_target(distance, obj.radius, obj.half_height,
                                 profile["h_res_deg"], profile["v_res_deg"])
            if n <= 0:
                continue
            spread = np.array([obj.radius, obj.radius, obj.half_height])
            clouds.append((obj.centre - origin)
                          + self._scatter_pattern(spec, n) * spread)

        points = np.concatenate(clouds).astype(np.float32)
        if self._noise:
            points = points + self._rng.normal(
                0.0, self._noise, points.shape
            ).astype(np.float32)
        ranges = np.linalg.norm(points, axis=1).astype(np.float32)

        data: dict[str, Any] = {
            "points": points,          # sensor-local metres
            "ranges": ranges,
            "num_returns": int(points.shape[0]),
        }
        if radar:
            # Clutter is stationary by construction; everything after it came
            # off a moving object, so the split is exactly the clutter length.
            radial = np.zeros(points.shape[0], dtype=np.float32)
            radial[len(clutter):] = self._path.speed
            data["radial_velocities"] = radial
            data["rcs"] = np.full(points.shape[0], 10.0, dtype=np.float32)
        else:
            data["intensities"] = np.clip(
                1.0 / np.maximum(ranges, 0.5) ** 2, 0.0, 1.0
            ).astype(np.float32)

        intrinsics = {
            "config": spec.config,
            "max_range_m": profile["max_range_m"],
            "horizontal_resolution_deg": profile["h_res_deg"],
            "vertical_resolution_deg": profile["v_res_deg"],
        }
        return data, intrinsics

    def _static_cloud(self, spec: SensorSpec, profile: dict) -> np.ndarray:
        """
        The unchanging part of the scene: walls, racking, floor.

        Generated once per sensor and cached, so an unchanged world gives a
        bit-identical background. Without that, "nothing moved" and "everything
        moved a little" would look the same to a residual.
        """
        key = spec.sensor_id
        if key not in self._clutter:
            rng = self._seeded_rng(f"clutter:{key}")
            n = profile["static_points"]
            azimuth = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
            elevation = rng.uniform(-0.25, 0.05, n)
            distance = rng.uniform(4.0, min(18.0, profile["max_range_m"]), n)
            self._clutter[key] = np.column_stack([
                distance * np.cos(elevation) * np.cos(azimuth),
                distance * np.cos(elevation) * np.sin(azimuth),
                distance * np.sin(elevation),
            ])
        return self._clutter[key]

    def _scatter_pattern(self, spec: SensorSpec, n: int) -> np.ndarray:
        """
        Unit scatter for points on an object, cached per sensor.

        Drawing these fresh each tick would make a *stationary* object's cloud
        jitter, and every change detector would then see change everywhere.
        The pattern is fixed; only how much of it is used varies with range.
        """
        pattern = self._scatter.get(spec.sensor_id)
        if pattern is None:
            rng = self._seeded_rng(f"scatter:{spec.sensor_id}")
            pattern = rng.normal(0.0, 0.4, size=(_MAX_OBJECT_POINTS, 3))
            self._scatter[spec.sensor_id] = pattern
        return pattern[:n]

    @staticmethod
    def _seeded_rng(key: str) -> np.random.Generator:
        # crc32, not hash(): str hashing is salted per process, so hash() would
        # make the "static" scene differ between runs of the same test.
        return np.random.default_rng(zlib.crc32(key.encode()))

    # --- Camera payloads -----------------------------------------------------

    def _camera_payload(self, spec: SensorSpec,
                        pose: Pose) -> tuple[dict[str, Any], dict[str, Any]]:
        width, height = spec.resolution or (640, 480)
        if self._is_avatar_mounted(spec):
            heading = _heading_of(pose)
            x, y, _ = pose.position
            rig = _look_at(
                np.array(pose.position, dtype=np.float64),
                np.array([x + 10.0 * math.cos(heading),
                          y + 10.0 * math.sin(heading), 0.9]),
                width,
                height,
            )
        else:
            rig = self._static_rig(spec)

        wanted = self._payload_keys(spec)
        rgb, depth, semantic = self._backdrop(spec, rig, wanted)
        xx, yy = self._grid(width, height)

        for obj, distance in self._visible(rig.position, _CAMERA_MAX_RANGE_M):
            if distance < _NEAR_CLIP_M:
                continue    # inside the near plane -- e.g. your own body
            projected = rig.project(obj.centre)
            if projected is None:
                continue
            u, v, axial = projected
            rx = rig.focal_px * obj.radius / axial
            ry = rig.focal_px * obj.half_height / axial
            if rx < 0.5 or ry < 0.5:
                continue    # sub-pixel: too far to register at all
            mask = ((xx - u) / rx) ** 2 + ((yy - v) / ry) ** 2 <= 1.0
            if not mask.any():
                continue    # off-frame
            if rgb is not None:
                rgb[mask] = _label_colour(obj.semantic_id)
            if depth is not None:
                # Euclidean range from the sensor origin, per DEPTH_CONVENTION
                # in core/observation.py -- `distance` and not the axial
                # `axial` computed above. One value per object rather than per
                # pixel: coarse, but coarse in the direction of the convention.
                depth[mask] = np.float32(distance)
            if semantic is not None:
                semantic[mask] = np.uint8(obj.semantic_id)

        data: dict[str, Any] = {}
        if rgb is not None:
            if self._noise:
                grain = self._rng.normal(0.0, self._noise * 255.0, rgb.shape)
                rgb = np.clip(rgb.astype(np.int16) + grain, 0, 255).astype(np.uint8)
            data["rgb"] = rgb
        if depth is not None:
            if self._noise:
                depth = depth + self._rng.normal(
                    0.0, self._noise, depth.shape
                ).astype(np.float32)
            data["depth"] = depth
        if semantic is not None:
            data["semantic"] = semantic
            data["semantic_labels"] = dict(_SEMANTIC_LABELS)

        intrinsics = {
            "width": width,
            "height": height,
            "horizontal_fov_deg": _CAMERA_HFOV_DEG,
            "focal_length_px": rig.focal_px,
        }
        return data, intrinsics

    def _backdrop(self, spec: SensorSpec, rig: _CameraRig,
                  wanted: set[str]) -> tuple[Any, Any, Any]:
        """
        The static scene behind the objects: cached per sensor, then copied.

        Rebuilding a 1280x720 gradient every tick for eight cameras is the one
        thing in here that would genuinely be slow.
        """
        key = spec.sensor_id
        if key not in self._backdrops:
            rows = np.linspace(0.0, 1.0, rig.height, dtype=np.float32)[:, None]
            grey = (40.0 + 60.0 * rows).astype(np.uint8)
            rgb = np.repeat(
                np.repeat(grey, rig.width, axis=1)[:, :, None], 3, axis=2
            )
            # How far the far wall sits behind the aim point. One constant
            # across the frame, which under euclidean depth is a dome around
            # the camera rather than a flat wall -- correct for the
            # convention, and not worth more geometry than that.
            standoff = float(np.linalg.norm(
                np.array([_PATH_CENTRE[0], _PATH_CENTRE[1], 0.9]) - rig.position
            )) + 6.0
            self._backdrops[key] = (
                rgb,
                np.full((rig.height, rig.width), standoff, dtype=np.float32),
                np.zeros((rig.height, rig.width), dtype=np.uint8),
            )
        rgb, depth, semantic = self._backdrops[key]
        return (
            rgb.copy() if "rgb" in wanted else None,
            depth.copy() if "depth" in wanted else None,
            semantic.copy() if "semantic" in wanted else None,
        )


_SEMANTIC_LABELS: dict[int, str] = {0: "BACKGROUND", 1: "person", 2: "object"}


def _label_colour(semantic_id: int) -> tuple[int, int, int]:
    return {1: (220, 170, 120), 2: (120, 200, 220)}.get(semantic_id, (200, 200, 200))
