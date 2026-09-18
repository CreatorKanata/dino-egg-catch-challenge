"""Single-piece fingertip dimensions in print coordinates; contact face is Z=0."""

from dataclasses import asdict, dataclass
from math import isfinite, sqrt
from pathlib import Path


OUTPUT_DIRECTORY = Path(__file__).resolve().parents[2] / "3d-models/tactile-fingertip"


@dataclass(frozen=True)
class Config:
    width: float = 20.0
    depth: float = 20.0
    height: float = 10.0
    contact_skin: float = 0.8
    mounting_skin: float = 0.8
    pocket_wall: float = 0.8
    pocket_roof: float = 0.8
    magnet_diameter: float = 3.0
    magnet_height: float = 1.5
    diameter_compensation: float = 0.0
    depth_compensation: float = 0.0
    cell_pitch: float = 8.0
    cell_count: int = 3
    strut: float = 0.8
    rib_depth: float = 1.2
    rib_pitch: float = 4.0
    rib_count: int = 5
    tie_thickness: float = 0.8
    tip_embed: float = 0.4
    mount_land: float = 2.4
    sensor_center_distance: float = 10.0
    layer_height: float = 0.1
    nozzle: float = 0.4
    stl_tolerance: float = 0.01
    angular_tolerance: float = 0.1
    geometry_tolerance: float = 1e-6
    mesh_volume_relative_tolerance: float = 0.002

    @property
    def pocket_diameter(self):
        return self.magnet_diameter + self.diameter_compensation

    @property
    def pocket_depth(self):
        return self.magnet_height + self.depth_compensation

    @property
    def pause_z(self):
        return self.contact_skin + self.pocket_depth

    @property
    def enclosure_top(self):
        return self.pause_z + self.pocket_roof

    @property
    def enclosure_diameter(self):
        return self.pocket_diameter + 2 * self.pocket_wall

    @property
    def magnet_center_z(self):
        return self.contact_skin + self.magnet_height / 2

    @property
    def sensor_z(self):
        return self.magnet_center_z + self.sensor_center_distance

    @property
    def cell_radius(self):
        return (self.height - self.mounting_skin - self.contact_skin + self.tip_embed) / 2

    @property
    def cell_center_z(self):
        return self.height - self.mounting_skin - self.cell_radius

    @property
    def inner_radius(self):
        return self.cell_radius - sqrt(2) * self.strut

    @property
    def land_bottom(self):
        return self.height - self.mounting_skin - self.mount_land / 2

    @property
    def tie_span(self):
        return (self.rib_count - 1) * self.rib_pitch + self.rib_depth

    @staticmethod
    def centers(count, pitch):
        return tuple((i - (count - 1) / 2) * pitch for i in range(count))

    def validate(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
                raise ValueError(f"{name} must be a finite number")
            minimum = 0 if name.endswith("compensation") else self.geometry_tolerance
            if value < minimum:
                raise ValueError(f"{name} is below its allowed minimum")
        for name in ("rib_count", "cell_count"):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f"{name} must be an integer")
        checks = {
            "square envelope for crossed ribs": abs(self.width - self.depth) < self.geometry_tolerance,
            "mounting skin": 0.5 <= self.mounting_skin <= 1.2,
            "wall between 0.5 and 1.0 mm": 0.5 <= self.pocket_wall <= 1.0,
            "floor and roof between 0.5 and 1.0 mm": (
                0.5 <= self.contact_skin <= 1.0 and 0.5 <= self.pocket_roof <= 1.0),
            "root attachment": self.tip_embed < self.contact_skin,
            "open diamond": self.inner_radius > self.strut,
            "mount land": self.cell_center_z < self.land_bottom < self.height,
            "cell spacing": self.cell_pitch > 2 * self.inner_radius,
            "rib gap": self.rib_pitch > self.rib_depth,
            "rib envelope": (self.rib_count - 1) * self.rib_pitch + self.rib_depth <= self.depth,
            "mount envelope": (self.cell_count - 1) * self.cell_pitch + self.mount_land <= self.width,
            "pocket envelope": self.enclosure_diameter < min(self.width, self.depth),
            "free enclosure": self.enclosure_top < self.cell_center_z,
            "sensor outside TPU": self.sensor_z > self.height,
        }
        for name, ok in checks.items():
            if not ok:
                raise ValueError(f"Invalid configuration: {name}")
        # Pause position must coincide with an actual constant-height layer boundary.
        for value in (self.contact_skin, self.pause_z, self.height):
            if abs(value / self.layer_height - round(value / self.layer_height)) > self.geometry_tolerance:
                raise ValueError("Floor, pause and height must align with layer_height")


DEFAULT = Config()


@dataclass(frozen=True)
class MountConfig:
    """Draft fixture: board outline from owner image; hole locations are estimates."""
    board_width: float = 26.3
    board_depth: float = 26.0
    board_thickness: float = 1.6
    width: float = 34.0
    depth: float = 34.0
    plate: float = 3.0
    standoff: float = 3.0
    boss_diameter: float = 6.0
    hole_pitch: float = 20.0
    hole_y: float = 10.0
    screw_clearance: float = 3.2
    head_diameter: float = 6.2
    chip_active_offset: float = 0.5

    @property
    def hole_centers(self):
        return ((-self.hole_pitch / 2, self.hole_y), (self.hole_pitch / 2, self.hole_y))

    def validate(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
                raise ValueError(f"{name} must be finite")
            if name != "hole_y" and value <= 0:
                raise ValueError(f"{name} must be positive")
        if not self.screw_clearance < self.head_diameter < min(self.width, self.depth):
            raise ValueError("Invalid screw/head diameter")
        if self.boss_diameter <= self.screw_clearance:
            raise ValueError("Boss must surround the screw")
        if (self.head_diameter-self.screw_clearance)/2 >= self.plate:
            raise ValueError("Countersink must leave plate material")
        for x, y in self.hole_centers:
            r = max(self.head_diameter, self.boss_diameter) / 2
            if abs(x)+r >= self.width/2 or abs(y)+r >= self.depth/2:
                raise ValueError("Fastener feature extends outside plate")


MOUNT_DEFAULT = MountConfig()
