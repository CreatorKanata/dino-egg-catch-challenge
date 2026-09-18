"""Check draft PCB mount geometry without claiming verified board fit."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from tactile_fingertip.config import MOUNT_DEFAULT
from tactile_fingertip.geometry import cylinder, slab
from tactile_fingertip.mount import generate_mount, make_mount


class MountTests(unittest.TestCase):
    def test_plate_support_bosses_and_fastener_access(self):
        c = MOUNT_DEFAULT
        shape = make_mount(c).val()
        self.assertTrue(shape.isValid())
        self.assertEqual(len(shape.Solids()), 1)
        self.assertTrue(shape.isInside((0, 0, c.plate/2)))
        for x, y in c.hole_centers:
            access = cylinder(c.screw_clearance, c.plate+c.standoff).translate((x, y, 0)).val()
            self.assertLess(shape.intersect(access).Volume(), 1e-6)
            self.assertTrue(shape.isInside((x+2.5, y, c.plate+c.standoff-0.1)))
        # A flush cushion occupies the opposite half-space from the fixture.
        cushion = slab(20, 20, 10, -10).val()
        self.assertLess(shape.intersect(cushion).Volume(), 1e-6)

    def test_export_preserves_draft_status_and_spacing(self):
        with tempfile.TemporaryDirectory() as directory:
            report = generate_mount(MOUNT_DEFAULT, directory)
            self.assertTrue(report["stl_watertight"])
            self.assertEqual({p.name for p in Path(directory).iterdir()},
                             {"mount-draft.stl", "mount-draft.json"})
            self.assertEqual(report["estimated_parameters"], ["hole_pitch", "hole_y"])
            self.assertAlmostEqual(report["nominal_magnet_to_sensor_mm"], 13.95)

    def test_invalid_mount_is_rejected(self):
        for args in ({"plate": 1}, {"hole_pitch": 40}, {"standoff": -1}):
            with self.assertRaises(ValueError):
                replace(MOUNT_DEFAULT, **args).validate()
