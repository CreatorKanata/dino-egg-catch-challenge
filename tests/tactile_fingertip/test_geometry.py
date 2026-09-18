"""Regression checks for the one-piece cavity, TPU walls, pause access and exports."""

from dataclasses import replace
from math import cos, sin, tau
from pathlib import Path
import tempfile
import unittest

from tactile_fingertip.config import DEFAULT
from tactile_fingertip.generate import generate
from tactile_fingertip.geometry import at_pause, cylinder, make_fingertip, slab
from tactile_fingertip.validation import validate_geometry


class FingertipTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.body = make_fingertip(DEFAULT)

    def test_one_solid_with_one_closed_cavity(self):
        report = validate_geometry(DEFAULT, self.body)
        self.assertEqual(report["cad_solids"], 1)
        self.assertEqual(report["cad_surface_shells"], 2)
        for actual, expected in zip(report["size_mm"], (20, 20, 10)):
            self.assertAlmostEqual(actual, expected)

    def test_exact_magnet_cavity_and_floor_roof(self):
        c, shape = DEFAULT, self.body.val()
        self.assertAlmostEqual(c.pocket_diameter, 3)
        self.assertAlmostEqual(c.pocket_depth, 1.5)
        for i in range(16):
            a = tau * i / 16
            self.assertFalse(shape.isInside((1.49 * cos(a), 1.49 * sin(a), c.magnet_center_z)))
            self.assertTrue(shape.isInside((1.51 * cos(a), 1.51 * sin(a), c.magnet_center_z)))
        self.assertTrue(shape.isInside((0, 0, c.contact_skin - 0.01)))
        self.assertFalse(shape.isInside((0, 0, c.contact_skin + 0.01)))
        self.assertFalse(shape.isInside((0, 0, c.pause_z - 0.01)))
        self.assertTrue(shape.isInside((0, 0, c.pause_z + 0.01)))
        self.assertTrue(shape.isInside((0, 0, c.enclosure_top - 0.01)))
        self.assertFalse(shape.isInside((0, 0, c.enclosure_top + 0.01)))

    def test_radial_wall_is_point_eight_away_from_rib_junctions(self):
        c, shape = DEFAULT, self.body.val()
        for sign in (-1, 1):
            self.assertTrue(shape.isInside((sign * 2.29 / 2**0.5, 2.29 / 2**0.5, c.magnet_center_z)))
            self.assertFalse(shape.isInside((sign * 2.31 / 2**0.5, 2.31 / 2**0.5, c.magnet_center_z)))
        self.assertAlmostEqual((c.enclosure_diameter - c.pocket_diameter) / 2, 0.8)

    def test_pause_is_accessible_before_roof(self):
        c = DEFAULT
        self.assertAlmostEqual(c.pause_z, 2.3)
        self.assertEqual(round(c.pause_z / c.layer_height), 23)
        pre = at_pause(c, self.body).val()
        magnet_path = cylinder(c.magnet_diameter, c.height, c.contact_skin).val()
        self.assertLess(pre.intersect(magnet_path).Volume(), c.geometry_tolerance)
        # The same vertical path must hit the sealed roof in the finished part.
        self.assertGreater(self.body.val().intersect(magnet_path).Volume(), 0)

    def test_both_faces_are_continuous(self):
        c, shape = DEFAULT, self.body.val()
        for z, thickness in ((0, c.contact_skin), (c.height-c.mounting_skin, c.mounting_skin)):
            probe = slab(c.width, c.depth, thickness, z).val()
            self.assertAlmostEqual(shape.intersect(probe).Volume(), probe.Volume(), places=6)

    def test_depth_ribs_and_internal_connections(self):
        c, shape = DEFAULT, self.body.val()
        # Both directions carry ribs at mid-height, independent of the skins.
        for point in ((4, 0, c.cell_center_z), (0, 4, c.cell_center_z)):
            self.assertTrue(shape.isInside(point))
        core = self.body.intersect(slab(c.width, c.depth,
                    c.land_bottom-c.enclosure_top, c.enclosure_top)).val()
        self.assertEqual(len(core.Solids()), 1)
        rotated = self.body.rotate((0, 0, 0), (0, 0, 1), 90).val()
        self.assertLess(shape.cut(rotated).Volume(), c.geometry_tolerance)

    def test_every_layer_prefix_remains_connected_to_bed(self):
        c = DEFAULT
        for layer in range(1, round(c.height / c.layer_height) + 1):
            with self.subTest(layer=layer):
                partial = self.body.intersect(slab(c.width, c.depth, layer * c.layer_height)).val()
                self.assertEqual(len(partial.Solids()), 1)

    def test_invalid_and_misaligned_parameters_fail(self):
        invalid = ({"pocket_wall": 0.4}, {"pocket_wall": 1.1}, {"magnet_height": 6},
                   {"layer_height": 0.2}, {"strut": 5}, {"width": 10},
                   {"rib_count": 2.5}, {"height": float("nan")}, {"rib_count": True},
                   {"diameter_compensation": -0.1}, {"sensor_center_distance": 1})
        for args in invalid:
            with self.subTest(args=args), self.assertRaises(ValueError):
                replace(DEFAULT, **args).validate()

    def test_wall_and_fit_variants(self):
        for wall in (0.5, 1.0):
            c = replace(DEFAULT, pocket_wall=wall, diameter_compensation=0.1)
            with self.subTest(wall=wall):
                report = validate_geometry(c, make_fingertip(c))
                self.assertEqual(report["cad_solids"], 1)

    def test_export_has_one_material_solid_and_negative_cavity_surface(self):
        with tempfile.TemporaryDirectory() as directory:
            report = generate(DEFAULT, Path(directory))
            self.assertTrue(report["geometry"]["stl_watertight"])
            self.assertEqual(report["geometry"]["stl_surface_shells"], 2)
            self.assertLess(report["geometry"]["surface_signed_volumes_mm3"][0], 0)
            self.assertEqual({p.name for p in Path(directory).iterdir()},
                             {"fingertip.stl", "validation.json"})
            self.assertEqual(len(list(Path(directory).glob("*.stl"))), 1)


if __name__ == "__main__":
    unittest.main()
