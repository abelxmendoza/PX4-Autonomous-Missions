"""Contract tests for the RealmForge world -> Gazebo SDF adapter."""
import importlib.util
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

SOURCE = Path(__file__).resolve().parents[1] / "tools" / "realmforge_to_sdf.py"
spec = importlib.util.spec_from_file_location("realmforge_to_sdf", SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def world(objects):
    return {"format": "realmforge-world", "version": 1, "objects": objects}


def obj(kind="wall", position=(3, 1, 4), size=(2, 6, 1), yaw=0, **kwargs):
    return {"kind": kind, "position": list(position), "size": list(size),
            "rotationY": yaw, **kwargs}


class AdapterTests(unittest.TestCase):
    def test_axis_mapping_and_base_center(self):
        xml, count = module.convert(world([obj(yaw=math.pi / 2)]))
        self.assertEqual(count, 1)
        root = ET.fromstring(xml)
        model = root.find("./world/model[@name='obstacle_0']")
        self.assertIsNotNone(model)
        pose = [float(x) for x in model.find("./link/pose").text.split()]
        for actual, expected in zip(pose, [3, -4, 4, 0, 0, math.pi / 2]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(model.find("./link/collision/geometry/box/size").text, "2 1 6")

    def test_obstacle_policy(self):
        objects = [obj("wall"), obj("crate"), obj("crate", solid=True),
                   obj("wall", solid=False), obj("platform", size=(2, 0.2, 2)),
                   obj("platform", size=(2, 0.5, 2))]
        _, count = module.convert(world(objects))
        self.assertEqual(count, 3)

    def test_bad_inputs_rejected(self):
        for bad in [world([obj(size=(0, 1, 1))]),
                    world([obj(position=(float("nan"), 0, 0))]),
                    {"format": "other", "version": 1, "objects": []},
                    world("not an array")]:
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    module.convert(bad)

    def test_px4_world_metadata(self):
        xml, _ = module.convert(world([]))
        root = ET.fromstring(xml)
        w = root.find("./world")
        self.assertEqual(w.find("./physics/max_step_size").text, "0.001")
        self.assertEqual(w.find("./spherical_coordinates/surface_model").text, "EARTH_WGS84")
        self.assertEqual(w.find("./spherical_coordinates/latitude_deg").text, "47.397742")
        self.assertIsNotNone(w.find("./light[@name='sun']"))
        self.assertEqual(w.findall("./plugin"), [])

    def test_empty_world_has_ground(self):
        xml, count = module.convert(world([]))
        self.assertEqual(count, 0)
        self.assertIsNotNone(ET.fromstring(xml).find("./world/model[@name='ground']"))


if __name__ == "__main__":
    unittest.main()
