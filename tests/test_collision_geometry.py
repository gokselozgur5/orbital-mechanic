"""Analytic controls for actual primitive geometry, scene changes and floor policy."""
import copy
import math
from pathlib import Path
import random
import unittest
import xml.etree.ElementTree as ET

from orbital.collision_geometry import (Scene, Shape, Transform, IDENTITY, relation, sdf_pose,
                                         SUPPORT_PENETRATION_M)
from orbital.trajectory import truth_from_message
from collision_fixture import scene_message

ROOT = Path(__file__).resolve().parents[1]
WORLD = (ROOT/'worlds/maintenance.sdf').read_bytes()


def shape(kind, size, xyz=(0., 0., 0.), quaternion=(0., 0., 0., 1.)):
    return Shape('test', kind, size, Transform.pose((*xyz, *quaternion)))


class PrimitiveGeometryTests(unittest.TestCase):
    def test_quaternion_pose_composition_uses_parent_axes(self):
        turn = Transform.pose((1., 2., 3., 0., 0., math.sqrt(.5), math.sqrt(.5)))
        value = turn.compose(Transform.pose((1., 0., 0., 0., 0., 0., 1.)))
        for actual, expected in zip(value.center, (1., 3., 3.)):
            self.assertAlmostEqual(actual, expected)

    def test_box_face_touch_clearance_and_crossing_edges(self):
        a = shape('box', (2., 2., 2.))
        self.assertEqual(relation(a, shape('box', (2., 2., 2.), (2., 0., 0.))), 'overlap_or_touch')
        self.assertEqual(relation(a, shape('box', (2., 2., 2.), (2.000001, 0., 0.))), 'clear')
        # Neither rod has a corner inside the other; their interiors cross.
        rod = shape('box', (3., .1, .1))
        cross_rod = shape('box', (3., .1, .1), quaternion=(0., 0., math.sqrt(.5), math.sqrt(.5)))
        self.assertEqual(relation(rod, cross_rod), 'overlap_or_touch')
        separated = shape('box', (3., .1, .1), (0., 0., .10001), (0., 0., math.sqrt(.5), math.sqrt(.5)))
        self.assertEqual(relation(rod, separated), 'clear')

    def test_sphere_box_corner_uses_distance_not_a_bounding_cube(self):
        box = shape('box', (2., 2., 2.))
        self.assertEqual(relation(shape('sphere', (.1,), (1.09, 1.09, 0.)), box), 'clear')
        self.assertEqual(relation(shape('sphere', (.1,), (1.06, 1.06, 0.)), box), 'overlap_or_touch')
        self.assertEqual(relation(shape('sphere', (.1,), (0., 0., 0.)), box), 'overlap_or_touch')

    def test_cylinder_does_not_use_its_enclosing_sphere_as_collision(self):
        cylinder = shape('cylinder', (1., .2))
        self.assertEqual(relation(cylinder, shape('box', (.1, .1, .1), (0., 0., .3))), 'clear')
        self.assertEqual(relation(cylinder, shape('box', (.1, .1, .1), (.5, .5, 0.))), 'overlap_or_touch')

    def test_cylinder_approximation_band_is_explicitly_indeterminate(self):
        cylinder = shape('cylinder', (1., 1.))
        angle = math.pi/32
        box = shape('box', (.0002, .0002, .1), (.998*math.cos(angle), .998*math.sin(angle), 0.))
        self.assertEqual(relation(cylinder, box), 'indeterminate')
        box = shape('box', (.0002, .0002, .1), (1.002*math.cos(angle), 1.002*math.sin(angle), 0.))
        self.assertEqual(relation(cylinder, box), 'clear')

    def test_random_axis_aligned_cylinders_agree_with_independent_circle_rectangle_oracle(self):
        rng = random.Random(27)
        cylinder = shape('cylinder', (1., .4))
        counts = {'clear': 0, 'overlap_or_touch': 0, 'indeterminate': 0}
        for _ in range(300):
            x, y, z = rng.uniform(-1.4, 1.4), rng.uniform(-1.4, 1.4), rng.uniform(-.4, .4)
            sx, sy, sz = rng.uniform(.002, .2), rng.uniform(.002, .2), rng.uniform(.01, .2)
            exact = (math.hypot(max(abs(x)-sx/2, 0), max(abs(y)-sy/2, 0)) <= 1.
                     and abs(z) <= .2+sz/2)
            result = relation(cylinder, shape('box', (sx, sy, sz), (x, y, z)))
            counts[result] += 1
            if result == 'clear':
                self.assertFalse(exact)
            elif result == 'overlap_or_touch':
                self.assertTrue(exact)
        self.assertGreater(counts['clear'], 0)
        self.assertGreater(counts['overlap_or_touch'], 0)

    def test_common_rigid_transform_preserves_primitive_results(self):
        frame = Transform.pose(sdf_pose(ET.fromstring('<link><pose>2 -3 .7 .4 -.3 .8</pose></link>')))
        for part in (shape('box', (1., .3, .2)), shape('sphere', (.2,)), shape('cylinder', (.3, .5))):
            for center in ((0., 0., 0.), (.3, .3, 0.), (1., 1., 1.)):
                obstacle = shape('box', (.2, .2, .2), center)
                expected = relation(part, obstacle)
                moved_part = Shape(part.name, part.kind, part.size, frame.compose(part.transform))
                moved_box = Shape(obstacle.name, obstacle.kind, obstacle.size, frame.compose(obstacle.transform))
                self.assertEqual(relation(moved_part, moved_box), expected)


class SceneGeometryTests(unittest.TestCase):
    def setUp(self):
        self.scene = Scene(WORLD)

    def observe(self, **kwargs):
        return truth_from_message(scene_message(WORLD, 10**9, **kwargs), self.scene)

    def test_all_parts_are_clear_at_origin_with_normal_support_contacts(self):
        result = self.scene.evaluate(self.observe(robot_pose=IDENTITY))
        self.assertEqual(result['robot_parts'], 4)
        self.assertEqual(result['checked_pairs'], 28)
        self.assertEqual(result['events'], [])
        self.assertEqual(len(result['support_penetration_m']), 3)
        self.assertLess(max(result['support_penetration_m'].values()), 1e-7)

    def test_moved_obstacle_detects_wheel_only_contact(self):
        truth = self.observe(robot_pose=IDENTITY, model_poses={'cooler': (0., .66, .6, 0., 0., 0., 1.)})
        events = self.scene.evaluate(truth)['events']
        self.assertEqual(events, [{'part': 'left_wheel/collision', 'obstacle': 'cooler/body/collision',
                                   'result': 'overlap_or_touch'}])

    def test_actual_link_pose_changes_geometry_independently_of_model_center(self):
        truth = self.observe(robot_pose=IDENTITY, link_poses={'base_link': (3., 0., .22, 0., 0., 0., 1.)})
        self.assertTrue(any(e['part'] == 'base_link/collision' and e['obstacle'].startswith('cooler/')
                            for e in self.scene.evaluate(truth)['events']))

    def test_rotated_obstacle_uses_observed_orientation(self):
        tree = ET.fromstring(WORLD)
        cooler = tree.find("world/model[@name='cooler']")
        cooler.find('link/collision/geometry/box/size').text = '.1 2 1.2'
        source = ET.tostring(tree)
        scene = Scene(source)
        # A narrow vertical bar at x=.6 is clear, but after yaw rotation reaches the robot.
        pose = (.6, 0., .6, 0., 0., 0., 1.)
        clear = truth_from_message(scene_message(source, 10**9, robot_pose=IDENTITY,
                                                model_poses={'cooler': pose}), scene)
        self.assertEqual(scene.evaluate(clear)['events'], [])
        pose = (.6, 0., .6, 0., 0., math.sqrt(.5), math.sqrt(.5))
        hit = truth_from_message(scene_message(source, 10**9, robot_pose=IDENTITY,
                                              model_poses={'cooler': pose}), scene)
        self.assertTrue(scene.evaluate(hit)['events'])

    def test_tipped_body_floor_contact_is_not_exempted_as_wheel_support(self):
        pose = (0., 0., 0., math.sin(math.pi/8), 0., 0., math.cos(math.pi/8))
        result = self.scene.evaluate(self.observe(robot_pose=pose))
        self.assertIn({'part': 'base_link/collision', 'obstacle': 'floor/body/collision',
                       'result': 'body_at_or_below_floor'}, result['events'])

    def test_support_penetration_is_measured_and_limited(self):
        for depth, bad in ((.001, False), (SUPPORT_PENETRATION_M+.001, True)):
            value = self.scene.evaluate(self.observe(robot_pose=(0, 0, -depth, 0, 0, 0, 1)))
            self.assertAlmostEqual(value['support_penetration_m']['caster/collision'], depth)
            self.assertEqual(any(e['result'] == 'support_penetration' for e in value['events']), bad)

    def test_outside_finite_floor_is_an_adverse_result(self):
        value = self.scene.evaluate(self.observe(robot_pose=(6., 0., 0., 0., 0., 0., 1.)))
        self.assertTrue(any(e['result'] == 'outside_floor' for e in value['events']))

    def test_missing_extra_and_duplicate_scene_entities_refuse_capture(self):
        for mutation in ('missing', 'extra', 'duplicate_id'):
            msg = scene_message(WORLD, 10**9)
            if mutation == 'missing':
                msg.pose = [p for p in msg.pose if p.name != 'left_wheel']
            elif mutation == 'extra':
                extra = copy.deepcopy(msg.pose[0])
                extra.name, extra.id = 'unexpected_obstacle', 999
                msg.pose.append(extra)
            else:
                msg.pose[-1].id = msg.pose[0].id
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                truth_from_message(msg, self.scene)

    def test_static_link_changes_and_moving_floor_refuse_supported_contract(self):
        msg = scene_message(WORLD, 10**9)
        next(p for p in msg.pose if p.name == 'body').position.x = .1
        with self.assertRaisesRegex(ValueError, 'Static link moved'):
            truth_from_message(msg, self.scene)
        with self.assertRaisesRegex(ValueError, 'Floor moved'):
            self.observe(model_poses={'floor': (0., 0., -.2, 0., 0., 0., 1.)})

    def test_same_named_static_links_cannot_hide_swapped_relative_poses(self):
        tree = ET.fromstring(WORLD)
        link = tree.find("world/model[@name='cooler']/link")
        ET.SubElement(link, 'pose').text = '-3 0 0 0 0 0'
        # Pose_V has no parent IDs. Different transforms under the same name
        # could be exchanged between models without changing the multiset.
        with self.assertRaisesRegex(ValueError, 'Ambiguous static-link parent'):
            Scene(ET.tostring(tree))
        link.set('name', 'cooler_body')
        source = ET.tostring(tree)
        scene = Scene(source)
        truth = truth_from_message(scene_message(source, 10**9, robot_pose=IDENTITY), scene)
        self.assertTrue(any(e['obstacle'] == 'cooler/cooler_body/collision'
                            for e in scene.evaluate(truth)['events']))

    def test_duplicate_pose_static_and_geometry_elements_are_ambiguous(self):
        for path, field in (("world/model[@name='cooler']", 'pose'),
                            ("world/model[@name='cooler']", 'static'),
                            ("world/model[@name='cooler']/link/collision", 'geometry')):
            tree = ET.fromstring(WORLD)
            owner = tree.find(path)
            owner.append(copy.deepcopy(owner.find(field)))
            with self.subTest(field=field), self.assertRaises(ValueError):
                Scene(ET.tostring(tree))

    def test_unsupported_source_geometry_and_semantics_are_not_skipped(self):
        variants = [WORLD.replace(b'<sdf version="1.9">', b'<sdf version="1.8">'),
                    WORLD.replace(b'<world name="maintenance">', b'<world name="maintenance"><include/>'),
                    WORLD.replace(b'<pose>0 0 .02', b'<pose relative_to="other">0 0 .02'),
                    WORLD.replace(b'<radius>.06</radius>', b'<radius>-1</radius>'),
                    WORLD.replace(b'<box><size>.55 .4 .16</size></box>', b'<mesh><uri>file://unknown</uri></mesh>'),
                    WORLD.replace(b'<size>.55 .4 .16</size>', b'<size>.55 .4 .16</size><size>9 9 9</size>'),
                    b'<invalid', b'<!DOCTYPE x [<!ENTITY a "xx">]><sdf/>']
        for source in variants:
            with self.subTest(source=source[:90]), self.assertRaises(ValueError):
                Scene(source)

    def test_missing_part_world_hash_and_nonfinite_pose_refuse_evaluation(self):
        for mutation in ('part', 'hash', 'nan'):
            value = self.observe(robot_pose=IDENTITY)
            if mutation == 'part':
                del value['links']['caster']
            elif mutation == 'hash':
                value['world_sha256'] = '0'*64
            else:
                value['links']['left_wheel']['pose'][0] = float('nan')
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                self.scene.evaluate(value)


if __name__ == '__main__':
    unittest.main()
