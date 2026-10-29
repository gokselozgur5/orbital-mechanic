import math
import unittest

from orbital.mapping import FREE, OCCUPIED, UNKNOWN, Grid
from orbital.path_clearance import require_clear_path


class SweptPathTests(unittest.TestCase):
    def setUp(self):
        self.grid = Grid(.1, (0., 0.), 100, 100, [FREE] * 10000)
        self.grid.cells[50*100+50] = OCCUPIED

    def test_clear_detour(self):
        require_clear_path(self.grid, [(1., 5.05), (1., 6.), (9., 6.), (9., 5.05)], .37)

    def test_sparse_segment_crosses_obstacle(self):
        with self.assertRaisesRegex(ValueError, 'non-free'):
            require_clear_path(self.grid, [(1., 5.05), (9., 5.05)], .37)

    def test_footprint_hits_without_center_collision(self):
        with self.assertRaisesRegex(ValueError, 'non-free'):
            require_clear_path(self.grid, [(1., 5.4), (9., 5.4)], .37)

    def test_unknown_is_blocking(self):
        self.grid.cells[50*100+50] = UNKNOWN
        with self.assertRaisesRegex(ValueError, 'non-free'):
            require_clear_path(self.grid, [(5.05, 5.05)], .37)

    def test_outside_or_invalid(self):
        for points in ([], [(math.nan, 2.)], [(0.2, 2.)], [(20., 2.)]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                require_clear_path(self.grid, points, .37)

    def test_single_point_and_diagonal_corner(self):
        require_clear_path(self.grid, [(4.7, 4.7)], .37)
        with self.assertRaisesRegex(ValueError, 'non-free'):
            require_clear_path(self.grid, [(4.8, 4.8)], .37)
        with self.assertRaisesRegex(ValueError, 'non-free'):
            require_clear_path(self.grid, [(4., 6.), (6., 4.)], .37)
