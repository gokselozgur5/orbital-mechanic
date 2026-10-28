"""Independent continuous swept-disc check against original occupancy cells.

This evaluates a planned polyline, not the executed trajectory or live contacts.
It does not consume Nav2's inflated costmap or trust a planner success flag.
"""
import math

from orbital.mapping import FREE


def _segment_box_distance(a, b, lo, hi):
    delta = (b[0] - a[0], b[1] - a[1])
    entry, leave = 0., 1.
    for axis in (0, 1):
        if delta[axis] == 0:
            if not lo[axis] <= a[axis] <= hi[axis]:
                break
        else:
            t0 = (lo[axis] - a[axis]) / delta[axis]
            t1 = (hi[axis] - a[axis]) / delta[axis]
            entry, leave = max(entry, min(t0, t1)), min(leave, max(t0, t1))
    else:
        if entry <= leave:
            return 0.
    distances = [math.hypot(*(max(lo[i] - p[i], p[i] - hi[i], 0.) for i in (0, 1)))
                 for p in (a, b)]
    squared = sum(d * d for d in delta)
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            t = max(0., min(1., ((x-a[0])*delta[0] + (y-a[1])*delta[1])/squared)) if squared else 0.
            distances.append(math.hypot(x-a[0]-t*delta[0], y-a[1]-t*delta[1]))
    return min(distances)


def require_clear_path(grid, points, radius):
    """Reject any segment whose swept disc touches unknown, occupied or outside.

    Occupied cells are closed squares, not point obstacles. Checking the entire
    segment prevents sparse paths from jumping across an obstacle between poses.
    """
    points = list(points)
    if not points or not math.isfinite(radius) or radius <= 0:
        raise ValueError('Expected a nonempty path and positive finite radius')
    if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
        raise ValueError('Invalid path coordinates')
    for p in points:
        for axis, cells in ((0, grid.width), (1, grid.height)):
            if not grid.origin[axis] + radius < p[axis] < grid.origin[axis] + cells*grid.resolution - radius:
                raise ValueError('Footprint reaches outside map')
    for a, b in zip(points, points[1:] or points):
        lower = grid.cell(min(a[0], b[0])-radius, min(a[1], b[1])-radius)
        upper = grid.cell(max(a[0], b[0])+radius, max(a[1], b[1])+radius)
        for row in range(lower[1]-1, upper[1]+1):
            for column in range(lower[0]-1, upper[0]+1):
                if grid.at(column, row) == FREE:
                    continue
                lo = (grid.origin[0]+column*grid.resolution, grid.origin[1]+row*grid.resolution)
                hi = tuple(v+grid.resolution for v in lo)
                if _segment_box_distance(a, b, lo, hi) <= radius + 1e-10:
                    raise ValueError(f'Footprint intersects non-free cell ({column}, {row})')
