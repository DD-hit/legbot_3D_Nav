import unittest
from unittest.mock import patch

import numpy as np

from studio_geometry import checked_edit, in_polygon, update_region
from studio_support import support_patch


class StairSupportTest(unittest.TestCase):
    def test_flat_fill_keeps_measured_ground(self):
        shape = (31, 31)
        floors = [[1.0, "Platform"]]
        heights = np.full(shape, 1.025)
        heights[15, 15] = np.nan
        base = np.zeros((5, 1, *shape), dtype="f4")
        base[0] = 1.0
        base[3, 0] = heights
        base[4] = np.nan
        data = dict(data=base.copy(), resolution=0.1, center=np.zeros(2))
        edit = checked_edit(
            dict(
                floor=0,
                action="fill",
                z=[0.8, 1.3],
                height=1.0,
                polygon=[[-0.4, -0.4], [0.4, -0.4], [0.4, 0.4], [-0.4, 0.4]],
            ),
            floors,
            data,
        )
        with patch("studio_geometry.tile_points", return_value=np.empty((0, 3))):
            update_region(
                None,
                data,
                base,
                (np.zeros(2, dtype=int), np.array(shape)),
                0,
                [edit],
                {"floors": floors},
                heights,
                0.25,
                np.ones(shape, dtype="u1"),
            )
        known = np.isfinite(heights)
        np.testing.assert_array_equal(data["data"][3, 0][known], base[3, 0][known])
        self.assertEqual(data["data"][3, 0, 15, 15], 1.0)

    def scene(self, resolution, angle, center):
        shape = (45, 45)
        center = np.asarray(center)
        xy = center + (np.indices(shape).transpose(1, 2, 0) - 22) * resolution
        side = np.array([np.cos(angle), np.sin(angle)])
        direction = np.array([-side[1], side[0]])
        along = (xy - center) @ direction
        floors = [(10.0, "A"), (11.6, "B"), (13.2, "C")]
        ground = 10.8 + np.round(along / 0.55) * 0.16
        values = np.zeros((5, 2, *shape))
        values[3, 0] = ground
        values[3, 1] = ground + 1.6
        values[4] = values[3] + 1.5
        values[3, 1, 22, 22] = ground[22, 22]
        flight = dict(
            angle=angle,
            x=center @ side,
            start=center @ direction - 2.75,
            end=center @ direction + 2.75,
            width=1.2,
            count=10,
            floor=1,
            to=2,
        )
        data = dict(data=values, center=center, resolution=resolution, slice_dh=0.5)
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * 0.35
        edit = dict(floor=1, height=11.6, z=[10.0, 14.0], polygon=(center + corners).tolist())
        return data, edit, flight, floors

    def test_overlapping_flights(self):
        for res, angle, center in [
            (0.05, 0.73, [110.0, -223.0]),
            (0.1, -1.2, [-19.0, 14.0]),
            (0.15, 0.0, [0.0, 0.0]),
        ]:
            with self.subTest(res=res, angle=angle):
                data, edit, flight, floors = self.scene(res, angle, center)

                def repair(selected, points=()):
                    return support_patch(
                        data,
                        edit,
                        selected,
                        floors,
                        in_polygon,
                        lambda *bounds: np.asarray(points).reshape(-1, 3),
                    )

                self.assertEqual(repair(dict(flight, floor=0, to=1)), [])
                self.assertEqual(repair(flight), [[1, 22, 22, 12.4]])
                self.assertEqual(repair(flight, [[*center, 12.65]]), [])
                edit["z"] = [10.0, 11.7]
                self.assertEqual(repair(flight), [])
                edit["z"] = [10.0, 14.0]
                data["data"][3, 1, 16:29, 22] = np.nan
                self.assertEqual(repair(flight), [])


if __name__ == "__main__":
    unittest.main()
