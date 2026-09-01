#!/usr/bin/env python3
"""Pure-Python synthetic regressions for local TSDF component safety."""

import unittest

import numpy as np

from local_tsdf_acceptance import (
    analyze_component_safety,
    frame_ids_match,
    prepare_topology,
)


def add_grid_plane(vertices, faces, fixed_axis, fixed_value, u_axis, u_values, v_axis, v_values):
    base = len(vertices)
    for u_value in u_values:
        for v_value in v_values:
            point = [0.0, 0.0, 0.0]
            point[fixed_axis] = fixed_value
            point[u_axis] = float(u_value)
            point[v_axis] = float(v_value)
            vertices.append(point)
    width = len(v_values)
    for u_index in range(len(u_values) - 1):
        for v_index in range(len(v_values) - 1):
            lower = base + u_index * width + v_index
            upper = lower + width
            faces.extend(
                (
                    (lower, upper, upper + 1),
                    (lower, upper + 1, lower + 1),
                )
            )


def plane_points(fixed_axis, fixed_value, u_axis, u_values, v_axis, v_values):
    points = []
    for u_value in u_values:
        for v_value in v_values:
            point = [0.0, 0.0, 0.0]
            point[fixed_axis] = fixed_value
            point[u_axis] = float(u_value)
            point[v_axis] = float(v_value)
            points.append(point)
    return points


def tunnel_fixture():
    vertices = []
    faces = []
    observed = []
    mesh_x = np.linspace(-4.0, 4.0, 21)
    cloud_x = np.linspace(-4.0, 4.0, 81)

    # Deliberate edge gaps keep all four valid tunnel surfaces disconnected.
    for y_value in (-2.0, 2.0):
        mesh_z = np.linspace(-1.2, 1.2, 7)
        cloud_z = np.linspace(-1.2, 1.2, 25)
        add_grid_plane(vertices, faces, 1, y_value, 0, mesh_x, 2, mesh_z)
        observed.extend(plane_points(1, y_value, 0, cloud_x, 2, cloud_z))
    for z_value in (-1.5, 1.5):
        mesh_y = np.linspace(-1.8, 1.8, 10)
        cloud_y = np.linspace(-1.8, 1.8, 37)
        add_grid_plane(vertices, faces, 2, z_value, 0, mesh_x, 1, mesh_y)
        observed.extend(plane_points(2, z_value, 0, cloud_x, 1, cloud_y))
    return vertices, faces, observed


def analyze_fixture(vertices, faces, observed, **kwargs):
    raw_vertices = np.asarray(vertices, dtype=np.float64)
    raw_faces = np.asarray(faces, dtype=np.int64)
    welded_vertices, welded_faces, areas, _, components = prepare_topology(
        raw_vertices, raw_faces, 1e-6, 1e-12
    )
    return analyze_component_safety(
        welded_vertices,
        welded_faces,
        areas,
        components,
        np.asarray(observed, dtype=np.float64),
        support_distance_m=0.15,
        min_component_area_m2=1.0,
        min_reverse_precision=0.90,
        **kwargs,
    )


class ComponentSafetyTest(unittest.TestCase):
    def test_disconnected_side_roof_and_floor_surfaces_pass(self):
        vertices, faces, observed = tunnel_fixture()
        result = analyze_fixture(vertices, faces, observed)

        self.assertTrue(result["passed"])
        self.assertEqual(result["component_count"], 4)
        self.assertEqual(result["large_component_count"], 4)
        self.assertEqual(result["unsupported_large_components"], [])
        self.assertEqual(result["cross_tunnel_curtain_components"], [])
        for component in result["components"]:
            self.assertGreaterEqual(component["reverse_precision"], 0.90)
            self.assertIn("longitudinal", component["bounds_m"])
            self.assertIn("main_normal", component["pca"])

    def test_floating_large_plane_fails_reverse_precision(self):
        vertices, faces, observed = tunnel_fixture()
        add_grid_plane(
            vertices,
            faces,
            1,
            0.0,
            0,
            np.linspace(-2.0, 2.0, 11),
            2,
            np.linspace(-0.5, 0.5, 4),
        )
        result = analyze_fixture(vertices, faces, observed)

        self.assertFalse(result["passed"])
        self.assertEqual(len(result["unsupported_large_components"]), 1)
        floating = result["components"][result["unsupported_large_components"][0]]
        self.assertGreaterEqual(floating["area_m2"], 1.0)
        self.assertLess(floating["reverse_precision"], 0.90)
        self.assertFalse(floating["cross_tunnel_curtain"])

    def test_attached_unsupported_flap_fails_local_patch_gate(self):
        vertices, faces, observed = tunnel_fixture()
        add_grid_plane(
            vertices,
            faces,
            2,
            1.2,
            0,
            np.linspace(-0.8, 0.8, 5),
            1,
            np.linspace(2.0, 2.5, 3),
        )
        result = analyze_fixture(vertices, faces, observed)

        self.assertFalse(result["passed"])
        self.assertEqual(result["unsupported_large_components"], [])
        self.assertEqual(len(result["unsupported_patch_components"]), 1)
        component = result["components"][result["unsupported_patch_components"][0]]
        unsafe = [patch for patch in component["unsupported_patches"] if patch["unsafe"]]
        self.assertTrue(unsafe)
        self.assertGreater(max(patch["unsupported_area_m2"] for patch in unsafe), 0.25)

    def test_multiple_small_unsupported_components_fail_total_area_gate(self):
        vertices, faces, observed = tunnel_fixture()
        for y_value in (0.0, 0.6):
            add_grid_plane(
                vertices,
                faces,
                1,
                y_value,
                0,
                np.linspace(-0.4, 0.4, 3),
                2,
                np.linspace(-0.4, 0.4, 3),
            )
        result = analyze_fixture(vertices, faces, observed)

        self.assertFalse(result["passed"])
        self.assertTrue(result["total_unsupported_area_exceeded"])
        self.assertGreater(result["total_unsupported_area_m2"], 1.0)

    def test_cross_tunnel_curtain_is_explicitly_rejected(self):
        vertices, faces, observed = tunnel_fixture()
        curtain_y = np.linspace(-1.8, 1.8, 10)
        curtain_z = np.linspace(-1.2, 1.2, 7)
        add_grid_plane(
            vertices,
            faces,
            0,
            0.0,
            1,
            curtain_y,
            2,
            curtain_z,
        )
        # Even a curtain with point support must fail the independent geometric
        # cross-wall gate; reverse precision alone is insufficient here.
        observed.extend(
            plane_points(
                0,
                0.0,
                1,
                np.linspace(-1.8, 1.8, 37),
                2,
                np.linspace(-1.2, 1.2, 25),
            )
        )
        result = analyze_fixture(vertices, faces, observed)

        self.assertFalse(result["passed"])
        self.assertEqual(len(result["cross_tunnel_curtain_components"]), 1)
        curtain = result["components"][result["cross_tunnel_curtain_components"][0]]
        self.assertGreaterEqual(curtain["reverse_precision"], 0.90)
        self.assertTrue(curtain["pca"]["planar_thin"])
        self.assertGreaterEqual(curtain["pca"]["normal_tunnel_axis_abs_dot"], 0.85)

    def test_wall_attached_curtain_is_rejected_as_local_planar_patch(self):
        vertices, faces, observed = tunnel_fixture()
        curtain_y = np.linspace(-1.6, 2.0, 10)
        curtain_z = np.linspace(-1.2, 1.2, 7)
        add_grid_plane(
            vertices,
            faces,
            0,
            0.0,
            1,
            curtain_y,
            2,
            curtain_z,
        )
        observed.extend(
            plane_points(
                0,
                0.0,
                1,
                np.linspace(-1.6, 2.0, 37),
                2,
                np.linspace(-1.2, 1.2, 25),
            )
        )
        result = analyze_fixture(vertices, faces, observed)

        self.assertFalse(result["passed"])
        self.assertEqual(len(result["cross_tunnel_curtain_components"]), 1)
        self.assertTrue(result["cross_tunnel_curtain_patches"])
        self.assertTrue(
            all(
                patch["cross_tunnel_curtain"]
                for patch in result["cross_tunnel_curtain_patches"]
            )
        )

    def test_short_ambiguous_cloud_reports_axis_unknown_without_curtain_veto(self):
        vertices, faces, observed = tunnel_fixture()
        vertices = np.asarray(vertices, dtype=np.float64)
        observed = np.asarray(observed, dtype=np.float64)
        vertices[:, 0] *= 0.25
        observed[:, 0] *= 0.25

        result = analyze_fixture(vertices, faces, observed)

        self.assertTrue(result["passed"])
        self.assertIsNone(result["tunnel_axis"])
        self.assertEqual(result["tunnel_axis_status"], "axis_unknown")
        self.assertFalse(result["tunnel_axis_confidence"]["confident"])
        self.assertEqual(result["cross_tunnel_curtain_components"], [])

    def test_explicit_axis_overrides_ambiguous_cloud_pca(self):
        vertices, faces, observed = tunnel_fixture()
        vertices = np.asarray(vertices, dtype=np.float64)
        observed = np.asarray(observed, dtype=np.float64)
        vertices[:, 0] *= 0.25
        observed[:, 0] *= 0.25

        result = analyze_fixture(
            vertices,
            faces,
            observed,
            tunnel_axis=np.asarray((4.0, 0.0, 0.0), dtype=np.float64),
        )

        self.assertTrue(result["passed"])
        self.assertEqual(result["tunnel_axis_status"], "explicit")
        np.testing.assert_allclose(result["tunnel_axis"], [1.0, 0.0, 0.0])

    def test_axis_unknown_does_not_relax_reverse_precision(self):
        vertices, faces, observed = tunnel_fixture()
        add_grid_plane(
            vertices,
            faces,
            1,
            0.0,
            0,
            np.linspace(-2.4, 2.4, 13),
            2,
            np.linspace(-0.5, 0.5, 4),
        )
        vertices = np.asarray(vertices, dtype=np.float64)
        observed = np.asarray(observed, dtype=np.float64)
        vertices[:, 0] *= 0.25
        observed[:, 0] *= 0.25

        result = analyze_fixture(vertices, faces, observed)

        self.assertEqual(result["tunnel_axis_status"], "axis_unknown")
        self.assertFalse(result["passed"])
        self.assertEqual(len(result["unsupported_large_components"]), 1)
        self.assertEqual(result["cross_tunnel_curtain_components"], [])

    def test_small_unsupported_noise_keeps_existing_one_square_meter_gate(self):
        vertices, faces, observed = tunnel_fixture()
        add_grid_plane(
            vertices,
            faces,
            1,
            0.0,
            0,
            np.linspace(-0.4, 0.4, 3),
            2,
            np.linspace(-0.4, 0.4, 3),
        )
        result = analyze_fixture(vertices, faces, observed)

        self.assertTrue(result["passed"])
        self.assertEqual(result["small_component_count"], 1)
        noise = next(component for component in result["components"] if not component["is_large"])
        self.assertAlmostEqual(noise["area_m2"], 0.64, places=6)
        self.assertLess(noise["reverse_precision"], 0.90)

    def test_frame_ids_require_an_exact_nonempty_match(self):
        self.assertTrue(frame_ids_match("base_link", "base_link"))
        self.assertFalse(frame_ids_match("base_link", "map"))
        self.assertFalse(frame_ids_match("", ""))


if __name__ == "__main__":
    unittest.main()
