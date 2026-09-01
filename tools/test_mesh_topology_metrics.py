#!/usr/bin/env python3
"""Focused tests for mesh topology diagnostics."""

import math
import unittest

import numpy as np

from mesh_topology_metrics import (
    analyze_mesh,
    cpp_llround,
    quantize_coordinates,
)


def analyze(vertices, faces):
    return analyze_mesh(
        np.asarray(vertices, dtype=np.float64),
        np.asarray(faces, dtype=np.int64),
        1e-6,
        1e-12,
        "z",
        0,
        0.05,
    )


class VertexLinkMetricsTest(unittest.TestCase):
    def test_open_triangle_disk_is_manifold(self):
        result = analyze(
            [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)],
            [(0, 1, 2), (0, 2, 3)],
        )
        self.assertEqual(result["nonmanifold_edges"], 0)
        self.assertEqual(result["nonmanifold_vertices"], 0)
        self.assertEqual(result["boundary_vertices"], 4)
        self.assertFalse(result["watertight"])

    def test_closed_tetrahedron_is_manifold_and_watertight(self):
        result = analyze(
            [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)],
            [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)],
        )
        self.assertEqual(result["nonmanifold_edges"], 0)
        self.assertEqual(result["nonmanifold_vertices"], 0)
        self.assertEqual(result["boundary_vertices"], 0)
        self.assertTrue(result["watertight"])

    def test_bow_tie_vertex_is_detected_when_edges_are_manifold(self):
        result = analyze(
            [
                (0, 0, 0),
                (1, 0, 0),
                (0, 1, 0),
                (-1, 0, 0),
                (0, -1, 0),
            ],
            [(0, 1, 2), (0, 3, 4)],
        )
        self.assertEqual(result["nonmanifold_edges"], 0)
        self.assertEqual(result["nonmanifold_vertices"], 1)
        self.assertEqual(result["bow_tie_vertices"], 1)
        self.assertEqual(result["max_vertex_link_components"], 2)

    def test_three_faces_on_one_edge_are_nonmanifold(self):
        result = analyze(
            [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1)],
            [(0, 1, 2), (1, 0, 3), (0, 1, 4)],
        )
        self.assertEqual(result["nonmanifold_edges"], 1)
        self.assertGreaterEqual(result["nonmanifold_vertices"], 2)


class CppLlroundTest(unittest.TestCase):
    def test_exact_half_cells_round_away_from_zero(self):
        values = np.asarray((-1.5, -0.5, 0.5, 1.5), dtype=np.float64)
        np.testing.assert_array_equal(cpp_llround(values), (-2, -1, 1, 2))

    def test_neighbors_on_both_sides_of_half_cells(self):
        values = np.asarray(
            (
                np.nextafter(-1.5, -np.inf),
                -1.5,
                np.nextafter(-1.5, np.inf),
                np.nextafter(-0.5, -np.inf),
                -0.5,
                np.nextafter(-0.5, np.inf),
                np.nextafter(0.5, -np.inf),
                0.5,
                np.nextafter(0.5, np.inf),
                np.nextafter(1.5, -np.inf),
                1.5,
                np.nextafter(1.5, np.inf),
            ),
            dtype=np.float64,
        )
        np.testing.assert_array_equal(
            cpp_llround(values),
            (-2, -2, -1, -1, -1, 0, 0, 1, 1, 1, 2, 2),
        )

    def test_vertex_quantization_uses_point_one_millimeter_grid(self):
        tolerance_m = 0.0001
        half_cell = tolerance_m / 2.0
        one_and_half_cells = tolerance_m + half_cell
        coordinates = np.asarray(
            (
                (-one_and_half_cells, -half_cell, half_cell),
                (one_and_half_cells, 0.0, -0.0),
            ),
            dtype=np.float64,
        )
        np.testing.assert_array_equal(
            quantize_coordinates(coordinates, tolerance_m),
            ((-2, -1, 1), (2, 0, 0)),
        )

    def test_rejects_invalid_input_like_cpp_key_builder(self):
        with self.assertRaises(ValueError):
            cpp_llround(np.asarray((math.inf,), dtype=np.float64))
        with self.assertRaises(ValueError):
            quantize_coordinates(np.zeros((1, 3)), 0.0)

    def test_mesh_analysis_still_filters_nonfinite_vertices(self):
        result = analyze(
            [(0, 0, 0), (1, 0, 0), (0, 1, 0), (math.nan, 0, 0)],
            [(0, 1, 2), (0, 1, 3)],
        )
        self.assertEqual(result["finite_triangles"], 1)
        self.assertEqual(result["triangles"], 1)


if __name__ == "__main__":
    unittest.main()
