#!/usr/bin/env python3
"""Schema-contract tests for local TSDF diagnostic aggregation."""

import unittest

from tsdf_bag_metrics import BOOLEAN_KEYS, NUMERIC_KEYS


class DiagnosticSchemaTest(unittest.TestCase):
    def test_direct_and_transaction_fields_are_typed(self):
        self.assertTrue(
            {
                "surface_frames_total",
                "surface_rejected_frames_total",
                "direct_applied_frames_total",
            }.issubset(NUMERIC_KEYS)
        )
        self.assertTrue(
            {
                "volume_updated",
                "volume_committed",
                "surface_prepared",
                "surface_output_accepted",
                "mesh_published_this_frame",
                "measured_capacity_limited",
                "measured_tsdf_enabled",
            }.issubset(BOOLEAN_KEYS)
        )

    def test_support_counts_and_flags_are_typed(self):
        self.assertTrue(
            {
                "support_input_points",
                "support_topology_points",
                "support_ring_pairs",
                "support_rejected_ring_pairs",
                "support_rejected_ring_order_pairs",
                "support_candidate_quads",
                "support_locally_valid_quads",
                "support_strong_quads",
                "support_accepted_quads",
                "support_verified_long_quads",
                "support_rejected_long_quads",
                "support_rejected_isolated_quads",
                "support_mesh_vertices",
                "support_mesh_triangles",
                "support_mesh_skipped_sensor_columns",
                "support_mesh_output_equivalence_input_triangles",
                "support_mesh_output_equivalence_removed_triangles",
                "support_mesh_output_equivalence_masked_sensor_columns",
                "support_validation_input_triangles",
                "support_validation_output_triangles",
                "support_validation_rejected_total",
            }.issubset(NUMERIC_KEYS)
        )
        self.assertTrue(
            {
                "support_enabled",
                "support_build_surface_rays",
                "support_build_indexed_mesh",
                "support_mesh_accepted",
                "support_mesh_applied",
                "support_output_topology_valid",
            }.issubset(BOOLEAN_KEYS)
        )

    def test_all_support_and_mesh_preparation_timings_are_numeric(self):
        self.assertTrue(
            {
                "support_build_ms",
                "support_integration_ms",
                "mesh_base_cleanup_ms",
                "mesh_base_equivalent_repair_ms",
                "mesh_support_validation_ms",
                "mesh_append_ms",
                "mesh_cross_equivalence_probe_ms",
                "mesh_combined_validation_ms",
                "total_ms",
            }.issubset(NUMERIC_KEYS)
        )

    def test_numeric_and_boolean_types_do_not_overlap(self):
        self.assertFalse(NUMERIC_KEYS & BOOLEAN_KEYS)


if __name__ == "__main__":
    unittest.main()
