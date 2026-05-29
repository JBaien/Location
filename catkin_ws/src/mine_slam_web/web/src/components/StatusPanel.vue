<script setup lang="ts">
import type { ViewerStatus } from '../viewer/StatusClient';
import type { SceneDebug } from '../viewer/SceneView';

defineProps<{
  cloudConnected: boolean;
  statusConnected: boolean;
  status: ViewerStatus | null;
  fps: number;
  frameMs: number;
  sceneDebug: SceneDebug | null;
}>();

function fmtBbox(box: [number, number, number] | null | undefined): string {
  if (!box) return 'none';
  return box.map((v) => v.toFixed(1)).join(',');
}
</script>

<template>
  <section class="panel-section">
    <h2>Status</h2>
    <div class="status-row">
      <span>Cloud WS</span>
      <strong :class="cloudConnected ? 'ok' : 'bad'">{{ cloudConnected ? 'connected' : 'offline' }}</strong>
    </div>
    <div class="status-row">
      <span>Status WS</span>
      <strong :class="statusConnected ? 'ok' : 'bad'">{{ statusConnected ? 'connected' : 'offline' }}</strong>
    </div>
    <div class="status-row">
      <span>Current</span>
      <strong>{{ status?.current_cloud_points ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Stable</span>
      <strong>{{ status?.stable_map_points ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Path</span>
      <strong>{{ status?.displayed_path_point_count ?? status?.path.length ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Path source</span>
      <strong>{{ status?.path_source ?? 'odom' }}</strong>
    </div>
    <div class="status-row">
      <span>Path mode</span>
      <strong>{{ status?.path_is_snapshot_or_delta ?? 'odom_accumulated' }}</strong>
    </div>
    <div class="status-row">
      <span>Path span</span>
      <strong>{{ (status?.path_start_s ?? 0).toFixed(1) }}→{{ (status?.path_end_s ?? 0).toFixed(1) }}</strong>
    </div>
    <div class="status-row">
      <span>Path complete</span>
      <strong :class="status?.path_snapshot_complete === false ? 'bad' : 'ok'">{{ status?.path_snapshot_complete === false ? 'fallback' : 'ok' }}</strong>
    </div>
    <div class="status-row">
      <span>Full path</span>
      <strong>{{ status?.full_path_point_count ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Web path pts</span>
      <strong>{{ status?.status_path_point_count ?? status?.path.length ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Frame</span>
      <strong>{{ status?.map_frame ?? 'map' }}</strong>
    </div>
    <div class="status-row">
      <span>Odom</span>
      <strong>{{ status?.odom_source ?? 'none' }}</strong>
    </div>
    <div class="status-row">
      <span>Path resets</span>
      <strong>{{ status?.path_reset_count ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Reset reason</span>
      <strong>{{ status?.last_path_reset_reason ?? 'none' }}</strong>
    </div>
    <div class="status-row">
      <span>Cloud layers</span>
      <strong>{{ status?.current_layer_semantics ?? 'replace' }}/{{ status?.stable_layer_semantics ?? 'replace' }}</strong>
    </div>
    <div class="status-row">
      <span>Current geom</span>
      <strong>{{ sceneDebug?.current.geometry_position_count ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Stable geom</span>
      <strong>{{ sceneDebug?.stable.geometry_position_count ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Path geom</span>
      <strong>{{ sceneDebug?.path_position_count ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Current bbox</span>
      <strong>{{ fmtBbox(sceneDebug?.current.bbox_min) }} / {{ fmtBbox(sceneDebug?.current.bbox_max) }}</strong>
    </div>
    <div class="status-row">
      <span>Stable bbox</span>
      <strong>{{ fmtBbox(sceneDebug?.stable.bbox_min) }} / {{ fmtBbox(sceneDebug?.stable.bbox_max) }}</strong>
    </div>
    <div class="status-row">
      <span>Frame audit</span>
      <strong>{{ status?.fixed_frame ?? 'map' }} {{ status?.transform_applied_in_frontend ? 'front-xform' : 'front-fixed' }}</strong>
    </div>
    <div class="status-row">
      <span>Progressive</span>
      <strong :class="status?.progressive_reveal_enabled ? 'ok' : 'bad'">{{ status?.progressive_reveal_enabled ? 'on' : 'off' }}</strong>
    </div>
    <div class="status-row">
      <span>Reveal s</span>
      <strong>{{ (status?.progressive_machine_s ?? 0).toFixed(1) }}/{{ (status?.progressive_revealed_s ?? 0).toFixed(1) }}</strong>
    </div>
    <div class="status-row">
      <span>Face wall</span>
      <strong>{{ (status?.progressive_face_wall_s ?? 0).toFixed(1) }}</strong>
    </div>
    <div class="status-row">
      <span>Reflectors v/h</span>
      <strong>{{ status?.progressive_visible_reflector_count ?? 0 }}/{{ status?.progressive_hidden_reflector_count ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>Hidden pts</span>
      <strong>{{ status?.progressive_hidden_unrevealed_point_count ?? 0 }}</strong>
    </div>
    <div class="status-row">
      <span>FPS</span>
      <strong>{{ fps.toFixed(1) }}</strong>
    </div>
    <div class="status-row">
      <span>Frame time</span>
      <strong>{{ frameMs.toFixed(1) }} ms</strong>
    </div>
    <div class="pose" v-if="status?.pose.valid">
      <span>x {{ status.pose.x.toFixed(2) }}</span>
      <span>y {{ status.pose.y.toFixed(2) }}</span>
      <span>z {{ status.pose.z.toFixed(2) }}</span>
    </div>
  </section>
</template>
