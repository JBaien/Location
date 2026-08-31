<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import ColorModeSelector from './components/ColorModeSelector.vue';
import LayerPanel from './components/LayerPanel.vue';
import type { ColorMode } from './viewer/ColorMap';
import { BinaryCloudClient } from './viewer/BinaryCloudClient';
import { LocalTsdfMeshClient } from './viewer/LocalTsdfMeshClient';
import type { LocalTsdfMeshDebug } from './viewer/LocalTsdfMeshLayer';
import type { MeshFrameMetrics, ScanMeshDebug } from './viewer/ScanMeshLayer';
import { SceneView, type LayerState } from './viewer/SceneView';
import { StatusClient, type EquipmentMetricState, type ViewerStatus } from './viewer/StatusClient';

const host = window.location.hostname || 'localhost';
const cloudUrl = `ws://${host}:9001/cloud`;
const statusUrl = `ws://${host}:9002/status`;
const meshUrl = `ws://${host}:9003/mesh`;

const sceneEl = ref<HTMLElement | null>(null);
const colorMode = ref<ColorMode>('height');
// Show the raw evidence together with one surface representation. Enabling the
// scan-strip mesh or stable map remains an explicit diagnostic choice and
// avoids silently overlaying layers that may use different coordinate frames.
const layers = ref<LayerState>({ current: true, mesh: false, tsdfMesh: true, stable: false, path: true, reflector: true, grid: true });
const cloudConnected = ref(false);
const statusConnected = ref(false);
const meshConnected = ref(false);
const status = ref<ViewerStatus | null>(null);
const menuOpen = ref(false);
const pointSize = ref(0.035);
const meshOpacity = ref(0.48);
const meshTriangles = ref(0);
const meshDebug = ref<ScanMeshDebug | null>(null);
const latestMeshFrame = ref<MeshFrameMetrics | null>(null);
const tsdfMeshDebug = ref<LocalTsdfMeshDebug | null>(null);
const tsdfMeshPacketErrors = ref(0);
const currentTime = ref('');

interface BridgeCloudSnapshot {
  stamp_ns: string;
  captured_at_ms: number;
  raw_points: number;
  encoded_points: number;
  encoded_retention: number;
  publisher_count: number | null;
  voxel_size_m: number | null;
  max_points: number | null;
  at_point_limit: boolean;
}

const bridgeSnapshotLimit = 180;
const bridgeSnapshots: BridgeCloudSnapshot[] = [];

let scene: SceneView | null = null;
let cloudClient: BinaryCloudClient | null = null;
let statusClient: StatusClient | null = null;
let meshClient: LocalTsdfMeshClient | null = null;
let clockTimer = 0;

function isTypingTarget(target: EventTarget | null): boolean {
  const element = target as HTMLElement | null;
  if (!element) return false;
  return (
    element.isContentEditable ||
    element.tagName === 'INPUT' ||
    element.tagName === 'SELECT' ||
    element.tagName === 'TEXTAREA'
  );
}

function toggleFullscreen(): void {
  menuOpen.value = false;
  if (document.fullscreenElement) {
    void document.exitFullscreen();
    return;
  }
  void document.documentElement.requestFullscreen?.();
}

function formatAngle(value: number | undefined): string {
  if (value === undefined || !Number.isFinite(value)) return '---';
  return value.toFixed(3);
}

function formatDistance(value: number | undefined): string {
  if (value === undefined || !Number.isFinite(value)) return '---';
  return value.toFixed(0);
}

function formatMm(value: number | undefined): string {
  if (value === undefined || !Number.isFinite(value)) return '---';
  return value.toFixed(0);
}

function formatPercent(value: number | undefined): string {
  if (value === undefined || !Number.isFinite(value)) return '---';
  return `${(value * 100).toFixed(1)}%`;
}

function formatMs(value: number | undefined): string {
  if (value === undefined || !Number.isFinite(value)) return '---';
  return `${value.toFixed(1)} ms`;
}

function formatRuntimeConfig(
  voxelSize: number | undefined,
  maxPoints: number | undefined
): string {
  if (voxelSize === undefined || maxPoints === undefined) return '---';
  return `${voxelSize.toFixed(3)} m / ${maxPoints}`;
}

function formatPublisherState(
  publisherCount: number | undefined,
  atPointLimit: boolean | undefined
): string {
  if (publisherCount === undefined || atPointLimit === undefined) return '---';
  return `${publisherCount} / ${atPointLimit ? '已触发上限' : '未触发上限'}`;
}

function badgeClass(valid: boolean | undefined): string {
  if (valid === undefined) return 'unknown';
  return valid ? 'online' : 'offline';
}

function statusBadgeClass(value: string | undefined): string {
  if (!value) return 'unknown';
  if (value === 'OK' || value === 'good') return 'online';
  if (value === 'DEGRADED' || value === 'degraded') return 'unknown';
  return 'offline';
}

function qualityText(value: string | undefined): string {
  const map: Record<string, string> = {
    OK: '正常',
    DEGRADED: '降级',
    INVALID: '无效',
    LOST: '离线',
    good: '正常',
    degraded: '降级',
    lost: '离线',
    none: '无数据'
  };
  return map[value ?? 'lost'] ?? value ?? '离线';
}

function colorModeText(value: ColorMode): string {
  const map: Record<ColorMode, string> = {
    height: '高度',
    intensity: '强度',
    'x-distance': 'X距离',
    'y-distance': 'Y距离',
    'z-distance': 'Z距离',
    lidar_id: '雷达编号',
    'stable-temporary': '地图/实时',
    reflector: '反光点'
  };
  return map[value] ?? value;
}

function emptyMetric(): EquipmentMetricState {
  return {
    seen: false,
    source: 'none',
    quality: 'lost',
    invalid_reason: 'none',
    overall_status: 'lost',
    stamp_ns: 0,
    attitude: {
      valid: false,
      roll_valid: false,
      pitch_valid: false,
      yaw_valid: false,
      roll_quality: 'INVALID',
      pitch_quality: 'INVALID',
      yaw_quality: 'INVALID',
      roll_invalid_reason: 'none',
      pitch_invalid_reason: 'none',
      yaw_invalid_reason: 'none',
      roll_deg: 0,
      pitch_deg: 0,
      yaw_deg: 0,
      ground_plane_rmse: 0,
      ground_points: 0,
      wall_points: 0,
      point_count: 0
    },
    distances: {
      valid: false,
      left_front_valid: false,
      left_rear_valid: false,
      right_front_valid: false,
      right_rear_valid: false,
      left_front_quality: 'INVALID',
      left_rear_quality: 'INVALID',
      right_front_quality: 'INVALID',
      right_rear_quality: 'INVALID',
      left_front_invalid_reason: 'none',
      left_rear_invalid_reason: 'none',
      right_front_invalid_reason: 'none',
      right_rear_invalid_reason: 'none',
      left_front_clearance_m: 0,
      left_rear_clearance_m: 0,
      right_front_clearance_m: 0,
      right_rear_clearance_m: 0,
      left_front_mm: 0,
      left_rear_mm: 0,
      right_front_mm: 0,
      right_rear_mm: 0,
      left_front_points: 0,
      left_rear_points: 0,
      right_front_points: 0,
      right_rear_points: 0
    }
  };
}

const cloudMetric = computed(() => status.value?.cloud_estimate ?? emptyMetric());
const realMetric = computed(() => status.value?.real_sensors ?? emptyMetric());
const cloudOverallStatus = computed(() => cloudMetric.value.overall_status ?? cloudMetric.value.quality);
const encodedRetention = computed(() => {
  const raw = status.value?.current_cloud_raw_points ?? 0;
  if (raw <= 0) return undefined;
  return (status.value?.current_cloud_points ?? 0) / raw;
});
const surfaceFrameMismatch = computed(() => {
  const frames: string[] = [];
  const cloudFrame = status.value?.current_cloud_frame_id;
  if (
    (layers.value.current || layers.value.mesh) &&
    (status.value?.current_cloud_points ?? 0) > 0 &&
    cloudFrame
  ) {
    frames.push(cloudFrame);
  }
  const appliedTsdfFrame = tsdfMeshDebug.value?.frame_id;
  if (
    layers.value.tsdfMesh &&
    (tsdfMeshDebug.value?.triangle_count ?? 0) > 0 &&
    appliedTsdfFrame
  ) {
    frames.push(appliedTsdfFrame);
  }
  const stableFrame = status.value?.stable_map_frame_id;
  if (
    layers.value.stable &&
    (status.value?.stable_map_points ?? 0) > 0 &&
    stableFrame
  ) {
    frames.push(stableFrame);
  }
  const pathFrame = status.value?.path_frame;
  if (
    layers.value.path &&
    (status.value?.status_path_point_count ?? 0) > 0 &&
    pathFrame
  ) {
    frames.push(pathFrame);
  }
  return new Set(frames).size > 1;
});

function updateClock(): void {
  currentTime.value = new Date().toLocaleString('zh-CN', {
    year: 'numeric',
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    hour12: false
  });
}

function handleKeydown(event: KeyboardEvent): void {
  if (event.key === 'Escape') {
    menuOpen.value = false;
    return;
  }
  if (event.code !== 'Space' || isTypingTarget(event.target)) return;
  event.preventDefault();
  toggleFullscreen();
}

onMounted(() => {
  if (!sceneEl.value) return;
  updateClock();
  clockTimer = window.setInterval(updateClock, 1000);
  scene = new SceneView(sceneEl.value);
  scene.setLayers(layers.value);
  scene.setColorMode(colorMode.value);
  scene.setPointSize(pointSize.value);
  scene.setMeshOpacity(meshOpacity.value);
  window.__MINE_SLAM_VIEWER_DEBUG__ = {
    meshDebug: () => scene?.currentMeshLayer.getDebug() ?? null,
    meshLatest: () => scene?.currentMeshLayer.getLatestFrameMetrics() ?? null,
    meshFrames: () => scene?.currentMeshLayer.getFrameHistory() ?? [],
    tsdfMesh: () => scene?.localTsdfMeshLayer.getDebug() ?? null,
    bridgeFrames: () => bridgeSnapshots.map((snapshot) => ({ ...snapshot })),
    clearMeshFrames: () => scene?.currentMeshLayer.clearFrameHistory(),
    clearBridgeFrames: () => bridgeSnapshots.splice(0, bridgeSnapshots.length)
  };
  window.addEventListener('keydown', handleKeydown);
  scene.onStats((stats) => {
    meshTriangles.value = scene?.currentMeshLayer.getTriangleCount() ?? 0;
    meshDebug.value = scene?.currentMeshLayer.getDebug() ?? null;
    latestMeshFrame.value = scene?.currentMeshLayer.getLatestFrameMetrics() ?? null;
    tsdfMeshDebug.value = scene?.localTsdfMeshLayer.getDebug() ?? null;
    window.__MINE_SLAM_VIEWER_STATS__ = {
      fps: stats.fps,
      frameMs: stats.frameMs,
      frameMeanMs: stats.frameMeanMs,
      frameP95Ms: stats.frameP95Ms,
      currentPoints: status.value?.current_cloud_points ?? 0,
      currentRawPoints: status.value?.current_cloud_raw_points ?? 0,
      currentCloudStampNs: status.value?.current_cloud_stamp_ns ?? '0',
      currentCloudPublisherCount: status.value?.current_cloud_publisher_count ?? 0,
      currentCloudAtPointLimit: status.value?.current_cloud_at_point_limit ?? false,
      stablePoints: status.value?.stable_map_points ?? 0,
      pathPoints: status.value?.path.length ?? 0,
      meshTriangles: meshTriangles.value,
      meshPointCoverage: meshDebug.value?.point_coverage ?? 0,
      meshQuadCoverage: meshDebug.value?.quad_coverage ?? 0,
      meshBuildMs: meshDebug.value?.mesh_build_ms ?? 0,
      meshApplyMs: meshDebug.value?.mesh_apply_ms ?? 0,
      meshTopRejectReason: meshDebug.value?.top_reject_reason ?? 'none',
      tsdfMeshConnected: meshConnected.value,
      tsdfMeshTriangles: tsdfMeshDebug.value?.triangle_count ?? 0,
      tsdfMeshRevision: tsdfMeshDebug.value?.revision ?? '0',
      tsdfMeshPacketErrors: tsdfMeshPacketErrors.value,
      cloudConnected: cloudConnected.value,
      statusConnected: statusConnected.value
    };
  });

  cloudClient = new BinaryCloudClient(
    cloudUrl,
    (cloud, delivery) => scene?.updateCloud(cloud, delivery),
    (connected) => {
      cloudConnected.value = connected;
    }
  );
  cloudClient.start();

  meshClient = new LocalTsdfMeshClient(
    meshUrl,
    (mesh) => {
      scene?.updateLocalTsdfMesh(mesh);
      tsdfMeshDebug.value = scene?.localTsdfMeshLayer.getDebug() ?? null;
    },
    (connected) => {
      meshConnected.value = connected;
      if (!connected) {
        scene?.clearLocalTsdfMesh();
        tsdfMeshDebug.value =
          scene?.localTsdfMeshLayer.getDebug() ?? null;
      }
    },
    () => {
      tsdfMeshPacketErrors.value += 1;
    }
  );
  meshClient.start();

  statusClient = new StatusClient(
    statusUrl,
    (nextStatus) => {
      status.value = nextStatus;
      const stampNs = nextStatus.current_cloud_stamp_ns;
      if (
        stampNs &&
        stampNs !== '0' &&
        bridgeSnapshots[bridgeSnapshots.length - 1]?.stamp_ns !== stampNs
      ) {
        const rawPoints = nextStatus.current_cloud_raw_points;
        const encodedPoints = nextStatus.current_cloud_points;
        bridgeSnapshots.push({
          stamp_ns: stampNs,
          captured_at_ms: performance.now(),
          raw_points: rawPoints,
          encoded_points: encodedPoints,
          encoded_retention: rawPoints > 0 ? encodedPoints / rawPoints : 0,
          publisher_count: nextStatus.current_cloud_publisher_count ?? null,
          voxel_size_m: nextStatus.current_cloud_voxel_size_m ?? null,
          max_points: nextStatus.current_cloud_max_points ?? null,
          at_point_limit: nextStatus.current_cloud_at_point_limit ?? false
        });
        if (bridgeSnapshots.length > bridgeSnapshotLimit) {
          bridgeSnapshots.splice(0, bridgeSnapshots.length - bridgeSnapshotLimit);
        }
      }
      scene?.setSourceTopics(nextStatus.current_cloud_source_topic, nextStatus.stable_map_source_topic);
      scene?.updatePath(nextStatus.path, nextStatus.pose);
    },
    (connected) => {
      statusConnected.value = connected;
    }
  );
  statusClient.start();
});

watch(colorMode, (mode) => scene?.setColorMode(mode));
watch(layers, (nextLayers) => scene?.setLayers(nextLayers), { deep: true });
watch(pointSize, (size) => scene?.setPointSize(size));
watch(meshOpacity, (opacity) => scene?.setMeshOpacity(opacity));

onBeforeUnmount(() => {
  window.removeEventListener('keydown', handleKeydown);
  window.clearInterval(clockTimer);
  cloudClient?.stop();
  statusClient?.stop();
  meshClient?.stop();
  scene?.dispose();
  delete window.__MINE_SLAM_VIEWER_DEBUG__;
});
</script>

<template>
  <main class="app-shell">
    <div v-if="menuOpen" class="drawer-backdrop" @click="menuOpen = false"></div>
    <header class="dashboard-header">
      <div class="header-spacer"></div>
      <div class="brand-block">
        <strong>智能掘进感知系统</strong>
      </div>
      <div class="header-actions">
        <time>{{ currentTime }}</time>
        <button
          type="button"
          class="menu-toggle"
          :class="{ active: menuOpen }"
          :aria-expanded="menuOpen"
          aria-label="Toggle layer controls"
          title="图层控制"
          @click="menuOpen = !menuOpen"
        >
          <span></span>
          <span></span>
          <span></span>
        </button>
      </div>
    </header>

    <section class="main-layout">
      <aside class="data-panel computed-panel">
        <section class="panel-status">
          <span>状态</span>
          <strong class="status-badge" :class="statusBadgeClass(cloudOverallStatus)">
            {{ qualityText(cloudOverallStatus) }}
          </strong>
        </section>
        <section class="info-section">
          <h3>相对标靶</h3>
          <div class="metric-card">
            <span class="metric-label">X 位移</span>
            <strong class="metric-value"><b>{{ formatMm(status?.target_xy?.seen ? status?.target_xy?.dx_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">Y 位移</span>
            <strong class="metric-value"><b>{{ formatMm(status?.target_xy?.seen ? status?.target_xy?.dy_mm : undefined) }}</b><em>mm</em></strong>
          </div>
        </section>
        <section class="info-section">
          <h3>姿态角</h3>
          <div class="metric-card">
            <span class="metric-label">俯仰角</span>
            <strong class="metric-value"><b>{{ formatAngle(cloudMetric.attitude.pitch_valid ? cloudMetric.attitude.pitch_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">横滚角</span>
            <strong class="metric-value"><b>{{ formatAngle(cloudMetric.attitude.roll_valid ? cloudMetric.attitude.roll_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">偏航角</span>
            <strong class="metric-value"><b>{{ formatAngle(cloudMetric.attitude.yaw_valid ? cloudMetric.attitude.yaw_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
        </section>
        <section class="metric-grid distance-grid">
          <div class="metric-card">
            <span class="metric-label">左前</span>
            <strong class="metric-value"><b>{{ formatDistance(cloudMetric.distances.left_front_valid ? cloudMetric.distances.left_front_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">右前</span>
            <strong class="metric-value"><b>{{ formatDistance(cloudMetric.distances.right_front_valid ? cloudMetric.distances.right_front_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">左后</span>
            <strong class="metric-value"><b>{{ formatDistance(cloudMetric.distances.left_rear_valid ? cloudMetric.distances.left_rear_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">右后</span>
            <strong class="metric-value"><b>{{ formatDistance(cloudMetric.distances.right_rear_valid ? cloudMetric.distances.right_rear_mm : undefined) }}</b><em>mm</em></strong>
          </div>
        </section>
      </aside>

      <section class="scene-panel">
        <div class="pointcloud-panel">
          <div ref="sceneEl" class="scene-host"></div>
          <div class="axis-legend" aria-label="XY 坐标方向">
            <span class="axis-legend-row axis-x"><i></i><b>+X</b><em>前向</em></span>
            <span class="axis-legend-row axis-y"><i></i><b>+Y</b><em>左向</em></span>
          </div>
          <div v-if="(status?.current_cloud_points ?? 0) === 0" class="cloud-empty-state">
            <strong>等待点云数据…</strong>
            <span>请检查雷达连接或 WebSocket 状态</span>
          </div>
        </div>
      </section>

      <aside class="data-panel real-panel">
        <section class="info-section">
          <h3>惯导数据</h3>
          <div class="metric-card">
            <span class="metric-label">俯仰角</span>
            <strong class="metric-value"><b>{{ formatAngle(realMetric.attitude.pitch_valid ? realMetric.attitude.pitch_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">横滚角</span>
            <strong class="metric-value"><b>{{ formatAngle(realMetric.attitude.roll_valid ? realMetric.attitude.roll_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">偏航角</span>
            <strong class="metric-value"><b>{{ formatAngle(realMetric.attitude.yaw_valid ? realMetric.attitude.yaw_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
        </section>
        <h3 class="section-heading radar-title">毫米波雷达数据</h3>
        <section class="metric-grid distance-grid">
          <div class="metric-card">
            <span class="metric-label">左前毫米波</span>
            <strong class="metric-value"><b>{{ formatDistance(realMetric.distances.left_front_valid ? realMetric.distances.left_front_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">右前毫米波</span>
            <strong class="metric-value"><b>{{ formatDistance(realMetric.distances.right_front_valid ? realMetric.distances.right_front_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">左后毫米波</span>
            <strong class="metric-value"><b>{{ formatDistance(realMetric.distances.left_rear_valid ? realMetric.distances.left_rear_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">右后毫米波</span>
            <strong class="metric-value"><b>{{ formatDistance(realMetric.distances.right_rear_valid ? realMetric.distances.right_rear_mm : undefined) }}</b><em>mm</em></strong>
          </div>
        </section>
      </aside>

      <footer class="status-bar">
        <span class="status-item">云端 WS <strong class="status-badge" :class="badgeClass(cloudConnected)">{{ cloudConnected ? '在线' : '离线' }}</strong></span>
        <span class="status-item">状态 WS <strong class="status-badge" :class="badgeClass(statusConnected)">{{ statusConnected ? '在线' : '离线' }}</strong></span>
        <span class="status-item">Mesh WS <strong class="status-badge" :class="badgeClass(meshConnected)">{{ meshConnected ? '在线' : '离线' }}</strong></span>
        <span v-if="surfaceFrameMismatch" class="status-item">坐标系 <strong class="status-badge offline">点云/TSDF 不一致</strong></span>
        <span class="status-item">当前点云 <strong class="value-pill">{{ status?.current_cloud_source_topic ?? '—' }}</strong></span>
        <span class="status-item">地图点数 <strong class="value-pill">{{ status?.stable_map_points ?? 0 }}</strong></span>
        <span class="status-item">路径点数 <strong class="value-pill">{{ status?.status_path_point_count ?? 0 }}</strong></span>
      </footer>
    </section>

    <aside class="side-panel" :class="{ open: menuOpen }">
      <section class="drawer-header">
        <strong>显示设置</strong>
        <button type="button" aria-label="关闭设置" title="关闭" @click="menuOpen = false">×</button>
      </section>
      <section class="panel-section current-settings">
        <h2>当前渲染参数</h2>
        <div class="setting-row">
          <span>颜色模式</span>
          <strong>{{ colorModeText(colorMode) }}</strong>
        </div>
        <div class="setting-row">
          <span>点云大小</span>
          <strong>{{ pointSize.toFixed(3) }}</strong>
        </div>
        <div class="setting-row">
          <span>实时表面</span>
          <strong>{{ layers.mesh ? '显示' : '隐藏' }}</strong>
        </div>
        <div class="setting-row">
          <span>表面三角形</span>
          <strong>{{ meshTriangles }}</strong>
        </div>
        <div class="setting-row">
          <span>本地 TSDF 表面</span>
          <strong>{{ layers.tsdfMesh ? '显示' : '隐藏' }}</strong>
        </div>
        <div class="setting-row">
          <span>TSDF 三角形</span>
          <strong>{{ tsdfMeshDebug?.triangle_count ?? 0 }}</strong>
        </div>
      </section>
      <section class="panel-section mesh-diagnostics">
        <h2>Mesh 诊断</h2>
        <div class="setting-row">
          <span>点覆盖率</span>
          <strong>{{ formatPercent(meshDebug?.point_coverage) }}</strong>
        </div>
        <div class="setting-row">
          <span>Quad 通过率</span>
          <strong>{{ formatPercent(meshDebug?.quad_coverage) }}</strong>
        </div>
        <div class="setting-row">
          <span>方位匹配率</span>
          <strong>{{ formatPercent(meshDebug?.match_coverage) }}</strong>
        </div>
        <div class="setting-row">
          <span>编码保留率</span>
          <strong>{{ formatPercent(encodedRetention) }}</strong>
        </div>
        <div class="setting-row">
          <span>体素 / 点上限</span>
          <strong>{{ formatRuntimeConfig(status?.current_cloud_voxel_size_m, status?.current_cloud_max_points) }}</strong>
        </div>
        <div class="setting-row">
          <span>发布者 / 截断</span>
          <strong>{{ formatPublisherState(status?.current_cloud_publisher_count, status?.current_cloud_at_point_limit) }}</strong>
        </div>
        <div class="setting-row">
          <span>构建 / 应用</span>
          <strong>{{ formatMs(meshDebug?.mesh_build_ms) }} / {{ formatMs(meshDebug?.mesh_apply_ms) }}</strong>
        </div>
        <div class="setting-row">
          <span>数据间隔</span>
          <strong>{{ formatMs(latestMeshFrame?.inter_arrival_ms ?? undefined) }}</strong>
        </div>
        <div class="setting-row diagnostic-reason">
          <span>主要拒绝原因</span>
          <strong>{{ meshDebug?.top_reject_reason ?? 'none' }}</strong>
        </div>
        <p class="diagnostic-note">
          Console 可通过 <code>__MINE_SLAM_VIEWER_DEBUG__</code> 获取最近 180 帧 Mesh 与 Bridge 快照。
        </p>
      </section>
      <LayerPanel :layers="layers" @change="layers = $event" />
      <ColorModeSelector :mode="colorMode" @change="colorMode = $event" />
      <section class="panel-section point-size-panel">
        <h2>点云尺寸</h2>
        <label class="range-row">
          <span>点大小</span>
          <strong>{{ pointSize.toFixed(3) }}</strong>
          <input v-model.number="pointSize" type="range" min="0.01" max="0.12" step="0.005" />
        </label>
        <label class="range-row">
          <span>表面透明度</span>
          <strong>{{ meshOpacity.toFixed(2) }}</strong>
          <input v-model.number="meshOpacity" type="range" min="0.10" max="0.90" step="0.05" />
        </label>
      </section>
    </aside>
  </main>
</template>
