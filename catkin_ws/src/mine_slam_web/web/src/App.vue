<script setup lang="ts">
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue';
import ColorModeSelector from './components/ColorModeSelector.vue';
import LayerPanel from './components/LayerPanel.vue';
import type { ColorMode } from './viewer/ColorMap';
import { BinaryCloudClient } from './viewer/BinaryCloudClient';
import { SceneView, type LayerState } from './viewer/SceneView';
import { StatusClient, type EquipmentMetricState, type ViewerStatus } from './viewer/StatusClient';

const host = window.location.hostname || 'localhost';
const cloudUrl = `ws://${host}:9001/cloud`;
const statusUrl = `ws://${host}:9002/status`;

const sceneEl = ref<HTMLElement | null>(null);
const colorMode = ref<ColorMode>('height');
const layers = ref<LayerState>({ current: true, stable: true, path: true, reflector: true, grid: true });
const cloudConnected = ref(false);
const statusConnected = ref(false);
const status = ref<ViewerStatus | null>(null);
const menuOpen = ref(false);
const pointSize = ref(0.035);
const currentTime = ref('');

let scene: SceneView | null = null;
let cloudClient: BinaryCloudClient | null = null;
let statusClient: StatusClient | null = null;
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

function badgeClass(valid: boolean | undefined): string {
  if (valid === undefined) return 'unknown';
  return valid ? 'online' : 'offline';
}

function qualityText(value: string | undefined): string {
  const map: Record<string, string> = {
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
    stamp_ns: 0,
    attitude: {
      valid: false,
      roll_deg: 0,
      pitch_deg: 0,
      yaw_deg: 0,
      ground_points: 0,
      wall_points: 0
    },
    distances: {
      valid: false,
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
  window.addEventListener('keydown', handleKeydown);
  scene.onStats((stats) => {
    window.__MINE_SLAM_VIEWER_STATS__ = {
      fps: stats.fps,
      frameMs: stats.frameMs,
      currentPoints: status.value?.current_cloud_points ?? 0,
      stablePoints: status.value?.stable_map_points ?? 0,
      pathPoints: status.value?.path.length ?? 0,
      cloudConnected: cloudConnected.value,
      statusConnected: statusConnected.value
    };
  });

  cloudClient = new BinaryCloudClient(
    cloudUrl,
    (cloud) => scene?.updateCloud(cloud),
    (connected) => {
      cloudConnected.value = connected;
    }
  );
  cloudClient.start();

  statusClient = new StatusClient(
    statusUrl,
    (nextStatus) => {
      status.value = nextStatus;
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

onBeforeUnmount(() => {
  window.removeEventListener('keydown', handleKeydown);
  window.clearInterval(clockTimer);
  cloudClient?.stop();
  statusClient?.stop();
  scene?.dispose();
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
          <strong class="status-badge" :class="badgeClass(status?.cloud_estimate?.seen)">
            {{ qualityText(status?.cloud_estimate?.quality) }}
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
            <strong class="metric-value"><b>{{ formatAngle(cloudMetric.attitude.valid ? cloudMetric.attitude.pitch_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">横滚角</span>
            <strong class="metric-value"><b>{{ formatAngle(cloudMetric.attitude.valid ? cloudMetric.attitude.roll_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">偏航角</span>
            <strong class="metric-value"><b>{{ formatAngle(cloudMetric.attitude.valid ? cloudMetric.attitude.yaw_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
        </section>
        <section class="metric-grid distance-grid">
          <div class="metric-card">
            <span class="metric-label">左前</span>
            <strong class="metric-value"><b>{{ formatDistance(cloudMetric.distances.valid ? cloudMetric.distances.left_front_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">右前</span>
            <strong class="metric-value"><b>{{ formatDistance(cloudMetric.distances.valid ? cloudMetric.distances.right_front_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">左后</span>
            <strong class="metric-value"><b>{{ formatDistance(cloudMetric.distances.valid ? cloudMetric.distances.left_rear_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">右后</span>
            <strong class="metric-value"><b>{{ formatDistance(cloudMetric.distances.valid ? cloudMetric.distances.right_rear_mm : undefined) }}</b><em>mm</em></strong>
          </div>
        </section>
      </aside>

      <section class="scene-panel">
        <div class="pointcloud-panel">
          <div ref="sceneEl" class="scene-host"></div>
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
            <strong class="metric-value"><b>{{ formatAngle(realMetric.attitude.valid ? realMetric.attitude.pitch_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">横滚角</span>
            <strong class="metric-value"><b>{{ formatAngle(realMetric.attitude.valid ? realMetric.attitude.roll_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">偏航角</span>
            <strong class="metric-value"><b>{{ formatAngle(realMetric.attitude.valid ? realMetric.attitude.yaw_deg : undefined) }}</b><em class="angle-unit">度</em></strong>
          </div>
        </section>
        <h3 class="section-heading radar-title">毫米波雷达数据</h3>
        <section class="metric-grid distance-grid">
          <div class="metric-card">
            <span class="metric-label">左前毫米波</span>
            <strong class="metric-value"><b>{{ formatDistance(realMetric.distances.valid ? realMetric.distances.left_front_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">右前毫米波</span>
            <strong class="metric-value"><b>{{ formatDistance(realMetric.distances.valid ? realMetric.distances.right_front_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">左后毫米波</span>
            <strong class="metric-value"><b>{{ formatDistance(realMetric.distances.valid ? realMetric.distances.left_rear_mm : undefined) }}</b><em>mm</em></strong>
          </div>
          <div class="metric-card">
            <span class="metric-label">右后毫米波</span>
            <strong class="metric-value"><b>{{ formatDistance(realMetric.distances.valid ? realMetric.distances.right_rear_mm : undefined) }}</b><em>mm</em></strong>
          </div>
        </section>
      </aside>

      <footer class="status-bar">
        <span class="status-item">云端 WS <strong class="status-badge" :class="badgeClass(cloudConnected)">{{ cloudConnected ? '在线' : '离线' }}</strong></span>
        <span class="status-item">状态 WS <strong class="status-badge" :class="badgeClass(statusConnected)">{{ statusConnected ? '在线' : '离线' }}</strong></span>
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
      </section>
    </aside>
  </main>
</template>
