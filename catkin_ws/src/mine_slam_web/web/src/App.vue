<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref, watch } from 'vue';
import ColorModeSelector from './components/ColorModeSelector.vue';
import LayerPanel from './components/LayerPanel.vue';
import type { ColorMode } from './viewer/ColorMap';
import { BinaryCloudClient } from './viewer/BinaryCloudClient';
import { SceneView, type LayerState } from './viewer/SceneView';
import { StatusClient, type ViewerStatus } from './viewer/StatusClient';

const host = window.location.hostname || 'localhost';
const cloudUrl = `ws://${host}:9001/cloud`;
const statusUrl = `ws://${host}:9002/status`;

const sceneEl = ref<HTMLElement | null>(null);
const colorMode = ref<ColorMode>('height');
const layers = ref<LayerState>({ current: true, stable: true, path: true, reflector: true, grid: true });
const cloudConnected = ref(false);
const statusConnected = ref(false);
const status = ref<ViewerStatus | null>(null);
const menuOpen = ref(true);
const pointSize = ref(0.035);

let scene: SceneView | null = null;
let cloudClient: BinaryCloudClient | null = null;
let statusClient: StatusClient | null = null;

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
  if (document.fullscreenElement) {
    void document.exitFullscreen();
    return;
  }
  void document.documentElement.requestFullscreen?.();
}

function handleKeydown(event: KeyboardEvent): void {
  if (event.code !== 'Space' || isTypingTarget(event.target)) return;
  event.preventDefault();
  toggleFullscreen();
}

onMounted(() => {
  if (!sceneEl.value) return;
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
  cloudClient?.stop();
  statusClient?.stop();
  scene?.dispose();
});
</script>

<template>
  <main class="app-shell" :class="{ 'menu-open': menuOpen }">
    <header class="viewport-bar">
      <span>Mine SLAM Cloud View</span>
      <strong>{{ cloudConnected || statusConnected ? 'LIVE' : 'OFFLINE' }}</strong>
    </header>
    <div ref="sceneEl" class="scene-host"></div>
    <button
      type="button"
      class="menu-toggle"
      :class="{ active: menuOpen }"
      :aria-expanded="menuOpen"
      aria-label="Toggle menu"
      title="Menu"
      @click="menuOpen = !menuOpen"
    >
      <span></span>
      <span></span>
      <span></span>
    </button>
    <aside class="side-panel" :class="{ open: menuOpen }">
      <LayerPanel :layers="layers" @change="layers = $event" />
      <ColorModeSelector :mode="colorMode" @change="colorMode = $event" />
    </aside>
  </main>
</template>
