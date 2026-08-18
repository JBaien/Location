<script setup lang="ts">
import type { LayerState } from '../viewer/SceneView';

defineProps<{ layers: LayerState }>();
const emit = defineEmits<{ change: [layers: LayerState] }>();

type VisibleLayer = 'path' | 'current' | 'mesh' | 'stable';

function update(key: VisibleLayer, value: boolean, layers: LayerState) {
  emit('change', { ...layers, [key]: value });
}
</script>

<template>
  <section class="panel-section">
    <h2>图层显示</h2>
    <label class="check-row">
      <input type="checkbox" :checked="layers.path" @change="update('path', ($event.target as HTMLInputElement).checked, layers)" />
      <span>轨迹</span>
    </label>
    <label class="check-row">
      <input type="checkbox" :checked="layers.current" @change="update('current', ($event.target as HTMLInputElement).checked, layers)" />
      <span>实时点云</span>
    </label>
    <label class="check-row">
      <input type="checkbox" :checked="layers.mesh" @change="update('mesh', ($event.target as HTMLInputElement).checked, layers)" />
      <span>实时表面</span>
    </label>
    <label class="check-row">
      <input type="checkbox" :checked="layers.stable" @change="update('stable', ($event.target as HTMLInputElement).checked, layers)" />
      <span>全局地图</span>
    </label>
  </section>
</template>
