<script setup lang="ts">
import type { ColorMode } from '../viewer/ColorMap';

defineProps<{ mode: ColorMode }>();
const emit = defineEmits<{ change: [mode: ColorMode] }>();

const modes: Array<{ value: ColorMode; label: string }> = [
  { value: 'height', label: '高度' },
  { value: 'intensity', label: '强度' },
  { value: 'x-distance', label: 'X距离' },
  { value: 'y-distance', label: 'Y距离' },
  { value: 'z-distance', label: 'Z距离' },
  { value: 'lidar_id', label: '雷达编号' },
  { value: 'stable-temporary', label: '地图/实时' },
  { value: 'reflector', label: '反光点' }
];
</script>

<template>
  <section class="panel-section">
    <h2>渲染方式</h2>
    <select class="select" :value="mode" @change="emit('change', ($event.target as HTMLSelectElement).value as ColorMode)">
      <option v-for="item in modes" :key="item.value" :value="item.value">{{ item.label }}</option>
    </select>
  </section>
</template>
